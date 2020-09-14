import csv
import cv2
import os
import sys
import numpy as np
import time
import PenguinPiC

import keyboardControlARtestStarter as Keyboard
from GazeboServiceCaller import GazeboServiceCaller
from utils import load_yaml, load_csv, check_folder, check_file
from pathlib import Path

collection_cfg = load_yaml("master_config.yaml")["collection"]
model_index = collection_cfg["label_select"]
model = collection_cfg["labels"][model_index]


class DatasetWriter:
    """Save images and a csv file with each row being [image_label,image_file_name], FOR EACH SPECIFIC MODEL"""

    def __init__(self, image_count: int):
        # Folder structure M3/dataset/model_name
        model = collection_cfg["labels"][model_index]
        self.model = model

        self.folder = Path("dataset") / model
        check_folder(self.folder, create_folder=True)

        labels_file = self.folder / "labels.csv"
        self.file_object = open(labels_file, "a")

    def __del__(self):
        """Finalizer. Called when object is garbage collected"""
        self.file_object.close()

    def write_image(self):
        """Save image with specified name"""
        image_name = self.folder / str(image_count) / ".png"
        row = [self.model, image_name]

        csv.writer(self.file_object).writerow(row)
        self.file_object.flush()

        ppi = PenguinPiC.PenguinPi()
        image = ppi.get_image()
        cv2.imwrite(image_name, image)


if __name__ == "__main__":
    print(f"Exit program using CTRL + \\ [SIGQUIT]")
    ppi = PenguinPiC.PenguinPi()
    # Initialise model service callers
    sheep1_caller = GazeboServiceCaller("sheep1")
    sheep1_caller.set_model([0, 6, 0], [0, 0, 0])

    sheep2_caller = GazeboServiceCaller("sheep2")
    sheep2_caller.set_model([1, 6, 0], [0, 0, 0])

    coke1_caller = GazeboServiceCaller("Coke1")
    coke1_caller.set_model([2, 6, 0], [0, 0, 0])

    coke2_caller = GazeboServiceCaller("Coke2")
    coke2_caller.set_model([3, 6, 0], [0, 0, 0])

    ppi_caller = GazeboServiceCaller("PenguinPi")
    (init_position, init_orientation) = ppi_caller.return_model_state()
    ppi_caller.set_model([0, 0, 0.012793], [-0.007493, -0.055376, 0])

    # specify number of x,y positions of penguinpi
    positions = collection_cfg["ppi"]["n_positions"]

    # specify number of theta positions for ppi
    ppi_rotations = collection_cfg["ppi"]["n_rotations"]
    d_th_ppi = 2 * np.pi / ppi_rotations

    # specify number of rotations about z-axis for model
    n_rotations = collection_cfg["object"]["n_rotations"]
    d_theta = 2 * np.pi / n_rotations

    # specify possible model positions
    phi = np.linspace(-0.6, 0.6, 3)
    model_dist = np.linspace(0.7, 3, 6)
    fov = 1.0855

    take_images = collection_cfg["taking_images"]
    csv_file = collection_cfg["ppi_csv"]
    # get an array of suitable x,y positions of ppi
    if not take_images:
        x_min = collection_cfg["x_min"]
        x_max = collection_cfg["x_max"]
        y_min = collection_cfg["y_min"]
        y_max = collection_cfg["y_max"]
        n_steps = collection_cfg["n_steps"]

        csv_path = Path(collection_cfg["ppi_csv"])
        if check_file(csv_path):
            check = input("Empty .csv before writing?[y/n]")
            if check.lower() == "y":
                check_file(csv_path, delete=True)
        with open(csv_file, "a") as coordinates_file:
            for ppi_x in np.linspace(x_min, x_max, n_steps):
                for ppi_y in np.linspace(y_min, y_max, n_steps):
                    ppi_caller.set_model(
                        [ppi_x, ppi_y, 0.012793], [-0.007493, -0.055376, 0]
                    )
                    check = input("Is ppi position ok?[y/n]\n")
                    if check.lower() == "y":
                        xy = [ppi_x, ppi_y]
                        csv.writer(coordinates_file).writerow(xy)
                        print(f"Saved x:{ppi_x}y:{ppi_y} in ppi_coordinates.csv")
                    elif check.lower() == "q":
                        break
                if check.lower() == "q":
                    break
        coordinates_file.close()
    elif take_images:
        coordinate_data = load_csv(csv_file, np.float32)
        ax = []
        ay = []
        for row in coordinate_data:
            ax.append(row[0])
            ay.append(row[1])
        # image taking section
        if model == "sheep":
            phi = np.linspace(-0.5, 0.5, 3)
            model_dist = [0.55, 0.65, 0.8, 1, 2, 3]
            image_count = 0
            ppi_pos = np.zeros(0)
            ppi_ori = np.zeros(0)
            m_pos = np.zeros(0)
            m_ori = np.zeros(0)
            for a in range(len(ax)):
                ppi_x = ax[a]
                ppi_y = ay[a]
                ppi_caller.set_model(
                    [ppi_x, ppi_y, 0.012793], [-0.007493, -0.055376, 0]
                )
                for i in range(ppi_rotations):
                    ppi_angle = i * d_th_ppi
                    ppi_caller.set_model(
                        [ppi_x, ppi_y, 0.012793], [-0.007493, -0.055376, ppi_angle]
                    )
                    check = input("Is ppi position ok?[y/n]\n")
                    if check.lower() == "y":
                        for w in phi:
                            for d in model_dist:
                                mdl_x = ppi_x + d * np.cos(ppi_angle + w * fov / 2)
                                mdl_y = ppi_y + d * np.sin(ppi_angle + w * fov / 2)
                                sheep1_caller.set_model([mdl_x, mdl_y, 0], [0, 0, 0])
                                check = input("Is model position ok?[y/n]\n")
                                if check.lower() == "y":
                                    for j in range(n_rotations):
                                        sheep1_caller.set_model(
                                            [mdl_x, mdl_y, 0], [0, 0, j * d_theta]
                                        )
                                        data = DatasetWriter(image_count)

                                        # pos1,ori1 = ppi_caller.return_model_state()
                                        # ppi_pos = np.append(ppi_pos,[pos1])
                                        # ppi_ori = np.append(ppi_ori,[ori1])
                                        # pos2,ori2 = sheep1_caller.return_model_state()
                                        # m_pos = np.append(m_pos,[pos2])
                                        # m_ori = np.append(m_ori,[ori2])
                                        # np.savetxt('dataset/sheep/ppi_pos.txt',ppi_pos,fmt = '%f')
                                        # np.savetxt('dataset/sheep/ppi_ori.txt',ppi_ori,fmt = '%f')
                                        # np.savetxt('dataset/sheep/m_pos.txt',m_pos,fmt = '%f')
                                        # np.savetxt('dataset/sheep/m_ori.txt',m_ori,fmt = '%f')

                                        data.write_image()
                                        print("Collected image No.", image_count)
                                        image_count += 1
                    elif check.lower() == "q":
                        continue
            print("Finished data collection.\n")

    if take_images and coke:
        phi = np.linspace(-0.5, 0.5, 3)
        model_dist = [0.35, 0.5, 0.75, 1, 2, 3]
        image_count = 0
        ppi_pos = np.zeros(0)
        ppi_ori = np.zeros(0)
        m_pos = np.zeros(0)
        m_ori = np.zeros(0)
        # load arrays of suitable x,y positions of ppi
        ax = np.loadtxt("ax.txt", dtype=float)
        ay = np.loadtxt("ay.txt", dtype=float)
        for a in range(len(ax)):
            ppi_x = ax[a]
            ppi_y = ay[a]
            ppi_caller.set_model([ppi_x, ppi_y, 0.012793], [-0.007493, -0.055376, 0])
            for i in range(ppi_rotations):
                ppi_angle = i * d_th_ppi
                ppi_caller.set_model(
                    [ppi_x, ppi_y, 0.012793], [-0.007493, -0.055376, ppi_angle]
                )
                check = input("Is ppi position ok?[y/n]\n")
                if check.lower() == "y":
                    for w in phi:
                        for d in model_dist:
                            mdl_x = ppi_x + d * np.cos(ppi_angle + w * fov / 2)
                            mdl_y = ppi_y + d * np.sin(ppi_angle + w * fov / 2)
                            coke1_caller.set_model([mdl_x, mdl_y, 0], [0, 0, 0])
                            check = input("Is model position ok?[y/n]\n")
                            if check.lower() == "y":

                                for j in range(n_rotations):
                                    coke1_caller.set_model(
                                        [mdl_x, mdl_y, 0], [0, 0, j * d_theta]
                                    )
                                    annot = 1  # annot = inputNumber('Label this image by entering 0, 1, or 2 (0 = sheep, 1 = coke, 2 = neither):\n')
                                    data = DatasetWriter(image_count)

                                    # pos1,ori1 = ppi_caller.return_model_state()
                                    # ppi_pos = np.append(ppi_pos,[pos1])
                                    # ppi_ori = np.append(ppi_ori,[ori1])
                                    # pos2,ori2 = coke1_caller.return_model_state()
                                    # m_pos = np.append(m_pos,[pos2])
                                    # m_ori = np.append(m_ori,[ori2])
                                    # np.savetxt('dataset/coke/ppi_pos.txt',ppi_pos,fmt = '%f')
                                    # np.savetxt('dataset/coke/ppi_ori.txt',ppi_ori,fmt = '%f')
                                    # np.savetxt('dataset/coke/m_pos.txt',m_pos,fmt = '%f')
                                    # np.savetxt('dataset/coke/m_ori.txt',m_ori,fmt = '%f')

                                    data.write_image()
                                    print("Collected image No.", image_count)
                                    image_count += 1
        print("Finished data collection.\n")

    if take_images and neither:
        ppi_rotations = 120
        d_th_ppi = 2 * np.pi / ppi_rotations
        image_count = 0
        ppi_pos = np.zeros(0)
        ppi_ori = np.zeros(0)
        # load arrays of suitable x,y positions of ppi
        ax = np.loadtxt("ax.txt", dtype=float)
        ay = np.loadtxt("ay.txt", dtype=float)
        for a in range(len(ax)):
            ppi_x = ax[a]
            ppi_y = ay[a]
            ppi_caller.set_model([ppi_x, ppi_y, 0.012793], [-0.007493, -0.055376, 0])
            for i in range(ppi_rotations):
                ppi_angle = i * d_th_ppi
                ppi_caller.set_model(
                    [ppi_x, ppi_y, 0.012793], [-0.007493, -0.055376, ppi_angle]
                )
                annot = 2  # annot = inputNumber('Label this image by entering 0, 1, or 2 (0 = sheep, 1 = coke, 2 = neither):\n')
                data = DatasetWriter(image_count)

                # pos1,ori1 = ppi_caller.return_model_state()
                # ppi_pos = np.append(ppi_pos,[pos1])
                # ppi_ori = np.append(ppi_ori,[ori1])
                # np.savetxt('dataset/neither/ppi_pos.txt',ppi_pos,fmt = '%f')
                # np.savetxt('dataset/neither/ppi_ori.txt',ppi_ori,fmt = '%f')

                data.write_image()
                print("Collected image No.", image_count)
                image_count += 1
        print("Finished data collection.\n")

    #####CODE WE'LL NEED LATER TO CALL ROBOT/MODEL POSE:#####
    # hello = np.loadtxt('ppi_pos.txt',dtype=float)
    # hello = hello.reshape(int(len(hello)/3),3)
