# Save and label images for training your neural network
# You may move the robot around by dragging it inside Gazebo
# or by running your keyboard teleoperation function

import csv
import cv2
import os
import sys
import numpy as np
import time

import PenguinPiC

ppi = PenguinPiC.PenguinPi()

import keyboardControlARtestStarter as Keyboard
from GazeboServiceCaller import GazeboServiceCaller

# save images and a csv file with each row being [image_label,image_file_name]
class DatasetWriter:
    def __init__(self, annot, image_count,model):
        self.folder = "dataset/"+model
        if not os.path.exists(self.folder):
            os.makedirs(self.folder)

        img_fname = self.folder + "labels.csv"
        self.img_f = open(img_fname, "a")
        self.img_fc = csv.writer(self.img_f)

    def __del__(self):
        self.img_f.close()

    def write_image(self):
        labels = ["sheep", "coke", "neither"]
        img_fname = self.folder + str(image_count) + ".png"
        row = [labels[annot], img_fname]

        self.img_fc.writerow(row)
        self.img_f.flush()

        image = ppi.get_image()
        cv2.imwrite(img_fname, image)


# get inputs for labeling
def inputNumber(message):
    while True:
        try:
            userInput = int(input(message))
        except ValueError:
            print("Not an integer! Try again.")
            continue
        else:
            return userInput
            break


# main program
if __name__ == "__main__":
    #Initialise model service callers
    sheep1_caller = GazeboServiceCaller("sheep1")
    sheep1_caller.set_model([0,6,0], [0,0,0])

    sheep2_caller = GazeboServiceCaller("sheep2")
    sheep2_caller.set_model([1,6,0], [0,0,0])

    coke1_caller = GazeboServiceCaller("Coke1")
    coke1_caller.set_model([2,6,0], [0,0,0])

    coke2_caller = GazeboServiceCaller("Coke2")
    coke2_caller.set_model([3,6,0], [0,0,0])

    ppi_caller = GazeboServiceCaller("PenguinPi")
    ppi_caller.set_model([0, 0,0.012793], [-0.007493,-0.055376,0])

    #specify number of x,y positions of penguinpi
    positions = 10

    #specify number of theta positions for ppi
    ppi_rotations = 6
    d_th_ppi = 2 * np.pi / ppi_rotations

    #specify number of rotations about z-axis for model
    n_rotations = 30
    d_theta = 2 * np.pi / n_rotations

    #specify possible model positions
    phi = np.linspace(-0.6,0.6,3)
    model_dist = np.linspace(0.7,3,6)
    fov = 1.0855

    take_images = True

    #get an array of suitable x,y positions of ppi
    if not take_images:
        x_ppi = np.zeros(0)
        y_ppi = np.zeros(0)
        for ppi_x in np.linspace(-2,3,21):
            for ppi_y in np.linspace(0,5,21):
                ppi_caller.set_model([ppi_x, ppi_y,0.012793], [-0.007493,-0.055376,0])
                check = input("Is ppi position ok?[y/n]\n")
                if check.lower() == "y":
                    x_ppi = np.append(x_ppi,ppi_x)
                    y_ppi = np.append(y_ppi,ppi_y)
                    np.savetxt('ax.txt',x_ppi,fmt = '%f')
                    np.savetxt('ay.txt',y_ppi,fmt = '%f')

    #choose model
    sheep = False
    coke = False
    neither = True

    #image taking section
    if take_images and sheep:
        phi = np.linspace(-0.5,0.5,3)
        model_dist = [0.55,0.65,0.8,1,2,3]
        image_count = 0
        ppi_pos = np.zeros(0)
        ppi_ori = np.zeros(0)
        m_pos = np.zeros(0)
        m_ori = np.zeros(0)
        #load arrays of suitable x,y positions of ppi
        ax = np.loadtxt('ax.txt',dtype=float)
        ay = np.loadtxt('ay.txt',dtype=float)
        for a in range(len(ax)):
            ppi_x = ax[a]
            ppi_y = ay[a]
            ppi_caller.set_model([ppi_x, ppi_y,0.012793], [-0.007493,-0.055376,0])
            for i in range(ppi_rotations):
                ppi_angle = i*d_th_ppi
                ppi_caller.set_model([ppi_x, ppi_y,0.012793], [-0.007493,-0.055376,ppi_angle])
                check = input("Is ppi position ok?[y/n]\n")
                if check.lower() == "y":
                    for w in phi:
                        for d in model_dist:
                            mdl_x = ppi_x + d*np.cos(ppi_angle+w*fov/2)
                            mdl_y = ppi_y + d*np.sin(ppi_angle+w*fov/2)
                            sheep1_caller.set_model([mdl_x,mdl_y,0], [0,0,0])
                            check = input("Is model position ok?[y/n]\n")
                            if check.lower() == "y":
                                for j in range(n_rotations):
                                    sheep1_caller.set_model([mdl_x,mdl_y,0], [0,0,j*d_theta])
                                    annot = 0 #annot = inputNumber('Label this image by entering 0, 1, or 2 (0 = sheep, 1 = coke, 2 = neither):\n')
                                    data = DatasetWriter(annot, image_count,"sheep/")

                                    #pos1,ori1 = ppi_caller.return_model_state()
                                    #ppi_pos = np.append(ppi_pos,[pos1])
                                    #ppi_ori = np.append(ppi_ori,[ori1])
                                    #pos2,ori2 = sheep1_caller.return_model_state()
                                    #m_pos = np.append(m_pos,[pos2])
                                    #m_ori = np.append(m_ori,[ori2])
                                    #np.savetxt('dataset/sheep/ppi_pos.txt',ppi_pos,fmt = '%f')
                                    #np.savetxt('dataset/sheep/ppi_ori.txt',ppi_ori,fmt = '%f')
                                    #np.savetxt('dataset/sheep/m_pos.txt',m_pos,fmt = '%f')
                                    #np.savetxt('dataset/sheep/m_ori.txt',m_ori,fmt = '%f')

                                    data.write_image()
                                    print("Collected image No.", image_count)
                                    image_count += 1
        print("Finished data collection.\n")

    if take_images and coke:
        model_dist = [0.35,0.5,0.75,1,2,3]
        image_count = 0
        ppi_pos = np.zeros(0)
        ppi_ori = np.zeros(0)
        m_pos = np.zeros(0)
        m_ori = np.zeros(0)
        #load arrays of suitable x,y positions of ppi
        ax = np.loadtxt('ax.txt',dtype=float)
        ay = np.loadtxt('ay.txt',dtype=float)
        for a in range(len(ax)):
            ppi_x = ax[a]
            ppi_y = ay[a]
            ppi_caller.set_model([ppi_x, ppi_y,0.012793], [-0.007493,-0.055376,0])
            for i in range(ppi_rotations):
                ppi_angle = i*d_th_ppi
                ppi_caller.set_model([ppi_x, ppi_y,0.012793], [-0.007493,-0.055376,ppi_angle])
                check = input("Is ppi position ok?[y/n]\n")
                if check.lower() == "y":
                    for w in phi:
                        for d in model_dist:
                            mdl_x = ppi_x + d*np.cos(ppi_angle+w*fov/2)
                            mdl_y = ppi_y + d*np.sin(ppi_angle+w*fov/2)
                            coke1_caller.set_model([mdl_x,mdl_y,0], [0,0,0])
                            check = input("Is model position ok?[y/n]\n")
                            if check.lower() == "y":

                                for j in range(n_rotations):
                                    coke1_caller.set_model([mdl_x,mdl_y,0], [0,0,j*d_theta])
                                    annot = 1 #annot = inputNumber('Label this image by entering 0, 1, or 2 (0 = sheep, 1 = coke, 2 = neither):\n')
                                    data = DatasetWriter(annot, image_count,"coke/")

                                    #pos1,ori1 = ppi_caller.return_model_state()
                                    #ppi_pos = np.append(ppi_pos,[pos1])
                                    #ppi_ori = np.append(ppi_ori,[ori1])
                                    #pos2,ori2 = coke1_caller.return_model_state()
                                    #m_pos = np.append(m_pos,[pos2])
                                    #m_ori = np.append(m_ori,[ori2])
                                    #np.savetxt('dataset/coke/ppi_pos.txt',ppi_pos,fmt = '%f')
                                    #np.savetxt('dataset/coke/ppi_ori.txt',ppi_ori,fmt = '%f')
                                    #np.savetxt('dataset/coke/m_pos.txt',m_pos,fmt = '%f')
                                    #np.savetxt('dataset/coke/m_ori.txt',m_ori,fmt = '%f')

                                    data.write_image()
                                    print("Collected image No.", image_count)
                                    image_count += 1
        print("Finished data collection.\n")

    if take_images and neither:
        ppi_rotations = 12
        d_th_ppi = 2 * np.pi / ppi_rotations
        image_count = 0
        ppi_pos = np.zeros(0)
        ppi_ori = np.zeros(0)
        #load arrays of suitable x,y positions of ppi
        ax = np.loadtxt('ax.txt',dtype=float)
        ay = np.loadtxt('ay.txt',dtype=float)
        for a in range(len(ax)):
            ppi_x = ax[a]
            ppi_y = ay[a]
            ppi_caller.set_model([ppi_x, ppi_y,0.012793], [-0.007493,-0.055376,0])
            for i in range(ppi_rotations):
                ppi_angle = i*d_th_ppi
                ppi_caller.set_model([ppi_x, ppi_y,0.012793], [-0.007493,-0.055376,ppi_angle])
                annot = 2 #annot = inputNumber('Label this image by entering 0, 1, or 2 (0 = sheep, 1 = coke, 2 = neither):\n')
                data = DatasetWriter(annot, image_count,"neither/")

                #pos1,ori1 = ppi_caller.return_model_state()
                #ppi_pos = np.append(ppi_pos,[pos1])
                #ppi_ori = np.append(ppi_ori,[ori1])
                #np.savetxt('dataset/neither/ppi_pos.txt',ppi_pos,fmt = '%f')
                #np.savetxt('dataset/neither/ppi_ori.txt',ppi_ori,fmt = '%f')

                data.write_image()
                print("Collected image No.", image_count)
                image_count += 1
        print("Finished data collection.\n")

    #####CODE WE'LL NEED LATER TO CALL ROBOT/MODEL POSE:#####
    #hello = np.loadtxt('ppi_pos.txt',dtype=float)
    #hello = hello.reshape(int(len(hello)/3),3)

