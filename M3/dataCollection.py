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
    def __init__(self, annot, image_count):
        self.folder = "dataset/"
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
    # collect data
    # Keyboard teleoperation components
    # keyboard_control = Keyboard.Keyboard(ppi)
    # model_name = str(input("Specify model"))
    position = inputNumber("How many positions to take pictures of robot?:\n")

    n_rotations = 10
    angle = 2 * 3.14 / n_rotations

    for i in range(position):
        ppi_caller = GazeboServiceCaller("PenguinPi")
        ppi_caller.set_model([i, 0], 0)
        check = input("Is robot in right position[y/n]\n")
        if check.lower() == "y":
            sheep_caller = GazeboServiceCaller("sheep1")
            sheep_caller.set_model([i + 1, 0], 0)
            check = input("Is sheep in right position[y/n]\n")
            if check.lower() == "y":
                for j in range(n_rotations):
                    sheep_caller.set_model([i + 1, 0], j * angle)
                    time.sleep(5)
                    # annot = inputNumber('Label this image by entering 0, 1, or 2 (0 = sheep, 1 = coke, 2 = neither):\n')
                    annot = 0
                    image_count = i * n_rotations + j
                    data = DatasetWriter(annot, image_count)
                    data.write_image()
                    print("Collected image No.", image_count)
    print("Finished data collection.\n")

