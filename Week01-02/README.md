# Week 1-2 Instructions

## Introduction
We will use the [PenguinPi robot](https://cirrusrobotics.com.au/products/penguinpi/) in the [Gazebo simulator](http://gazebosim.org/) for our lab project. 

[ECE4078LabProject.pdf](ECE4078LabProject.pdf) provides an overview of the lab project and the six marking milestones.

## Objectives
1. Get an overview of the lab project.
2. Set up your environment locally or on cloud-based platforms.
3. Implement keyboard teleoperation of the robot.

## Marking schemes
- Teleoperation (80pts): 60pts for implementing robot teleoperation using arrow keys and space key; 20pts for adding additional controls.
- Interface (20pts): 15pts for showing a live camera feed; 5pts for displaying additional information.

## Getting-started
1. [GazeboPenguinPiStetup.md](GazeboPenguinPiStetup.md) provides instructions on setting up the simulator environment on cloud-based platforms or locally, as well as connecting to the physical robot.
2. [test_camera_motors.py](test_camera_motors.py) rotates the robot's wheels and shows an image captured by its camera. If you have set up your environment correctly, you should be able to run this script to see the robot inside the simulator moving and its camera input.
3. [penguinPiC.py](penguinPiC.py) provides functions needed for accessing the robot's wheels and camera through a webserver.
4. [keyboardControlStarter.py](keyboardControlStarter.py) provides skeleton codes that you may start your implementation of keyboard teleoperation with. Feel free to write your own codes.
