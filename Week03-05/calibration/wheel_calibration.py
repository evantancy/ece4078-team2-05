import numpy as np

import os
import sys
# Import PenguinPiC
os.chdir("../")
sys.path.insert(0, "{}".format(os.getcwd()))
import PenguinPiC


def calibrateWheelRadius(ppi):
    # Compute the robot scale parameter using a range of wheel velocities.
    # For each wheel velocity, the robot scale parameter can be computed
    # by comparing the time and distance driven to the input wheel velocities.

    # wheel_velocities = range(20, 81, 15)
    wheel_velocities = [60]
    delta_times = []

    for wheel_vel in wheel_velocities:
        print("Driving at {} ticks/s.".format(wheel_vel))

        # Repeat the test until the correct time is found.
        while True:
            delta_time = input("Input the time to drive in seconds: ")
            try:
                delta_time = float(delta_time)
            except ValueError:
                print("Time must be a number.")
                continue

            # Drive the robot at the given speed for the given time
            ppi.set_velocity(wheel_vel, wheel_vel, delta_time)

            uInput = input("Did the robot travel 1m?[y/N]")
            if uInput == 'y':
                delta_times.append(delta_time)
                print("Recording that the robot drove 1m in {:.2f} seconds at wheel speed {}.\n".format(delta_time,
                                                                                                        wheel_vel))
                break

    # Once finished driving, compute the scale parameter by averaging
    num = len(wheel_velocities)
    scale = 0
    for delta_time, wheel_vel in zip(delta_times, wheel_velocities):
        scale += 1 / num * (1 / (wheel_vel * delta_time))
    print("The scale parameter is estimated as {:.5f} m/ticks.".format(scale))

    return scale


def calibrateBaseline(ppi, scale):
    # Compute the robot basline parameter using a range of wheel velocities.
    # For each wheel velocity, the robot baseline parameter can be computed by
    # comparing the time elapsed and rotation completed to the input wheel
    # velocities to find out the distance between the wheels.

    # Feel free to change the range / step
    wheel_velocities = [20]#range(30, 60, 10)
    delta_times = []

    for wheel_vel in wheel_velocities:
        print("Driving at {} ticks/s.".format(wheel_vel))

        # Repeat the test until the correct time is found.
        while True:
            delta_time = input("Input the time to drive in seconds: ")
            try:
                delta_time = float(delta_time)
            except ValueError:
                print("Time must be a number.")
                continue

            # Spin the robot at the given speed for the given time
            ppi.set_velocity(-wheel_vel, wheel_vel, delta_time)

            uInput = input("Did the robot spin 360deg?[y/N]")
            if uInput == 'y':
                delta_times.append(delta_time)
                print("Recording that the robot spun 360deg in {:.2f} seconds at wheel speed {}.\n".format(delta_time,
                                                                                                           wheel_vel))
                break

    # Once finished driving, compute the basline parameter by averaging
    num = len(wheel_velocities)
    baseline = 0
    # TODO: compute baseline parameter
    # ------------------------------------------
    for delta_time, wheel_vel in zip(delta_times, wheel_velocities):
        baseline += 1 / num * (scale * wheel_vel * delta_time)/np.pi
    # ------------------------------------------
    print("The baseline parameter is estimated as {:.2f} m.".format(baseline))

    return baseline


if __name__ == "__main__":
    # calibrate pibot scale and baseline
    dataDir = "{}/calibration/wheel_calibration/".format(os.getcwd())

    ppi = PenguinPiC.PenguinPi()

    print('Calibrating PiBot scale...\n')
    scale = calibrateWheelRadius(ppi)
    fileNameS = "{}scale.txt".format(dataDir)
    np.savetxt(fileNameS, np.array([scale]), delimiter=',')

    print('Calibrating PiBot baseline...\n')
    baseline = calibrateBaseline(ppi, scale)
    fileNameB = "{}baseline.txt".format(dataDir)
    np.savetxt(fileNameB, np.array([baseline]), delimiter=',')

    print('Finished calibration')
