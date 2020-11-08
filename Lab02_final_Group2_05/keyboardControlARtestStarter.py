from pynput.keyboard import Key, Listener, KeyCode
import cv2
import numpy as np
import cv2.aruco as aruco
from PenguinPiC import PenguinPi


class Keyboard:
    # Calibrated at 60 m/s
    LINEAR_COMPENSATION = 69 / 60
    # Taken from baseline.txt
    # BASELINE = 1.142968276500685443
    # TURN_COMPENSATION = LINEAR_COMPENSATION * BASELINE

    def __init__(self, ppi=None, forward_vel=60, turning_vel=28) -> None:
        # storage for key presses
        self.directions = [False for _ in range(4)]
        self.signal_stop = False

        # connection to PenguinPi robot
        self.ppi = ppi

        # [Left, Right]
        self.wheel_vels = [0, 0]
        self.wheel_vel_forward = forward_vel
        self.wheel_vel_turning = turning_vel
        self.key_pressed = None
        self.run_yolo = False
        print("Keyboard: YOLO is off! Turn on using 'y'")

        self.listener = Listener(
            on_press=self.on_press, on_release=self.on_release
        ).start()

    def trigger_state(self, key: Key, activate: bool) -> None:
        """Set different states of robot's motion
        Args:
            key (Key): keyboard input
            activate (bool): turn state on/off
        """

        if key == Key.space:
            self.signal_stop = activate

        try:
            if key == Key.up:
                self.directions[0] = activate

            elif key == Key.down:
                self.directions[1] = activate

            elif key == Key.left:
                self.directions[2] = activate

            elif key == Key.right:
                self.directions[3] = activate

            elif str(key.char).lower() == "y" and activate:
                self.run_yolo = not self.run_yolo
                if self.run_yolo:
                    print("Keyboard: YOLO On")
                else:
                    print("Keyboard: YOLO Off")

        except AttributeError:
            pass

    def set_target(self, key: Key) -> None:
        """Increase/decrease target velocities
        Args:
            key (Key): keyboard input
        """
        try:
            if str(key.char).lower() == "w":
                if self.wheel_vel_forward < 100:
                    self.wheel_vel_forward += 2

            elif str(key.char).lower() == "s":
                if self.wheel_vel_forward > 0:
                    self.wheel_vel_forward -= 2

            elif str(key.char).lower() == "a":
                if self.wheel_vel_turning > 0:
                    self.wheel_vel_turning -= 2

            elif str(key.char).lower() == "d":
                if self.wheel_vel_turning < 100:
                    self.wheel_vel_turning += 2
        except AttributeError:
            pass

    def on_press(self, key: Key) -> None:
        """Function call when keys are pressed
        Args:
            key (Key): keyboard input
        """
        self.key_pressed = key
        self.trigger_state(key, True)
        self.set_target(key)
        # Request robot to move
        self.send_drive_signal()

    def on_release(self, key: Key) -> None:
        """Function call when keys are released
        Args:
            key (Key): keyboard input
        """
        self.trigger_state(key, False)

    # Should be called calculate_drive_signal tbh
    def get_drive_signal(self) -> list:
        """Determine target velocities
        Returns:
            left_target: Target velocity for left wheel
            right_target: Target velocity for right wheel
        """

        # Update speed - Check the current state of the robot movement so we can update the wheel velocity from the speed adjustments accordingly
        [left_target, right_target, _] = self.latest_drive_signal()

        if self.signal_stop == True:
            left_target = 0
            right_target = 0
            self.wheel_vel_forward = 60
            self.wheel_vel_turning = 28
            self.signal_stop = False
        else:
            # Update direction - check within directions array and adjust left and right wheel velocity accordingly
            for index, direction in enumerate(self.directions):
                if direction == True:
                    if index == 0:
                        # Forward
                        left_target = self.wheel_vel_forward * self.LINEAR_COMPENSATION
                        right_target = self.wheel_vel_forward
                    elif index == 1:
                        # Backward
                        left_target = -(
                            self.wheel_vel_forward * self.LINEAR_COMPENSATION
                        )
                        right_target = -self.wheel_vel_forward
                    elif index == 2:
                        # Turn Left
                        # left_target = -(self.wheel_vel_turning + self.TURN_COMPENSATION)
                        left_target = -self.wheel_vel_turning
                        right_target = self.wheel_vel_turning
                    elif index == 3:
                        # Turn Right
                        # left_target = self.wheel_vel_turning + self.TURN_COMPENSATION
                        left_target = -self.wheel_vel_turning
                        right_target = -self.wheel_vel_turning

        # Latching onto states
        if left_target > right_target * self.LINEAR_COMPENSATION:
            # left_target = self.wheel_vel_turning + self.TURN_COMPENSATION
            left_target = self.wheel_vel_turning
            right_target = -self.wheel_vel_turning
        elif left_target < right_target * self.LINEAR_COMPENSATION:
            # left_target = -(self.wheel_vel_turning + self.TURN_COMPENSATION)
            left_target = -self.wheel_vel_turning
            right_target = self.wheel_vel_turning
        elif left_target > 0 and right_target > 0:
            left_target = self.wheel_vel_forward * self.LINEAR_COMPENSATION
            right_target = self.wheel_vel_forward
        elif left_target < 0 and right_target < 0:
            left_target = -self.wheel_vel_forward * self.LINEAR_COMPENSATION
            right_target = -self.wheel_vel_forward

        # Convert to int if not request fails
        left_target = int(round(left_target))
        # print(f"L_VEL:{left_target:4.0f} R_VEL:{right_target:3.0f}")
        return left_target, right_target

    def send_drive_signal(self) -> None:
        """Send drive signal to PenguinPi"""
        if self.ppi is not None:
            l_target, r_target = self.get_drive_signal()
            self.ppi.set_velocity(l_target, r_target)
            self.wheel_vels = [l_target, r_target]

    def latest_drive_signal(self) -> list:
        """
        Returns:
            list[int, int, Key]: Get drive signal from PenguinPi, store as member variables
        """
        l, r = self.wheel_vels
        k = self.key_pressed

        # Remove LINEAR_COMPENSATION before telling the our robot what the current speeds are
        # LINEAR_COMPENSATION is meant to help robot drive straight in Gazebo!
        if k == Key.up or k == Key.down:
            l = l / self.LINEAR_COMPENSATION
        # elif k == Key.left or k == Key.right:
        #     l = l - self.TURN_COMPENSATION
        return l, r, k
