from pynput.keyboard import Key, Listener, KeyCode

class Keyboard:
    def __init__(self, ppi=None, forward_vel=60, turning_vel=28):
        # storage for key presses
        self.directions = [False for _ in range(4)]
        self.signal_stop = False

        # connection to PenguinPi robot
        self.ppi = ppi

        # [Left, Right]
        self.wheel_vels = [0, 0]
        self.wheel_vel_forward = forward_vel
        self.wheel_vel_turning = turning_vel
        self.key_pressed = 'NONE'

        self.listener = Listener(on_press=self.on_press, on_release=self.on_release).start()

    def trigger_state(self, key, activate):
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
        except AttributeError:
            pass


    def set_target(self, key):
        """Increase/decrease target velocities
        Args:
            key (Key): keyboard input
        """
        try:
            if str(key.char).lower() == 'w':
                if self.wheel_vel_forward < 300:
                    self.wheel_vel_forward += 5

            elif str(key.char).lower() == 's':
                if self.wheel_vel_forward > 0:
                    self.wheel_vel_forward -= 5

            elif str(key.char).lower() == 'a':
                if self.wheel_vel_turning > 0:
                    self.wheel_vel_turning -= 2

            elif str(key.char).lower() == 'd':
                if self.wheel_vel_turning < 100:
                    self.wheel_vel_turning += 2
        except AttributeError:
            pass

    def on_press(self, key):
        """Function call when keys are pressed
        Args:
            key (Key): keyboard input
        """
        self.key_pressed = key
        self.trigger_state(key, True)
        self.set_target(key)
        # Request robot to move
        self.send_drive_signal()

    def on_release(self, key):
        """Function call when keys are released
        Args:
            key (Key): keyboard input
        """
        self.trigger_state(key, False)

    def get_drive_signal(self):
        """Determine target velocities
        Returns:
            left_target: Target velocity for left wheel
            right_target: Target velocity for right wheel
        """

        """Update speed - Check the current state of the robot movement so we can update the wheel velocity from the speed adjustments accordingly"""
        [left_target,right_target, _ ] = self.latest_drive_signal()

        if left_target > right_target:
            left_target = self.wheel_vel_turning
            right_target = -self.wheel_vel_turning
        elif left_target < right_target:
            left_target = -self.wheel_vel_turning
            right_target = self.wheel_vel_turning
        elif left_target > 0 and right_target > 0:
            left_target = self.wheel_vel_forward
            right_target = self.wheel_vel_forward
        elif left_target < 0 and right_target < 0:
            left_target = -self.wheel_vel_forward
            right_target = -self.wheel_vel_forward

        if self.signal_stop == True:
            left_target = 0
            right_target = 0
            self.wheel_vel_forward = 60
            self.wheel_vel_turning = 28
            self.signal_stop = False
        else:
            """Update direction - check within directions array and adjust left and right wheel velocity accordingly"""
            for index, direction in enumerate(self.directions):
                if direction == True:
                    if index == 0:
                        # Forward
                        left_target = self.wheel_vel_forward
                        right_target = self.wheel_vel_forward

                    elif index == 1:
                        # Backward
                        left_target = -self.wheel_vel_forward
                        right_target = -self.wheel_vel_forward

                    elif index == 2:
                        # Turn Left
                        left_target = -self.wheel_vel_turning
                        right_target = self.wheel_vel_turning

                    elif index == 3:
                        # Turn Right
                        left_target = self.wheel_vel_turning
                        right_target = -self.wheel_vel_turning

        return left_target, right_target

    def send_drive_signal(self):
        if not self.ppi is None:
            l_target, r_target = self.get_drive_signal()
            self.wheel_vels = self.ppi.set_velocity(l_target, r_target)

    def latest_drive_signal(self):
        # Return current state of robot
        l,r = self.wheel_vels
        k = self.key_pressed
        return l,r,k
