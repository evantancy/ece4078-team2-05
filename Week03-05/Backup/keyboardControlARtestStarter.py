# show ARUCO marker detection annotations when teleoperating the robot through keyboard

from pynput.keyboard import Key, Listener, KeyCode
import cv2
import numpy as np

# import OpenCV ARUCO functions
import cv2.aruco as aruco

# replace with your own keyboard teleoperation codes
class Keyboard:
    def __init__(self, ppi=None, forward_vel=60, turning_vel=40):
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
            if str(key.char).lower() == 'w':
                self.directions[0] = activate
                
            elif str(key.char).lower() == 's':
                self.directions[1] = activate
                
            elif str(key.char).lower() == 'a':
                self.directions[2] = activate
                
            elif str(key.char).lower() == 'd':
                self.directions[3] = activate
                
        except AttributeError:
            pass
    
    
    def set_target(self, key):
        """Increase/decrease target velocities
        Args:
            key (Key): keyboard input
        """

        if key == Key.up:
            if self.wheel_vel_forward < 300:
                self.wheel_vel_forward += 5
            
        elif key == Key.down:
            if self.wheel_vel_forward > 0:
                self.wheel_vel_forward -= 5
            
        elif key == Key.left:
            if self.wheel_vel_turning > 0:
                self.wheel_vel_turning -= 2
            
        elif key == Key.right:
            if self.wheel_vel_turning < 100:
                self.wheel_vel_turning += 2

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
            self.wheel_vel_turning = 40
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
    

if __name__ == "__main__":
    import PenguinPiC
    ppi = PenguinPiC.PenguinPi()

    keyboard_control = Keyboard(ppi)

    cv2.namedWindow('video', cv2.WINDOW_NORMAL);
    cv2.setWindowProperty('video', cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE);

    while True:
        # font display options
        font = cv2.FONT_HERSHEY_SIMPLEX
        location = (0, 0)
        font_scale = 1
        font_col = (255, 255, 255)
        line_type = 2

        # Get velocity of each wheel
        left_wheel_vel, right_wheel_vel, key_pressed = keyboard_control.latest_drive_signal();

        # Get current frame
        curr = ppi.get_image()
        
        # uncomment to see how noises influence the accuracy of ARUCO marker detection
        #im = np.zeros(np.shape(curr), np.uint8)
        #cv2.randn(im,(0),(99))
        #curr = curr + im
        
        # show ARUCO marker detection annotations
        aruco_params = aruco.DetectorParameters_create()
        aruco_params.minDistanceToBorder = 0
        aruco_params.adaptiveThreshWinSizeMax = 1000
        aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    
        corners, ids, rejected = aruco.detectMarkers(curr, aruco_dict, parameters=aruco_params)
    
        aruco.drawDetectedMarkers(curr, corners, ids) # for detected markers show their ids
        aruco.drawDetectedMarkers(curr, rejected, borderColor=(100, 0, 240)) # unknown squares
        
        # Scale to 144p
        resized = cv2.resize(curr, (960, 720), interpolation = cv2.INTER_AREA)
        x = 15
        y = 30
        # Replace with your own GUI
        cv2.putText(resized, 'PenguinPi', (x, y), font, font_scale, font_col, line_type)
        cv2.putText(resized, 'L VEL = '+str(left_wheel_vel)+', R VEL = '+str(right_wheel_vel), (15, 2*y), font, font_scale, (0,0,0), 2)
        cv2.putText(resized, 'KEY PRESSED: '+str(key_pressed), (15, 3*y), font, font_scale, (255,0,0), 3)

        cv2.imshow('video', resized)

        cv2.waitKey(1)
        #if keyboard_control.isded:
        #    break

        continue
