from pynput.keyboard import Key, Listener, KeyCode
import cv2
import numpy as np
import penguinPiC

class Keyboard:
    
    
    
    def __init__(self, ppi=None, forward_vel=100, turning_vel=40):
        """Constructor
        Args:
            ppi (penguinPiC): penguinPiC Object. Defaults to None.
            forward_vel (int): Forward/backwards velocity Defaults to 100.
            turning_vel (int): Turning velocity Defaults to 40.
        """
        
        # Triggers for key presses
        self.directions = [False for _ in range(4)]
        self.signal_stop = False
        
        # Connection to PenguinPi robot
        self.ppi = ppi
        
        # [Left, Right]
        self.wheel_vels = [0, 0]
        self.wheel_vel_forward = forward_vel
        self.wheel_vel_turning = turning_vel
        
        # Initialize pynput keyboard listener (non-blocking)
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
        print(key)

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
        
        "Update speed - Check the current state of the robot movement so we can update the wheel velocity from the speed adjustments accordingly:"
        [left_target,right_target] = self.latest_drive_signal()
        if left_target>right_target:
            left_target = self.wheel_vel_turning
            right_target = 0
        elif left_target<right_target:
            left_target = 0
            right_target = self.wheel_vel_turning
        elif left_target>0 and right_target>0:
            left_target = self.wheel_vel_forward
            right_target = self.wheel_vel_forward
        elif left_target<0 and right_target<0:
            left_target = -self.wheel_vel_forward
            right_target = -self.wheel_vel_forward
        
        "stop on space bar"
        if self.signal_stop == True:
            left_target = 0
            right_target = 0
            self.wheel_vel_forward = 100
            self.wheel_vel_turning = 40
            self.signal_stop = False

        "Update direction - check within directions array and adjust left and right wheel velocity accordingly"
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
                    left_target = 0
                    right_target = self.wheel_vel_turning
                    
                elif index == 3:
                    # Turn Right
                    left_target = self.wheel_vel_turning
                    right_target = 0
                    
        return left_target, right_target
    
    
    def send_drive_signal(self):
        if not self.ppi is None:
            l_target, r_target = self.get_drive_signal()
            self.wheel_vels = self.ppi.set_velocity(l_target, r_target)
            
    def latest_drive_signal(self):
        # Return current state of robot
        return self.wheel_vels
    
    def diagnostics(self):
        lst = [self.wheel_vel_forward, self.wheel_vel_turning]
    

if __name__ == "__main__":
    print("Use the W,A,S,D Keys to drive to robot")
    print("Use the Up/Down arrow keys to increase/decrease speed moving forwards/backwards")
    print("Use the Left/Right arrow keys to increase/decrease speed turning left/right")
    
    ppi = penguinPiC.PenguinPi()
    keyboard_control = Keyboard(ppi)
    
    cv2.namedWindow('PenguinPi Stream', cv2.WINDOW_AUTOSIZE);

    while True:
        # Font display options
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.8
        font_col = (255, 127, 127)
        line_type = 1

        left_wheel_vel, right_wheel_vel = keyboard_control.latest_drive_signal();

        # Get current camera frame
        frame = ppi.get_image()
        
        # Scale to 144p
        scale = 1.25
        width = int(scale * frame.shape[1])
        height = int(scale * frame.shape[0])
        resized_frame = cv2.resize(frame, (width, height), interpolation = cv2.INTER_AREA)
        
        # OpenCV display
        x = 15
        y = 30
        cv2.putText(resized_frame, 'PenguinPi', (x, y), font, font_scale, font_col, line_type)
        cv2.putText(resized_frame, 'L_VEL = '+str(left_wheel_vel)+', R_VEL = '+str(right_wheel_vel), (15, 2*y), font, font_scale, font_col, line_type)
        cv2.imshow('PenguinPi Stream', resized_frame)
        cv2.waitKey(1)

        continue
