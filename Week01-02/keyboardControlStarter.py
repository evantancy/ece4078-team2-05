from pynput.keyboard import Key, Listener, KeyCode
import cv2
import numpy as np
import penguinPiC

class Keyboard:
    
    
    def __init__(self, ppi=None, fwd=100, rot=40):
        """Constructor
        Args:
            ppi (penguinPiC): penguinPiC object. Defaults to None.
        """
        
        # Triggers for key presses
        self.directions = [False for _ in range(4)]
        # Triggers for incrementing speeds
        self.increments = self.directions
        self.signal_stop = False
        
        # Connection to PenguinPi robot
        self.ppi = ppi
        
        # [Left, Right]
        self.wheel_vels = [0, 0]
        self.wheel_vel_forward = fwd
        self.wheel_vel_rotation = rot
        
        # Initialize pynput keyboard listener
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release).start()
        
        
    def set_state(self, key, activate):
        """What we want the robot to do
        Args:
            key (Key): keyboard input
            activate (bool): turn state on/off
        """
        
        if str(key.char).lower() == 'w':
            self.directions[0] = activate
        elif str(key.char).lower() == 's':
            self.directions[1] = activate
        elif str(key.char).lower() == 'a':
            self.directions[2] = activate
        elif str(key.char).lower() == 'd':
            self.directions[3] = activate
        elif key == Key.space:
            self.signal_stop = activate


    def on_press(self, key):
        """Function call when keys are pressed
        Args:
            key (Key): keyboard input
        """
        try:
            self.set_state(key, True)
        except AttributeError:
            print("Key {0} pressed, use W,A,S,D Keys!".format(
            key.char))
        # Request robot to move
        self.send_drive_signal()
        
    def on_release(self, key):
        """Function call when keys are release
        Args:
            key (Key): keyboard input
        """
        try:
            self.set_state(key, False)
        except AttributeError:
            print("Key {0} pressed, use W,A,S,D Keys!".format(
            key.char))
        
    def get_drive_signal(self):           
        
        # compute drive_forward and drive_rotate using wheel_vel_forward and wheel_vel_rotation
        #drive_forward = 100*(np.multiply(self.directions[0],1)-np.multiply(self.directions[1],1))
        #drive_rotate = 20*(np.multiply(self.directions[2],1)-np.multiply(self.directions[3],1))

        # translate drive_forward and drive_rotate into left_speed and right_speed
        #left_speed = drive_forward -(drive_rotate<0)*drive_rotate
        #right_speed = drive_forward +(drive_rotate>0)*drive_rotate
        
        if self.directions[0]:
            left_speed = self.wheel_vel_forward
            right_speed = self.wheel_vel_forward
            self.directions[0] = False

        elif self.directions[1]:
            left_speed = -self.wheel_vel_forward
            right_speed = -self.wheel_vel_forward
            self.directions[1] = False

        elif self.directions[2]:
            left_speed = 0
            right_speed = self.wheel_vel_rotation
            self.directions[2] = False

        elif self.directions[3]:
            left_speed = self.wheel_vel_rotation
            right_speed = 0
            self.directions[3] = False

        elif self.directions[4]:
            self.wheel_vel_forward+=50
            self.directions[4] = False
            if self.wheel_vels[0]==0 and self.wheel_vels[1]==0:
                left_speed = 0
                right_speed = 0
            elif self.wheel_vels[0]>0 and self.wheel_vels[1]>0:
                left_speed = self.wheel_vel_forward
                right_speed = self.wheel_vel_forward
            elif self.wheel_vels[0]<0 and self.wheel_vels[1]<0:
                left_speed = -self.wheel_vel_forward
                right_speed = -self.wheel_vel_forward
            elif self.wheel_vels[0]>self.wheel_vels[1]:
                left_speed = self.wheel_vel_rotation
                right_speed = 0
            else:
                left_speed = 0
                right_speed = self.wheel_vel_rotation

            

        elif self.signal_stop:
            left_speed = 0
            right_speed = 0
            self.wheel_vel_forward = 100
            self.wheel_vel_rotation = 20
            self.signal_stop = False

        return left_speed, right_speed
    
    def send_drive_signal(self):
        if not self.ppi is None:
            lv, rv = self.get_drive_signal()
            lv, rv = self.ppi.set_velocity(lv, rv)
            self.wheel_vels = [lv, rv]
            
    def latest_drive_signal(self):
        # Return current state of robot
        return self.wheel_vels
    

if __name__ == "__main__":
    print("Use the W,A,S,D Keys to drive to robot")
    print("Use the Up/Down arrow keys to increase/decrease speed moving forwards/backwards")
    print("Use the Left/Right arrow keys to increase/decrease speed turning left/right")
    
    ppi = penguinPiC.PenguinPi()
    keyboard_control = Keyboard(ppi)
    cv2.namedWindow('video', cv2.WINDOW_NORMAL);
    cv2.setWindowProperty('video', cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE);

    while True:
        # font display options
        font = cv2.FONT_HERSHEY_SIMPLEX
        location = (0, 0)
        font_scale = 1
        font_col = (255, 0, 0)
        line_type = 2

        # get velocity of each wheel
        wheel_vels = keyboard_control.latest_drive_signal();
        L_Wvel = wheel_vels[0]
        R_Wvel = wheel_vels[1]

        # get current camera frame
        curr = ppi.get_image()

        # scale to 144p
        # feel free to change the resolution
        resized = cv2.resize(curr, (960, 720), interpolation = cv2.INTER_AREA)

        # feel free to add more GUI texts
        cv2.putText(resized, 'PenguinPi', (15, 50), font, font_scale, font_col, line_type)
        cv2.putText(resized, 'L Vel = '+str(L_Wvel)+', R Vel = '+str(R_Wvel), (15,100), font, font_scale, font_col, line_type)

        cv2.imshow('video', resized)
        cv2.waitKey(1)

        continue
