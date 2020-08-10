# teleoperate the robot through keyboard control
# getting-started code

from pynput.keyboard import Key, Listener, KeyCode
import cv2
import numpy as np

class Keyboard:
    # feel free to change the speed, or add keys to do so
    wheel_vel_forward = 100
    wheel_vel_rotation = 20

    def __init__(self, ppi=None):
        # storage for key presses
        self.directions = [False for _ in range(5)]
        self.signal_stop = False 

        # connection to PenguinPi robot
        self.ppi = ppi
        self.wheel_vels = [0, 0]

        self.listener = Listener(on_press=self.on_press).start()

    def on_press(self, key):
        print(key)
        # use arrow keys to drive, space key to stop
        # feel free to add more keys
        if key == Key.up:
            self.directions[0] = True
        elif key == Key.down:
            self.directions[1] = True
        elif key == Key.left:
            self.directions[2] = True
        elif key == Key.right:
            self.directions[3] = True
        elif key == key.shift:
            self.directions[4] = True

        #elif key == Key.S:
        #    self.directions[5] = True
        elif key == Key.space:
            self.signal_stop = True

        self.send_drive_signal()
        
    def get_drive_signal(self):           
        # translate the key presses into drive signals 
        
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
        return self.wheel_vels
    

if __name__ == "__main__":
    import penguinPiC
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
