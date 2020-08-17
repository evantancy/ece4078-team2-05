# test access to wheels and camera
# for physical robot use port 8080

import time
import numpy as np
import requests
import cv2 

def set_velocity(vel0, vel1):
    r = requests.get("http://localhost:40000/robot/set/velocity?value="+str(vel0)+","+str(vel1))

def get_image():
    r = requests.get("http://localhost:40000/camera/get")
    img = cv2.imdecode(np.fromstring(r.content,np.uint8), cv2.IMREAD_COLOR)
    return img

if __name__ == "__main__":
    print("test left motor")
    set_velocity(10,0)
    time.sleep(3)
    print("test right motor")
    set_velocity(0,10)
    time.sleep(3)
    print("stop")
    set_velocity(0,0)
    print("capture image")
    image = get_image()
    cv2.namedWindow('camera view', cv2.WINDOW_NORMAL)
    cv2.imshow('camera view', image)
    cv2.waitKey(3000)
    print("image size %d by %d" % (image.shape[0],image.shape[1]))


