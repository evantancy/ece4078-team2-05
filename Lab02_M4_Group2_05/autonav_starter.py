# assumption of this basic vision-based auto nav implementation: 
# if the robot can see a marker, then that means there is a path to that marker

# implementation explanation
# the robot first spins 360 to find all visible markers and their estimated pose
# it then creates a list of markers that is reachable from current location and their estimated distance
# the robot now goes to the nearest reachable marker A
# once reached marker A, the robot spins 360 again to find all accessable markers from marker A and their estimated pose
# repeat until all markers are found or timed out

# import required modules
import time
import cv2
import numpy as np
import cv2.aruco as aruco
import os, sys
import json

# don't forget to put penguinPiC.py in the same directory
import PenguinPiC
ppi = PenguinPiC.PenguinPi()
sys.path.insert(0, "{}/slam".format(os.getcwd()))
import slam.Slam as Slam
import slam.Robot as Robot
import slam.Measurements as Measurements

# initialize resulting map containing paths between markers
marker_list = []
saved_map = []
map_f = 'map.txt'
# there are 8 markers in total in the arena
total_marker_num = 8

# drive settings, feel free to change them
wheel_vel = 28
fps = 10

# camera calibration parameters (from M2: SLAM)
camera_matrix = np.loadtxt('camera_calibration/intrinsic.txt', delimiter=',')
dist_coeffs = np.loadtxt('camera_calibration/distCoeffs.txt', delimiter=',')

marker_length = 0.1

# wheel calibration parameters (from M2: SLAM)
wheels_scale = np.loadtxt('calibration/wheel_calibration/scale.txt', delimiter=',')
wheels_width = np.loadtxt('calibration/wheel_calibration/baseline.txt', delimiter=',')

# Slam code functions
class Operate:
    def __init__(self):
        self.pibot = Robot.Robot(wheels_width,wheels_scale*0.5, camera_matrix, dist_coeffs) # manually adjusted baseline value to be more accurate
        self.dt1 = time.time()
    def control(self,lv,rv):
        self.dt2 = time.time()
        drive_meas = Measurements.DriveMeasurement(lv, rv, self.dt2-self.dt1)
        self.dt1 = time.time()
        self.pibot.drive(drive_meas)
        
# display window for visulisation
cv2.namedWindow('video', cv2.WINDOW_NORMAL);
cv2.setWindowProperty('video', cv2.WND_PROP_AUTOSIZE, cv2.WINDOW_AUTOSIZE);

# font display options
font = cv2.FONT_HERSHEY_SIMPLEX
location = (0, 0)
font_scale = 1
font_col = (255, 255, 255)
line_type = 2

# initial location of the robot
robot_pose = [0,0]
current_marker = 'start'

# 15 minutes time-out to prevent being stuck during auto nav
timeout = time.time() + 60*15  
start_t = time.time()

# start flag
start = True
centre = 320#centre of pixels with respect to the camera

# Driving class
class Driving:
    def __init__():
        ppi.set_velocity(0,0)

    def centre (marker_id): #function to centre robot pose towrds target marker
        lv = -wheel_vel
        rv = wheel_vel
        ppi.set_velocity(lv,rv)
        operate.dt1 = time.time()# used to estimate dt to produce accurate measurements of robot pose
        centre_calc=0 #init calculation of aruco marker postion in camera frame

        while centre_calc<centre:
            operate.control(lv,rv)# continue turning until target marker is in the centre of the frame

            # get current frame
            curr = ppi.get_image()

            # visualise ARUCO marker detection annotations
            aruco_params = aruco.DetectorParameters_create()
            aruco_params.minDistanceToBorder = 0
            aruco_params.adaptiveThreshWinSizeMax = 1000
            aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

            corners, ids, rejected = aruco.detectMarkers(curr, aruco_dict, parameters=aruco_params)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

            aruco.drawDetectedMarkers(curr, corners, ids) # for detected markers show their ids
        
            aruco.drawDetectedMarkers(curr, rejected, borderColor=(100, 0, 240)) # unknown squares

            # scale to 144p
            resized = cv2.resize(curr, (960, 720), interpolation = cv2.INTER_AREA)

            # add GUI text
            cv2.putText(resized, 'PenguinPi', (15, 50), font, font_scale, font_col, line_type)

            # visualisation
            cv2.imshow('video', resized)
            cv2.waitKey(1)
            if ids is None:
                continue
            else:
                for i in range(len(ids)):
                    idi = ids[i,0]
                    if idi == marker_id: #calculate position of marker in camera frame
                        centre_calc = corners[i][0][0][0]+(corners[i][0][1][0]-corners[i][0][0][0])/2
        ppi.set_velocity(0,0)

    def recentre (marker_id): #slowly turn in opposite direction after centering in case of significant overshoot, ensuring marker is in centre of camera frame
        lv = 14
        rv = -14
        ppi.set_velocity(lv,rv)
        operate.dt1 = time.time() # used to estimate dt to produce accurate measurements of robot pose
        centre_calc=600 #init calculation of aruco marker postion in camera frame
        while centre_calc>centre:
            operate.control(lv,rv)

            # get current frame
            curr = ppi.get_image()

            # visualise ARUCO marker detection annotations
            aruco_params = aruco.DetectorParameters_create()
            aruco_params.minDistanceToBorder = 0
            aruco_params.adaptiveThreshWinSizeMax = 1000
            aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

            corners, ids, rejected = aruco.detectMarkers(curr, aruco_dict, parameters=aruco_params)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

            aruco.drawDetectedMarkers(curr, corners, ids) # for detected markers show their ids
        
            aruco.drawDetectedMarkers(curr, rejected, borderColor=(100, 0, 240)) # unknown squares

            # scale to 144p
            resized = cv2.resize(curr, (960, 720), interpolation = cv2.INTER_AREA)

            # add GUI text
            cv2.putText(resized, 'PenguinPi', (15, 50), font, font_scale, font_col, line_type)

            # visualisation
            cv2.imshow('video', resized)
            cv2.waitKey(1)
            if ids is None:
                continue
            else:
                for i in range(len(ids)):
                    idi = ids[i,0]
                    if idi == marker_id: 
                        centre_calc = corners[i][0][0][0]+(corners[i][0][1][0]-corners[i][0][0][0])/2
        ppi.set_velocity(0,0)

    def straight (marker_id): # driving straight towards target marker after centering towards it
        centre_calc = centre
        reached_marker = False
        marker_flag = 0
        lv = 60
        rv = 60
        ppi.set_velocity(lv,rv)
        operate.dt1 = time.time()
        dist = 10
        while not reached_marker:
            operate.control(60,60)

            # get current frame
            curr = ppi.get_image()

            # visualise ARUCO marker detection annotations
            aruco_params = aruco.DetectorParameters_create()
            aruco_params.minDistanceToBorder = 0
            aruco_params.adaptiveThreshWinSizeMax = 1000
            aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

            corners, ids, rejected = aruco.detectMarkers(curr, aruco_dict, parameters=aruco_params)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

            aruco.drawDetectedMarkers(curr, corners, ids) # for detected markers show their ids
        
            aruco.drawDetectedMarkers(curr, rejected, borderColor=(100, 0, 240)) # unknown squares

            # scale to 144p
            resized = cv2.resize(curr, (960, 720), interpolation = cv2.INTER_AREA)

            # add GUI text
            cv2.putText(resized, 'PenguinPi', (15, 50), font, font_scale, font_col, line_type)

            # visualisation
            cv2.imshow('video', resized)
            cv2.waitKey(1)

            dist = min(dist,np.sqrt((measurements[ind_id][2]-operate.pibot.state[0]) ** 2 + (measurements[ind_id][3]-operate.pibot.state[1]) ** 2))

            if ids is None:
                marker_flag = marker_flag+1 # check to see if we can no longer detect our traget marker
            else:
                for i in range(len(ids)):
                    idi = ids[i,0]
                    if idi == marker_id: 
                        centre_calc = corners[i][0][0][0]+(corners[i][0][1][0]-corners[i][0][0][0])/2
                marker_flag = 0

            if marker_flag >2 and dist <1:# if we can no longer see our target marker, and our distance away from the marker is less than 1, then we have reached the marker!
                reached_marker = True

            #Proportional controller based on marker position with respect to camera frame
            if centre_calc >centre:
                lv = min(lv+1,65)
                ppi.set_velocity(lv,rv)
            if centre_calc <centre:
                lv = max(lv-1,62)
                ppi.set_velocity(lv,rv)

        ppi.set_velocity(0,0)






#### Initialisations ####
prev_markers = np.zeros(4)
operate = Operate()
ind_id = 0
fin_detection = False
all_seen_ids = np.array([0])

# repeat until all markers are found or until time out
while not fin_detection:

    # save all the seen markers and their estimated poses at each step
    measurements = []
    seen_ids = []
    lv = -wheel_vel
    rv = wheel_vel
    fin_spin = False
    ppi.set_velocity(lv,rv)
    operate.dt1 = time.time()
    spin_time = time.time()

    print("Scanning ...")
    while start and (not fin_spin):
        operate.control(lv,rv)

        # get current frame
        curr = ppi.get_image()

        # visualise ARUCO marker detection annotations
        aruco_params = aruco.DetectorParameters_create()
        aruco_params.minDistanceToBorder = 0
        aruco_params.adaptiveThreshWinSizeMax = 1000
        aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)

        corners, ids, rejected = aruco.detectMarkers(curr, aruco_dict, parameters=aruco_params)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        aruco.drawDetectedMarkers(curr, corners, ids) # for detected markers show their ids
        
        aruco.drawDetectedMarkers(curr, rejected, borderColor=(100, 0, 240)) # unknown squares

        # scale to 144p
        resized = cv2.resize(curr, (960, 720), interpolation = cv2.INTER_AREA)

        # add GUI text
        cv2.putText(resized, 'PenguinPi', (15, 50), font, font_scale, font_col, line_type)

        # visualisation
        cv2.imshow('video', resized)
        cv2.waitKey(1)

        # compute a marker's estimated pose and distance to the robot
        if ids is None:
            continue
        else:
            for i in range(len(ids)):
                idi = ids[i,0]
                # Some markers appear multiple times but should only be handled once.
                if idi in seen_ids: 
                    if idi == seen_ids[0]:
                        if time.time()-dead_time>5:
                            fin_spin = True
                            ppi.set_velocity(0,0)
                        dead_time = time.time()
                    continue
                else:
                    seen_ids.append(idi)
                #Pose estimation
                lm_tvecs = tvecs[ids==idi].T
                lm_bff2d = np.block([[lm_tvecs[2,:]],[-lm_tvecs[0,:]]])
                lm_bff2d = np.mean(lm_bff2d, axis=1).reshape(-1,1)
                th = operate.pibot.state[2]
                robot_xy = operate.pibot.state[0:2,:]
                R_theta = np.block([[np.cos(th), -np.sin(th)],[np.sin(th), np.cos(th)]])
                lm_inertial = robot_xy + R_theta @ lm_bff2d

                # compute Euclidean distance between the robot and the marker
                dist = np.sqrt((lm_inertial[0][0]-robot_pose[0]) ** 2 + (lm_inertial[1][0]-robot_pose[1]) ** 2)

                # save marker measurements and distance
                lm_measurement = [idi, dist, lm_inertial[0][0], lm_inertial[1][0]]

                print("Detected marker",np.around(lm_measurement[0],2),": distance =",np.around(lm_measurement[1],2),", postion =",np.around(lm_measurement[2],2),",",np.around(lm_measurement[3],2))
                measurements.append(lm_measurement)

                if idi == seen_ids[0]:
                    dead_time = time.time()

        if time.time()-spin_time>60:
            fin_spin = True
    

    #EXPAND MAP
    measurements = sorted(measurements, key=lambda x: x[1]) # sort seen markers by distance (closest first)
    ppi.set_velocity(0, 0)
    if len(measurements) > 0:
        # add discovered markers to map
        for accessible_marker in measurements:
            if current_marker != accessible_marker[0]: # avoid adding path to self
                path = []
                path.append(current_marker)
                path.append(accessible_marker[0])
                path.append(accessible_marker[1])
                saved_map.append(path)
                if accessible_marker[0] not in [found[0] for found in marker_list]: # avoid adding repeated marker
                    marker_list.append([accessible_marker[0], accessible_marker[2], accessible_marker[3]])
                else:
                    continue
            else:
                continue

        if len(marker_list) > total_marker_num-1: #check if we have detected all markers, and break from while loop if true
            fin_detection = True
            break

        #Algorithm for deciding which marker to prioritise 

        current_marker = measurements[0][0]
        ind_id = 0
        if len(measurements)==1:
            current_marker = measurements[0][0]
            ind_id = 0

        if len(measurements)==2:
            if current_marker == prev_markers[-2]:
                current_marker = measurements[1][0]
                ind_id = 1
            else:
                for mark in range(len(measurements)):
                    if np.any(prev_markers == measurements[mark][0]):
                        continue
                    else:
                        current_marker = measurements[mark][0]
                        ind_id = mark
                        break
        
        if len(measurements)>2:
            potential_marks = np.array([0])
            for mark in range(len(measurements)):
                if np.any(prev_markers == measurements[mark][0]):
                    continue
                else:
                    potential_marks = np.append(potential_marks,mark)
                    if not np.any(all_seen_ids == measurements[mark][0]):
                        potential_marks = np.array([mark])
                        break
            if len(potential_marks)>1:
                potential_marks = potential_marks[1]
            ind_id = int(potential_marks)
            current_marker = measurements[ind_id][0]
   
              
        print("centering towards marker:", current_marker)
        Driving.centre(marker_id=current_marker)
        print("recentering towards marker:", current_marker)
        Driving.recentre(marker_id=current_marker)
        print("driving towards marker:", current_marker)
        Driving.straight(marker_id=current_marker)

        # update the robot's pose to location of the marker it tries to reach
        robot_pose = [measurements[ind_id][2],measurements[ind_id][3]]#[operate.pibot.state[0],operate.pibot.state[1]]
        prev_markers=np.append(prev_markers,current_marker)
        all_seen_ids = np.append(all_seen_ids,seen_ids)
        print('current map [current marker id, accessible marker id, distance]:\n',saved_map)
        print('current marker list [id, x, y]:\n',marker_list)

    else:
        print('no markers in sight!')
    # ------------------------------------------------------------------------------------

    # time out after 15 minutes
    if time.time() > timeout:
        break

# show time spent generating the map
end_t = time.time()
map_t = (end_t - start_t) / 60
print('time spent generating the map (in minutes): ',map_t)

# save results to map.txt
# sort marker list by id before printing
marker_list = sorted(marker_list, key=lambda x: x[0])
with open(map_f,'w') as f:
    f.write('id, x, y\n')
    for markers in marker_list:
        for marker in markers:
            f.write(str(marker) + ',')
        f.write('\n')
    f.write('\ncurrent id, accessible id, distance\n')
    for routes in saved_map:
        for route in routes:
            f.write(str(route) + ',')
        f.write('\n')
print('map saved!')
