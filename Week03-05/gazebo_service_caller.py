import rospy
import std_msgs
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from pynput.keyboard import Key, Listener, KeyCode

""" 
Automatic calibration of penguinPi Robot
"""

class GazeboServiceCaller:
    def __init__(self):
        rospy.init_node('gazebo_service_caller')
    
    def reset_world_service(self):
        rws = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
    def get_model_state(self, model, entity=None):
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        result = gms(model, entity)
        return result
            
if __name__ == "__main__":
    # Gazebo info
    gz_info = GazeboServiceCaller()
    model_name = "PenguinPi"
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        a = gz_info.get_model_state(model_name)
        print("x: {:.5f}".format(a.pose.position.x))
        print("y: {:.5f}".format(a.pose.position.y))
        print("z: {:.5f}\n".format(a.pose.position.z))
        rate.sleep()

