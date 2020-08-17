import rospy
import std_msgs
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState
from pynput.keyboard import Key, Listener, KeyCode

""" @TODO
Automatic calibration of penguinPi Robot
"""

class GazeboServiceCaller:
    def __init__(self):
        rospy.init_node('gazebo_service_caller')
        self.position = [0 for _ in range(3)]
        self.orientation = [0 for _ in range(3)]
    
    def reset_world_service(self):
        """Reset Gazebo world
        """
        rws = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        
    def get_model_state(self, model, entity=None):
        """Call get_model_state service to get latest state from Gazebo
        Args:
            model (string): Name of model
            entity (string, optional): Name of entity i.e. link/joint. Defaults to None.
        """
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        result = gms(model, entity)
        self.position = result.pose.position
        self.orientation = result.pose.orientation
    
    def diagnostics(self, attribute):
        """Print properties of gazebo simulation
        Args:
            attribute (string): Desired property of GazeboServiceCaller to monitor 
        """
        x = getattr(self, attribute)
        print("{:.5f}".format(x))
            
if __name__ == "__main__":
    # Gazebo info
    gsc = GazeboServiceCaller()
    model_name = "PenguinPi"
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        gsc.get_model_state(model_name)
        gsc.diagnostics('position')

        rate.sleep()
