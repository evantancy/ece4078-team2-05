import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState


class GazeboServiceCaller:
    def __init__(self, model_name="PenguinPi"):
        """
        Args:
            model_name (str, optional): Name of model you want to call services for. Defaults to "PenguinPi".
        """
        self.model_name = model_name

    def reset_world(self):
        """Reset Gazebo world
        WARNING: This resets the map and models too
        """
        rospy.ServiceProxy("/gazebo/reset_world", Empty)

    def print_model_state(self, entity=None):
        """Call get_model_state service to get latest state from Gazebo
        Args:
            model (string): Name of model
            entity (string, optional): Name of entity i.e. link/joint.
                                        Defaults to None.
        """
        gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        result = gms(self.model_name, entity)
        position = result.pose.position
        print(f"{position}")

    def reset_model(self, entity=None):
        """Reset model to original pose
        Args:
            entity (string, optional): Name of entity i.e. link/joint.
            Defaults to None.
        """
        state_msg = ModelState()
        state_msg.model_name = self.model_name
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0
        try:
            set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            set_state(state_msg)
        except rospy.ServiceException:
            print(f"Service call failed")

if __name__ == "__main__":
    a = GazeboServiceCaller()
    a.reset_model("PenguinPi")
