import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState


def set_model_abs_y(model_name, y):
    rospy.wait_for_service("/gazebo/set_model_state")
    func = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    msg = ModelState()
    msg.model_name = model_name
    msg.pose.position.y = y

    func(msg)


def get_model_state(model_name):
    rospy.wait_for_service("/gazebo/get_model_state")
    func = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    return func(model_name, None)
