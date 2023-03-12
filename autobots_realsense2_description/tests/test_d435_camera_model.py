import time

import pytest
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from .utils import get_model_state, set_model_abs_y


@pytest.fixture(autouse=True)
def node():
    rospy.init_node("test_d435_camera_model", anonymous=True)


@pytest.fixture()
def bridge():
    return CvBridge()


@pytest.fixture(autouse=True)
def reset_state():
    origin = get_model_state("camera")
    yield
    set_model_abs_y("camera", origin.pose.position.y)


@pytest.mark.parametrize(
    "topic",
    [
        "/camera/aligned_depth_to_color/camera_info",
        "/camera/aligned_depth_to_color/image_raw",
    ],
)
def test_topic_list(topic):
    """A simple test to show-case how to check that a topic is published."""
    # check that the topic is published
    topics = rospy.get_published_topics()
    assert topic in [t[0] for t in topics], "topic not published"


@pytest.mark.parametrize(
    "topic,shape",
    [
        ["/camera/color/image_raw", (480, 640, 3)],
        ["/camera/depth/image_raw", (720, 1280)],
        ["/camera/aligned_depth_to_color/image_raw", (480, 640)],
    ],
)
def test_receive_image(bridge, topic, shape):
    """A simple test to show-case how to mock a message and check that it is received."""

    # check that the image is received
    msg = rospy.wait_for_message(topic, Image)
    print(msg.encoding)
    assert msg, "no message received"
    img = bridge.imgmsg_to_cv2(msg, msg.encoding)
    assert img.shape == shape


def test_position_is_static():
    set_model_abs_y("camera", 0.5)
    original_state = get_model_state("camera")
    time.sleep(0.25)
    new_state = get_model_state("camera")
    assert original_state.pose.position.y == new_state.pose.position.y
