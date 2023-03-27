from pathlib import Path

import pytest
import rospy
from cv_bridge import CvBridge


@pytest.fixture(autouse=True)
def init_node():
    rospy.init_node("test_autobots_calibration", anonymous=True)


@pytest.fixture()
def launch_path():
    return Path(__file__).parent.parent / "launch"


@pytest.fixture()
def cv_bridge():
    return CvBridge()
