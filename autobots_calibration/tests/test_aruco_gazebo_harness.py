import time

import pytest
import roslaunch
import rospy

from autobots_calibration.msg import CameraStatus

SLEEP_TIME = 1


@pytest.fixture()
def launch_process():
    node = roslaunch.core.Node("autobots_calibration", "aruco_gazebo_harness.py")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    # http://docs.ros.org/en/melodic/api/roslaunch/html/
    process = launch.launch(node)
    yield process
    if process.is_alive():
        process.stop()


def test_msg_camera_status():
    msg = CameraStatus()
    assert msg.status == CameraStatus.WAITING
    msg = CameraStatus(status=CameraStatus.STABLE)
    assert msg.status == CameraStatus.STABLE


def test_node_launches_without_issues(launch_process):
    """A simple test to show-case how to check that a node is running."""
    # check that the node is running
    time.sleep(SLEEP_TIME)
    assert launch_process.is_alive()
    launch_process.stop()
    assert not launch_process.is_alive()


def test_node_stops_after_seeing_stable_status(launch_process):
    """A simple test to show-case how to check that a node is running."""

    pub = rospy.Publisher("/calibration/camera/status", CameraStatus, queue_size=1)
    assert launch_process.is_alive()

    msg = CameraStatus(status=CameraStatus.CALIBRATING)
    pub.publish(msg)
    time.sleep(SLEEP_TIME)
    assert launch_process.is_alive()
    print(dir(launch_process))

    msg.status = CameraStatus.STABLE
    pub.publish(msg)
    time.sleep(SLEEP_TIME)
    assert not launch_process.is_alive()
