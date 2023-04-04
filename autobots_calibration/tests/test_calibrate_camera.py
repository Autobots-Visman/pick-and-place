import time

import pytest
import roslaunch
import rospy

from autobots_calibration.msg import CameraStatus

SLEEP_TIME = 1
STATUS_TOPIC = "/calibration/status_topic/status"


@pytest.fixture()
def launch_process():
    node = roslaunch.core.Node("autobots_calibration", "calibrate_camera.py")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    # http://docs.ros.org/en/melodic/api/roslaunch/html/
    process = launch.launch(node)
    yield process
    if process.is_alive():
        process.stop()


def test_calibrate_camera(launch_process):
    time.sleep(SLEEP_TIME)
    assert launch_process.is_alive()

    # check the status topic
    while True:
        msg = rospy.wait_for_message(STATUS_TOPIC, CameraStatus)
        if msg.status == CameraStatus.STABLE:
            print("calibration stable")
            break
