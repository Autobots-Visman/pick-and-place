#!/usr/bin/env python3
"""A harness that places a flat marker on a flat surface for calibration."""
from argparse import ArgumentParser

import rospy

from autobots_calibration.msg import CameraStatus


def parse_args():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera-info-topic", type=str, default="/camera/color/camera_info"
    )
    parser.add_argument("--image-topic", type=str, default="/camera/color/image_raw")
    parser.add_argument(
        "--output-topic", type=str, default="/calibration/camera/matrix_camera_link"
    )
    parser.add_argument(
        "--status-topic", type=str, default="/calibration/camera/status"
    )
    # ignore any other args
    args, _ = parser.parse_known_args()
    return args


def throw_when_status_stable(msg: CameraStatus):
    if msg.status == CameraStatus.STABLE:
        raise rospy.ROSInterruptException("Camera calibration parameters are stable")


def main():
    args = parse_args()
    rospy.init_node("aruco_harness", anonymous=True)

    print("TODO: spawn a flat marker in gazebo")

    while True:
        msg = rospy.wait_for_message(args.status_topic, CameraStatus)
        if msg.status == CameraStatus.STABLE:
            print("calibration stable")
            break

    print("TODO: remove the flat marker from gazebo")


if __name__ == "__main__":
    main()
