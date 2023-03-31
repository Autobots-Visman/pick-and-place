#!/usr/bin/env python3
"""Find calibration parameters for the camera."""
from argparse import ArgumentParser

import cv2
import numpy as np
import rospy
from camera.extrinsic.aruco import CtoW_Calibrator_aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from autobots_calibration.msg import CameraExtrinsics, CameraStatus


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
    parser.add_argument(
        "--max-frames",
        type=int,
        default=100,
        help="number of frames to use for calibration",
    )
    parser.add_argument(
        "--marker-length",
        type=float,
        default=0.1,
        help="length of the marker in meters",
    )
    # ignore any other args
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rospy.init_node("calibrate_camera", anonymous=True)
    cv_bridge = CvBridge()

    status_pub = rospy.Publisher(
        args.status_topic, CameraStatus, queue_size=1, latch=True
    )
    extrinsics_pub = rospy.Publisher(
        args.output_topic, CameraExtrinsics, queue_size=1, latch=True
    )

    camera_info = rospy.wait_for_message(args.camera_info_topic, CameraInfo)
    intrinsic_mat = np.array(camera_info.K).reshape(3, 3)
    calibrator = CtoW_Calibrator_aruco(
        intrinsic_mat, markerLength_CL=args.marker_length, maxFrames=args.max_frames
    )

    # wait for the camera to come online
    status_pub.publish(CameraStatus(status=CameraStatus.WAITING))
    msg = rospy.wait_for_message(args.image_topic, Image)
    status_pub.publish(CameraStatus(status=CameraStatus.CALIBRATING))

    # feed messages into the calibrator until it is stable
    while not calibrator.stable_status:
        msg = rospy.wait_for_message(args.image_topic, Image)
        img = cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        calibrator.process(img)

    # publish the calibration as a flattened 4x4 matrix in a latched message
    msg = CameraExtrinsics(M_CL=calibrator.M_CL.flatten().tolist())
    extrinsics_pub.publish(msg)

    # publish the status as a latched message
    msg = CameraStatus(status=CameraStatus.STABLE)
    status_pub.publish(msg)

    # done!
    print("calibration complete")


if __name__ == "__main__":
    main()
