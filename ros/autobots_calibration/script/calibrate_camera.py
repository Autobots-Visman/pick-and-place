#!/usr/bin/env python3
"""Find calibration parameters for the camera."""
from argparse import ArgumentParser

import cv2
import numpy as np
import rospy
import tf2_ros
from autobots_calibration.msg import CameraExtrinsics, CameraStatus
from camera.extrinsic.aruco import CtoW_Calibrator_aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf.transformations import quaternion_from_matrix


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
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="rate at which to publish the transform",
    )
    parser.add_argument(
        "--camera-frame-id",
        type=str,
        default="camera/base_link",
        help="frame id of the camera",
    )
    parser.add_argument(
        "--marker-frame-id",
        type=str,
        default="aruco_tile/base_link",
        help="frame id of the marker",
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
        intrinsic_mat,
        markerLength_CL=args.marker_length,
        maxFrames=args.max_frames,
        flag_vis_extrinsic=False,
        flag_print_MCL=False,
        aruco_dict=cv2.aruco.DICT_5X5_250,
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
    print(f"M_CL:{calibrator.M_CL}")

    # let's also publish the calibration as a static transform using tf2
    rospy.loginfo(
        f"publishing tf transform from {args.marker_frame_id} to {args.camera_frame_id}"
    )
    M_CL = np.array(calibrator.M_CL)
    q = quaternion_from_matrix(M_CL)
    camera_pose = TransformStamped(
        header=Header(frame_id=args.marker_frame_id),
        child_frame_id=args.camera_frame_id,
        transform=Transform(
            translation=Vector3(x=M_CL[0, 3], y=M_CL[1, 3], z=M_CL[2, 3]),
            rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        ),
    )

    # rospy.loginfo(
    #     f"publishing tf transform from {args.camera_frame_id} to {args.marker_frame_id}"
    # )
    # # lets also publish the inverse transform
    # M_CL_inv = np.linalg.inv(M_CL)
    # q_inv = quaternion_from_matrix(M_CL_inv)
    # marker_pose = TransformStamped(
    #     header=Header(frame_id=args.camera_frame_id),
    #     child_frame_id=args.marker_frame_id,
    #     transform=Transform(
    #         translation=Vector3(x=M_CL_inv[0, 3], y=M_CL_inv[1, 3], z=M_CL_inv[2, 3]),
    #         rotation=Quaternion(x=q_inv[0], y=q_inv[1], z=q_inv[2], w=q_inv[3]),
    #     ),
    # )

    rate = rospy.Rate(args.rate)
    broadcaster = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        camera_pose.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(camera_pose)
        # marker_pose.header.stamp = rospy.Time.now()
        # broadcaster.sendTransform(marker_pose)
        rate.sleep()


if __name__ == "__main__":
    main()
