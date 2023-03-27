#!/usr/bin/env python3
"""Stream images from a topic using cv2.

https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html
"""
from argparse import ArgumentParser

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def parse_args():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--image-topic", type=str, default="/camera/color/image_raw")
    # ignore any other args
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rospy.init_node("stream_camera", anonymous=True)
    bridge = CvBridge()

    cv2.namedWindow("image", cv2.WINDOW_AUTOSIZE)
    while True:
        msg = rospy.wait_for_message(args.image_topic, Image)
        img = bridge.imgmsg_to_cv2(msg, msg.encoding)
        cv2.imshow("image", img)
        if cv2.waitKey(1) != -1:
            break


if __name__ == "__main__":
    main()
