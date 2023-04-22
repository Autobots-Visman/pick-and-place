#!/usr/bin/env python3
"""
A utility to select keypoints from a color image.
"""
from argparse import ArgumentParser
from functools import partial

import cv2
import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge
from gknet_msgs.msg import KeypointList, ObjectFilterList
from sensor_msgs.msg import Image


def annotate_keypoints(img, keypoint_list):
    """Annotate the image with the detected keypoints."""
    img = img.copy()
    for kp in keypoint_list.keypoints:
        kp_lm = kp.left_middle
        kp_rm = kp.right_middle
        center = kp.center
        # draw arrow for left-middle and right-middle key-points
        lm_ep = (
            int(kp_lm[0] + (kp_rm[0] - kp_lm[0]) / 5.0),
            int(kp_lm[1] + (kp_rm[1] - kp_lm[1]) / 5.0),
        )
        rm_ep = (
            int(kp_rm[0] + (kp_lm[0] - kp_rm[0]) / 5.0),
            int(kp_rm[1] + (kp_lm[1] - kp_rm[1]) / 5.0),
        )
        img = cv2.arrowedLine(img, kp_lm, lm_ep, (0, 0, 0), 2)
        img = cv2.arrowedLine(img, kp_rm, rm_ep, (0, 0, 0), 2)
        # draw left-middle, right-middle and center key-points
        img = cv2.circle(img, (int(kp_lm[0]), int(kp_lm[1])), 2, (0, 0, 255), 2)
        img = cv2.circle(img, (int(kp_rm[0]), int(kp_rm[1])), 2, (0, 0, 255), 2)
        img = cv2.circle(img, (int(center[0]), int(center[1])), 2, (0, 0, 255), 2)
    return img


def annotate_object_filter(img, object_filter_list):
    img = img.copy()
    for obj in object_filter_list.objects:
        bbox = obj.bbox
        img = cv2.rectangle(
            img,
            (int(bbox[0]), int(bbox[1])),
            (int(bbox[2]), int(bbox[3])),
            (255, 0, 0),
            3,
        )
    return img


class UIState:
    def __init__(self):
        self.callbacks = []
        self.img = None
        self.keypoint_list = None
        self.object_filter_list = None

    def subscriber_callback(self, cv_bridge, rgb_msg, keypoint_msg, object_filter_msg):
        self.img = np.array(
            cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8"), dtype=np.uint8
        )
        self.keypoint_list = keypoint_msg
        self.object_filter_list = object_filter_msg

    def interact_callback(self, event, x, y, flags, param):
        rospy.loginfo("clicked!")

    def draw_loop(self, title="image"):
        # https://stackoverflow.com/questions/56623487/why-does-a-right-click-open-a-drop-down-menu-in-my-opencv-imshow-window
        cv2.namedWindow(
            title,
            flags=cv2.WINDOW_AUTOSIZE | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL,
        )
        cv2.setMouseCallback(title, partial(self.interact_callback))

        while not rospy.is_shutdown():
            img = annotate_keypoints(self.img, self.keypoint_list)
            img = annotate_object_filter(img, self.object_filter_list)
            cv2.imshow(title, img)
            if (
                cv2.waitKey(20) & 0xFF in [27, ord("q")]
                or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1
            ):
                break
        cv2.destroyAllWindows()


def parse_args():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--color-image-topic",
        type=str,
        default="/camera/color/image_raw",
    )
    parser.add_argument("--keypoints-topic", type=str, default="/gknet/keypoints")
    parser.add_argument(
        "--object-filter-topic", type=str, default="/gknet/object_filter"
    )
    parser.add_argument(
        "--annotated-image-topic", type=str, default="/gknet/annotated_image"
    )
    parser.add_argument("--subscriber-queue-size", type=int, default=100)
    parser.add_argument("--slop", type=float, default=0.1)
    # ignore any other args
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()
    rospy.loginfo("starting annotate with args: ", args)

    rospy.init_node("annotate", anonymous=True, log_level=rospy.INFO)
    cv_bridge = CvBridge()

    # let's wait for the first message to arrive
    rospy.loginfo(f"waiting for message on {args.color_image_topic}")
    rospy.wait_for_message(args.color_image_topic, Image)
    rospy.loginfo(f"waiting for message on {args.keypoints_topic}")
    rospy.wait_for_message(args.keypoints_topic, KeypointList)
    rospy.loginfo(f"waiting for message on {args.object_filter_topic}")
    rospy.wait_for_message(args.object_filter_topic, ObjectFilterList)
    rospy.loginfo("input topics ready for processing")

    image_sub = message_filters.Subscriber(args.color_image_topic, Image)
    keypoint_sub = message_filters.Subscriber(args.keypoints_topic, KeypointList)
    object_filter_sub = message_filters.Subscriber(
        args.object_filter_topic, ObjectFilterList
    )
    ts = message_filters.ApproximateTimeSynchronizer(
        [image_sub, keypoint_sub, object_filter_sub],
        args.subscriber_queue_size,
        args.slop,
    )

    ui_state = UIState()
    ts.registerCallback(partial(ui_state.subscriber_callback, cv_bridge))

    # wait for the first message to arrive
    rospy.loginfo(f"waiting for message on {args.color_image_topic}")
    while ui_state.img is None:
        rospy.sleep(0.1)
    rospy.loginfo(f"got our first image with shape {ui_state.img.shape}")

    ui_state.draw_loop(title=f"select ui: {args.color_image_topic}")


if __name__ == "__main__":
    main()
