#!/usr/bin/env python3
import cv2
import geometry_msgs.msg
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from sensor_msgs.msg import CameraInfo, Image
from tf.transformations import inverse_matrix, quaternion_matrix


def wait_for_transform(tf_buffer, source_frame, target_frame):
    rospy.loginfo(f"waiting for transform from {source_frame} to {target_frame}")
    while not rospy.is_shutdown():
        try:
            return tf_buffer.lookup_transform(
                source_frame, target_frame, rospy.Time(0), rospy.Duration(1.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            continue


def do_transform_point_2d(p_camera, fx, fy, cx, cy):
    # Convert the 3D point in the camera frame to a 2D point
    u = fx * p_camera.point.x / p_camera.point.z + cx
    v = fy * p_camera.point.y / p_camera.point.z + cy
    return u, v


def get_camera_info(topic):
    # Wait for camera info message to become available
    cam_info = rospy.wait_for_message(topic, CameraInfo)
    # Extract camera intrinsic parameters from camera info message
    # Intrinsic camera matrix for the raw (distorted) images.
    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]
    # Projects 3D points in the camera coordinate frame to 2D pixel
    # coordinates using the focal lengths (fx, fy) and principal point
    # (cx, cy).
    # http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html
    fx = cam_info.K[0]
    fy = cam_info.K[4]
    cx = cam_info.K[2]
    cy = cam_info.K[5]
    return fx, fy, cx, cy


def do_inverse_transform_point(point, transform):
    t = transform.transform
    translation = np.array([t.translation.x, t.translation.y, t.translation.z])
    rotation = np.array([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
    rotation_matrix = quaternion_matrix(rotation)[:3, :3]
    transform_matrix = np.identity(4)
    transform_matrix[:3, :3] = rotation_matrix
    transform_matrix[:3, 3] = translation

    inverse_transform_matrix = inverse_matrix(transform_matrix)

    point_np = np.array([point.point.x, point.point.y, point.point.z, 1])
    transformed_point_np = np.dot(inverse_transform_matrix, point_np)

    transformed_point = geometry_msgs.msg.PointStamped(
        header=transform.header,
        point=geometry_msgs.msg.Point(
            x=transformed_point_np[0],
            y=transformed_point_np[1],
            z=transformed_point_np[2],
        ),
    )
    return transformed_point


def main():
    rospy.init_node("gazebo_model_list")

    rospy.wait_for_service("/gazebo/get_world_properties")
    rospy.wait_for_service("/gazebo/get_model_state")

    get_world_properties = rospy.ServiceProxy(
        "/gazebo/get_world_properties", GetWorldProperties
    )
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    # Wait for camera info message to become available
    fx, fy, cx, cy = get_camera_info("/camera/color/camera_info")

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    # Wait for the world to camera transform to become available
    world_to_camera = wait_for_transform(tf_buffer, "world", "camera/base_link")
    world_properties = get_world_properties()
    points = []
    for model_name in world_properties.model_names:
        model_state = get_model_state(model_name, "")
        pose = model_state.pose
        rospy.loginfo(f"Model: {model_name}, Pose:\n{pose}")

        # Define a 3D point in the world frame
        p_world = geometry_msgs.msg.PointStamped(
            header=rospy.Header(stamp=rospy.Time.now(), frame_id="world"),
            point=pose.position,
        )
        p_camera = do_inverse_transform_point(p_world, world_to_camera)
        # Transform the point and print the result
        u, v = do_transform_point_2d(p_camera, fx, fy, cx, cy)
        rospy.loginfo("Point in 2D camera frame: u={}, v={}".format(u, v))
        if model_name.startswith("warehouse") or model_name.startswith("aruco"):
            points.append((model_name, (u, v)))

    # get an image from the color channel
    bridge = CvBridge()
    msg = rospy.wait_for_message("/camera/color/image_raw", Image)
    img = bridge.imgmsg_to_cv2(msg, msg.encoding)
    for model_name, (u, v) in points:
        rospy.loginfo(f"name={model_name}, u={u}, v={v}")
        cv2.circle(img, (int(u), int(v)), 5, (0, 0, 255), -1)
    cv2.imshow("image", img)

    while not rospy.is_shutdown():
        if (
            cv2.waitKey(20) & 0xFF in [27, ord("q")]
            or cv2.getWindowProperty("image", cv2.WND_PROP_VISIBLE) < 1
        ):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
