#!/usr/bin/env python3
import geometry_msgs.msg
import rospy
import tf2_ros
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from sensor_msgs.msg import CameraInfo
from tf2_geometry_msgs import do_transform_point


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
    fx = cam_info.K[0]
    fy = cam_info.K[4]
    cx = cam_info.K[2]
    cy = cam_info.K[5]
    return fx, fy, cx, cy


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
    aruco_to_camera = wait_for_transform(
        tf_buffer, "aruco_tile/base_link", "camera/base_link"
    )
    world_to_aruco = wait_for_transform(tf_buffer, "world", "aruco_tile/base_link")

    world_properties = get_world_properties()
    for model_name in world_properties.model_names:
        model_state = get_model_state(model_name, "")
        pose = model_state.pose
        rospy.loginfo(f"Model: {model_name}, Pose:\n{pose}")

        # Define a 3D point in the world frame
        p_world = geometry_msgs.msg.PointStamped(
            header=rospy.Header(stamp=rospy.Time.now(), frame_id="world"),
            point=model_state.pose.position,
        )
        p_aruco = do_transform_point(p_world, world_to_aruco)
        p_camera = do_transform_point(p_aruco, aruco_to_camera)

        # Transform the point and print the result
        u, v = do_transform_point_2d(p_camera, fx, fy, cx, cy)
        rospy.loginfo("Point in 2D camera frame: u={}, v={}".format(u, v))


if __name__ == "__main__":
    main()
