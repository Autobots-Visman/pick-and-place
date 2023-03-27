import cv2
import numpy as np
import rospy
from camera.extrinsic.aruco import CtoW_Calibrator_aruco
from sensor_msgs.msg import CameraInfo, Image


def test_aruco_tag_is_detected(cv_bridge, tmp_path):
    # let's read a few messages off the topic to make sure the tag is loaded
    for _ in range(10):
        msg = rospy.wait_for_message("/camera/color/image_raw", Image)
        assert msg, "no message received"
    img = cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
    assert img.shape == (480, 640, 3)
    # write image to file for debugging
    cv2.imwrite(str(tmp_path / "aruco_tag.png"), img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters
    )
    assert ids is not None, f"{(corners, ids, rejectedImgPoints)}"
    assert len(ids[0]) == 1
    assert ids.tolist()[0] == [0]


def test_aruco_tag_calibrated_by_ivapylib_camera_module(cv_bridge):
    # get the intrinsic matrix from the camera_info
    msg = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
    assert msg, "no message received"
    intrinsic_mat = np.array(msg.K).reshape(3, 3)

    max_frames = 10
    calibrator = CtoW_Calibrator_aruco(
        intrinsic_mat, markerLength_CL=0.1, maxFrames=max_frames
    )
    assert calibrator.detected is False
    assert calibrator.stable_status is False

    for _ in range(max_frames + 10):
        msg = rospy.wait_for_message("/camera/color/image_raw", Image)
        assert msg, "no message received"
        img = cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
        _, _, _, detected = calibrator.process(img)
        if detected and calibrator.stable_status:
            break
    assert calibrator.detected
    assert calibrator.stable_status

    assert calibrator.M_CL.shape == (4, 4)
    assert calibrator.img_with_ext.shape == (480, 640, 3)
