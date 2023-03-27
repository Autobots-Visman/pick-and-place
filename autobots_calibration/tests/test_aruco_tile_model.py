import cv2
import rospy
from sensor_msgs.msg import Image


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
