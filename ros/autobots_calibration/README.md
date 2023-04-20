# autobots calibration

Camera calibration is the task of determining where the camera is relative to the robot given a fixed landmark.
We use a single ArUco marker to calibrate the camera.

We publish the following topics:

```
/calibration/camera/matrix_camera_link
/calibration/camera/is_stable
/calibration/camera/camera_info
/calibration/robot/matrix_base_link
```

We have three main scripts in this package.
The first spawns an ArUco marker harness with a known position.

1. Spawn an ArUco tag
1. Wait for the calibration topic topic to become stable
1. Destroy the ArUco tag

The second script is a node that performs the actual calibration process.

1. Republish the camera info topic
1. Wait until an ArUco tag is detected
1. Perform calibration; wait until transformation matrix is stable
1. Publish the transformation matrix
1. Publish calibration status (stable/unstable)

The third script is a node that publishes the transformation matrix from the ArUco tag to the camera.
This script provides an API that other pose estimation nodes can use to get the transformation matrix, without hard coding the values.

## notes

AFter running the demo, we can check on the transformations being published:

```bash
roslaunch autobots_calibration demo.launch
```

```bash
rosrun tf tf_echo /aruco_tile/base_link /camera/base_link

# use this to ensure that the topics
rosrun tf2_tools view_frames.py
```

The transformation from the aruco tag to the camera base link is not quite right, because the camera lense is not at the base of the camera.
However, for the purposes of the simulation, this should actually be good enough.
