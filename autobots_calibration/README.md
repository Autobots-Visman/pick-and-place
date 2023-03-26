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
