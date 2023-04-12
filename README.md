# project for pick and place simulation

## quickstart

### dependencies and building

Ensure you've installed ros-noetic-desktop on ubuntu 20.04 packages and create a new catkin workspace.

```bash
apt install ros-noetic-desktop

# create a new catkin workspace, you can name it whatever you want
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# initialize the workspace
source /opt/ros/noetic/setup.bash
catkin init
```

We focus on the [handy robot](https://github.com/ivaROS/ivaHandy).
You'll need to include a particular implementation of the realsense-ros-gazebo plugin in your workspace.

```bash
git clone git@github.com:ivaROS/ivaDynamixel.git
git clone git@github.com:ivaROS/ivaHandy.git
git clone --recurse-submodules git@github.com:rickstaa/realsense-ros-gazebo.git
git clone git@github.com:acmiyaguchi/GraspKpNet.git

# clone this repo
git clone --recurse-submodules git@github.com:Autobots-Visman/pick-and-place.git
```

Install all the other dependencies via [`rosdep`](http://wiki.ros.org/rosdep):

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### creating a new project

To create a new project:

```bash
catkin create pkg --rosdistro noetic ${package_name}
```

### running the simulation

```bash
# launch the gazebo world
roslaunch autobots_handy_simulation demo.launch

# spawn some random objects
rosrun autobots_handy_simulation spawn_random_objects.py

# TODO: configure a docker container from this repository
# from GraspKpNet

docker compose run --rm gpu \
    roslaunch gknet_perception detect.launch \
        color_image_topic:=/camera/color/image_raw \
        depth_image_topic:=/camera/aligned_depth_to_color/image_raw
```
