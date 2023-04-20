# project for pick and place simulation

## quickstart

### docker

You can use docker to run most of the simulation.
You will not be able to run the grasping network module if you do not have an nvidia gpu.
Make sure you've checked out the repository with submodules:

```bash
git clone --recurse-submodules git@github.com:Autobots-Visman/pick-and-place.git

# or after the fact
git submodule update --init --recursive
```

When you pull changes, make sure to update the submodule as well:

```bash
git pull
git submodule update --recursive
```

Let's grant docker access to the X11 display, which is used throughout the program for visualization.

```bash
xhost +local:docker
```

Build the docker images:

```bash
docker compose build
```

Then run the tests to verify that everything is working:

```bash
docker compose run --rm base catkin test \
    autobots_realsense2_description

# TODO: calibration tests are currently broken
# TODO: handy simulation is missing tests
```

Also run the gknet tests to verify that it is working correctly:

```bash
docker compose run --rm --no-deps gknet catkin test
```

Also verify that you can successfully launch the simulation via docker.

```bash
# manually running using the base command
docker compose run --rm base-gpu roslaunch autobots_handy_simulation gazebo.launch
```

Now you can run the entire simulation via docker:

```bash
docker compose up
```

Then spawn some objects for testing:

```bash
docker compose run --rm base rosrun autobots_handy_simulation spawn_random_objects.py
```

There are few Makefile targets to make the syntax here a bit easier:

```bash
make build
make up
make down
```

#### WSL2

If you're using WSL2 with docker and a GPU, you may need to use the following docker override for gazebo to run properly:

```bash
docker compose -f docker-compose.yml -f docker-compose.wsl.yml up
```

This can be shorted to the following Makefile target:

```bash
make up-wsl
```

#### Clean up your containers

Make sure you clean up the docker containers after you're done:

```bash
docker compose down
```

When you run `docker ps`, there should be no running containers.

#### Docker tidbits

As a note to the docker neophytes, here are some of the flags above:

- `--rm`: remove the container after it exits.
- `--no-deps`: do not start the dependencies of the service. We might use this if we want to run a container without starting the `core` container which is the master node for coordinating between services.

### (manual) dependencies and building

It's also useful to be able to run the packages from a host machine.
Rviz is going to be significantly more performant on the host, and it can be nice to start up random ROS utilities.

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
git clone https://github.com/ivaROS/ivaDynamixel.git
git clone https://github.com/ivaROS/ivaHandy.git
git clone --recurse-submodules https://github.com/rickstaa/realsense-ros-gazebo.git
git clone https://github.com/ivalab/simData.git
git clone https://github.com/ivalab/GraspKpNet.git

# clone this repo
git clone --recurse-submodules https://github.com/Autobots-Visman/pick-and-place.git
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
