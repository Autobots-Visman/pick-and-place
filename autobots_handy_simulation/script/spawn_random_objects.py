#!/usr/bin/env python3
"""Spawn random objects from the simData package."""
import random
from argparse import ArgumentParser
from pathlib import Path

import rospkg
import rospy
from gazebo_msgs.srv import SetPhysicsProperties, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion

models = {
    "bowl": 7,
    "cup": 6,
    "hammer": 4,
    "knife": 12,
    "ladle": 3,
    "mallet": 4,
    "mug": 20,
    "pot": 2,
    "saw": 3,
    "scissors": 8,
    "scoop": 2,
    "shears": 2,
    "shovel": 2,
    "spoon": 10,
    "tenderizer": 1,
    "trowel": 3,
    "turner": 7,
}


def random_pose():
    margin = 0.1
    return Pose(
        Point(
            random.uniform(0 + margin, 0.8 - margin),
            random.uniform(-0.75 + margin, 0.75 - margin),
            1.03,
        ),
        # random rotation along the table plane
        Quaternion(0, 0, 0, 0),
    )


def parse_args():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=3)
    # ignore any other args
    args, _ = parser.parse_known_args()
    return args


def main():
    args = parse_args()

    # spawn an sdf model via gazebo into a random location sitting in a 1.5x4
    # box about a meter off the ground
    rospy.init_node("spawn_random", anonymous=True)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    set_physics = rospy.ServiceProxy(
        "/gazebo/set_physics_properties", SetPhysicsProperties
    )

    rospack = rospkg.RosPack()
    warehouse_path = rospack.get_path("warehouse")

    for _ in range(args.count):
        model_type = random.choice(list(models.keys()))
        model_index = random.randint(1, models[model_type])
        model_name = f"{model_type}_{model_index:02}"
        model_path = f"{warehouse_path}/models/{model_name}/sdf/description.sdf"

        xml_data = Path(model_path).read_text()
        # replace static property
        # xml_data = xml_data.replace("<static>true</static>", "<static>false</static>")

        spawn_model(model_name, xml_data, "/", random_pose(), "world")
        # set gravity on model
        set_physics(gravity=Point(0, 0, -9.81))


if __name__ == "__main__":
    main()
