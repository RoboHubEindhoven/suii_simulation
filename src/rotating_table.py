#!/usr/bin/env python

import rospy
import rospkg
import os
import random
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel

class RotatingTable:
    def __init__(self, name):
        self.name = name
        rospy.init_node("rotating_table_node")
        rospy.wait_for_service("gazebo/spawn_urdf_model")

        self.spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.rotate = rospy.Publisher(name + "/speed", Float32, queue_size=10)

        rp = rospkg.RosPack()
        with open(os.path.join(rp.get_path("suii_simulation"), "urdf", "rotating_table.urdf"), "r") as f:
            self.model = f.read()

    def spawn(self, x, y):
        pose = Pose(Point(x=x, y=y, z=0.1), Quaternion(x=0, y=0, z=0, w=1))
        self.spawner(self.name, self.model, "", pose, "world")

    def set_speed(self, speed):
        msg = Float32()
        msg.data = speed
        while not rospy.is_shutdown():
            self.rotate.publish(msg)

if __name__ == "__main__":
    rt = RotatingTable("table_1")
    rt.spawn(1, 1)
    rt.set_speed(random.choice([-1, 1]) * random.uniform(0.1, 0.3))
