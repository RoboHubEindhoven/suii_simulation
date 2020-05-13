#!/usr/bin/env python

import rospy
import rospkg
import os
import random
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.srv import SpawnModel, SetLinkState
from gazebo_msgs.msg import LinkState

class RotatingTable:
    def __init__(self, name):
        self.name = name
        rospy.init_node("rotating_table_node")
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        rospy.wait_for_service("gazebo/set_link_state")

        self.spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
        self.rotate = rospy.ServiceProxy("/gazebo/set_link_state", SetLinkState)

        rp = rospkg.RosPack()
        with open(os.path.join(rp.get_path("suii_simulation"), "urdf", "rotating_table.urdf"), "r") as f:
            self.model = f.read()

    def spawn(self, x, y):
        self.x = x
        self.y = y
        pose = Pose(Point(x=x, y=y, z=0.1), Quaternion(x=0, y=0, z=0, w=1))
        self.spawner(self.name, self.model, "", pose, "world")

    def set_speed(self, speed):
        msg = LinkState()
        msg.link_name = "table"
        msg.pose = Pose(Point(x=self.x, y=self.y, z=0.1), Quaternion(x=0, y=0, z=0, w=1))
        msg.twist.linear.x = 0; msg.twist.linear.y = 0; msg.twist.linear.z = 0
        msg.twist.angular.x = 0; msg.twist.angular.y = 0; msg.twist.angular.z = speed
        msg.reference_frame = "base_link"
        self.rotate(msg)

if __name__ == "__main__":
    rt = RotatingTable("Table_1")
    rt.spawn(1,1)
    rt.set_speed(random.choice([-1, 1]) * random.uniform(0.1, 0.3))
