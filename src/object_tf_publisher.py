#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
import tf

class TF_broadcaster():
    def __init__(self):
        self.model = rospy.myargv()[2]
        rospy.init_node(self.model + "_tf_publisher")
        self.sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback, queue_size=10)
        self.br = tf.TransformBroadcaster()
        rospy.wait_for_message("/gazebo/model_states", ModelStates)

    def callback(self, msg):
        index = msg.name.index(self.model)
        pose = msg.pose[index].position
        rot = msg.pose[index].orientation
        self.br.sendTransform((pose.x, pose.y, pose.z), (rot.x, rot.y, rot.z, rot.w), rospy.Time.now(), self.model, "map")

if __name__ == "__main__":
    TF = TF_broadcaster()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        r.sleep()