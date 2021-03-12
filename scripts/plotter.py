#! /usr/bin/env python
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

from std_msgs.msg import ColorRGBA, Float32
from dynamixel_msgs.msg import JointState
import rospy

def plot_cb (msg):
    val = float(msg.load) #sub msg is float64 but pub msg is float32
    value_pub.publish(val)

if __name__ == '__main__':
    rospy.init_node("plotter_sample")
    value_pub = rospy.Publisher("value_load", Float32, queue_size=1)
    rospy.Subscriber('/lgripper/finger1_joint_controller/state',  JointState, plot_cb)
    rospy.spin()
