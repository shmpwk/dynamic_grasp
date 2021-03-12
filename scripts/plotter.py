#! /usr/bin/env python
try:
  from jsk_rviz_plugins.msg import *
except:
  import roslib;roslib.load_manifest("jsk_rviz_plugins")
  from jsk_rviz_plugins.msg import *

from std_msgs.msg import ColorRGBA, Float32
from dynamixel_msgs.msg import JointState
import rospy
import message_filters

def plot_cb (msg1, msg2, msg3):
    val1 = float(msg1.load) #sub msg is float64 but pub msg is float32
    val2 = float(msg2.load) #sub msg is float64 but pub msg is float32
    val3 = float(msg3.load) #sub msg is float64 but pub msg is float32
    value1_pub.publish(val1)
    value2_pub.publish(val2)
    value3_pub.publish(val3)

if __name__ == '__main__':
    rospy.init_node("plotter_sample")
    value1_pub = rospy.Publisher("value1_load", Float32, queue_size=1)
    value2_pub = rospy.Publisher("value2_load", Float32, queue_size=1)
    value3_pub = rospy.Publisher("value3_load", Float32, queue_size=1)
    sub1 = message_filters.Subscriber('/lgripper/finger1_joint_controller/state', JointState)
    sub2 = message_filters.Subscriber('/lgripper/finger2_joint_controller/state', JointState)
    sub3 = message_filters.Subscriber('/lgripper/finger3_joint_controller/state', JointState)
    delay = 1 / 30
    ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2,sub3], 100, 0.1)
    ts.registerCallback(plot_cb)
    #rospy.Subscriber('/lgripper/finger1_joint_controller/state',  JointState, plot_cb)
    rospy.spin()
