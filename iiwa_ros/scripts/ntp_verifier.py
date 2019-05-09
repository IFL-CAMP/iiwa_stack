#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import JointState
from iiwa_msgs.msg import CartesianPose

rospy.init_node('ntp_verifier')

last_msg = None

def ntplog(msg):
	global last_msg
	last_msg = msg

def print_msg(msg):
  if msg is not None:
    diff = msg.poseStamped.header.stamp - rospy.Time.now()
    print('The current time difference is: {} secs'.format(diff.to_sec()))

sub = rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, ntplog, queue_size=1)

while not rospy.is_shutdown():
	print_msg(last_msg)
