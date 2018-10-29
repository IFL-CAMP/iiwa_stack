#!/usr/bin/env python2

import rospy

from numpy import cos, sin, array, matrix
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from rospy import DEBUG, logdebug, loginfo, loginfo_throttle, logwarn, logerr
from rospy import Subscriber, Publisher, Service, ServiceProxy
from rospy import ROSException
from rospy import init_node, get_param, spin
from tf.transformations import quaternion_from_matrix

def Hrrt(ty, tz, l):
  cy = cos(ty)
  sy = sin(ty)
  cz = cos(tz)
  sz = sin(tz)
  return matrix([[cy * cz, -sz, sy * cz, 0.0],
                 [cy * sz, cz, sy * sz, 0.0],
                 [-sy, 0.0, cy, l],
                 [0.0, 0.0, 0.0, 1.0]])

class IiwaKinematics(object):
  def __init__(self):
    init_node('iiwa_kinematics', log_level = DEBUG)

    joint_states_sub = Subscriber('joint_states', JointState, self.jointStatesCb, queue_size = 1)
    command_pose_sub = Subscriber('command/CartesianPose', PoseStamped, self.commandPoseCb, queue_size = 1)

    self.state_pose_pub = Publisher('state/CartesianPose', PoseStamped, queue_size = 1)
    self.joint_trajectory_pub = Publisher('EffortJointInterface_trajectory_controller/command', JointTrajectory, queue_size = 1)

    self.joint_names = ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4',
                        'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7']
    self.l02 = 0.34
    self.l24 = 0.4
    self.l46 = 0.4
    self.l6E = 0.126

    spin()

  def jointStatesCb(self, msg):
    t = msg.position

    H02 = Hrrt(t[1], t[0], self.l02)
    H24 = Hrrt(-t[3], t[2], self.l24)
    H46 = Hrrt(t[5], t[4], self.l46)
    H6E = Hrrt(0.0, t[6], self.l6E)

    H0E = H02 * H24 * H46 * H6E
    q0E = quaternion_from_matrix(H0E)

    self.state_pose_pub.publish(
        PoseStamped(
          pose = Pose(
            position = Point(
              x = H0E[0,3], y = H0E[1,3], z = H0E[2,3]),
            orientation = Quaternion(
              x = q0E[0], y = q0E[1], z = q0E[2], w = q0E[3]))))

  def commandPoseCb(self, msg):
    jtp = JointTrajectoryPoint()
    jtp.positions = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0]
    jtp.time_from_start = rospy.Duration.from_sec(0.5)
    jt = JointTrajectory()
    jt.joint_names = self.joint_names
    jt.points.append(jtp)
    self.joint_trajectory_pub.publish(jt)

if __name__ == "__main__":
  ik = IiwaKinematics()
