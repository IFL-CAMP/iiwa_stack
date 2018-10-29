#!/usr/bin/env python2

import rospy

from numpy import pi, sqrt, cos, sin, arctan2, array, matrix
from numpy.linalg import norm
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from rospy import DEBUG, logdebug, loginfo, loginfo_throttle, logwarn, logerr
from rospy import Subscriber, Publisher, Service, ServiceProxy
from rospy import ROSException
from rospy import init_node, get_param, spin
from tf.transformations import quaternion_from_matrix, quaternion_matrix

def R(q):
  return matrix(quaternion_matrix(q)[:3,:3])

def trigonometry(t):
  return (cos(t), sin(t))

def rr(p):
  ty = arctan2(sqrt(p[0,0]**2 + p[1,0]**2), p[2,0])
  tz = arctan2(p[1,0], p[0,0])

  if tz < -pi/2.0:
    ty = -ty
    tz += pi
  elif tz > pi/2.0:
    ty = -ty
    tz -= pi

  return (ty, tz)

def Rz(tz):
  (cz, sz) = trigonometry(tz)
  return matrix([[ cz, -sz, 0.0],
                 [ sz,  cz, 0.0],
                 [0.0, 0.0, 1.0]])

def Ryz(ty, tz):
  (cy, sy) = trigonometry(ty)
  (cz, sz) = trigonometry(tz)
  return matrix([[cy * cz, -sz, sy * cz],
                 [cy * sz, cz, sy * sz],
                 [-sy, 0.0, cy]])

def Hrrt(ty, tz, l):
  (cy, sy) = trigonometry(ty)
  (cz, sz) = trigonometry(tz)
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
    t = 7 * [0.0]
    pE0 = matrix([[msg.pose.position.x],
                  [msg.pose.position.y],
                  [msg.pose.position.z]])
    qE0 = array([msg.pose.orientation.x,
                 msg.pose.orientation.y,
                 msg.pose.orientation.z,
                 msg.pose.orientation.w])

    pE6 = matrix([[0.0], [0.0], [self.l6E]])
    p20 = matrix([[0.0], [0.0], [self.l02]])

    RE0 = R(qE0)
    p6E0 = RE0 * pE6
    p60 = pE0 - p6E0
    p260 = p60 - p20

    (tys, tzs) = rr(p260)
    s = norm(p260)
    tp24z0 = 1/(2.0 * s) * (self.l24**2 - self.l46**2 + s**2)
    tp240 = matrix([[-sqrt(self.l24**2 - tp24z0**2)], [0.0], [tp24z0]])
    p240 = Ryz(tys, tzs) * Rz(0.0) * tp240
    (t[1], t[0]) = rr(p240)

    R20 = Ryz(t[1], t[0])
    p40 = p20 + p240
    p460 = p60 - p40
    p462 = R20.T * p460
    (t[3], t[2]) = rr(p462)
    t[3] = -t[3]

    R42 = Ryz(-t[3], t[2])
    R40 = R20 * R42
    p6E4 = R40.T * p6E0
    (t[5], t[4]) = rr(p6E4)

    R64 = Ryz(t[5], t[4])
    R60 = R40 * R64
    RE6 = R60 * RE0
    t[6] = arctan2(RE6[1,0], RE6[0,0])

    jtp = JointTrajectoryPoint()
    jtp.positions = t
    jtp.time_from_start = rospy.Duration.from_sec(1.0)
    jt = JointTrajectory()
    jt.joint_names = self.joint_names
    jt.points.append(jtp)
    self.joint_trajectory_pub.publish(jt)

if __name__ == "__main__":
  ik = IiwaKinematics()
