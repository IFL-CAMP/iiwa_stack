#!/usr/bin/env python2

# Copyright (C) 2016-2019
#
# David Wuthier - daw@mp.aau.dk
# Aalborg University
# Robotics, Vision and Machine Intelligence Laboratory
# Department of Materials and Production
# A. C. Meyers Vaenge 15, 2450 Copenhagen SV, Denmark
# http://rvmi.aau.dk/
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.


import rospy

from iiwa_msgs.msg import JointPosition
from iiwa_msgs.srv import ConfigureControlMode, ConfigureControlModeRequest, ConfigureControlModeResponse
from iiwa_msgs.srv import SetSmartServoJointSpeedLimits, SetSmartServoJointSpeedLimitsRequest, SetSmartServoJointSpeedLimitsResponse
from iiwa_msgs.srv import SetSmartServoLinSpeedLimits, SetSmartServoLinSpeedLimitsRequest, SetSmartServoLinSpeedLimitsResponse
from numpy import pi, sqrt, cos, sin, arctan2, array, matrix
from numpy.linalg import norm
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from rospy import DEBUG, INFO, logdebug, loginfo, loginfo_throttle, logwarn, logerr
from rospy import Subscriber, Publisher, Service, ServiceProxy
from rospy import ROSException
from rospy import init_node, get_param, spin
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from std_msgs.msg import Float64, Header
from time import clock

def linearlyMap(x, x1, x2, y1, y2):
  return (y2 - y1)/(x2 - x1) * (x - x1) + y1

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

class IiwaSunrise(object):
  def __init__(self):
    init_node('iiwa_sunrise', log_level = INFO)

    hardware_interface = get_param('~hardware_interface', 'PositionJointInterface')
    self.robot_name = get_param('~robot_name', 'iiwa')
    model = get_param('~model', 'iiwa14')
    tool_length = get_param('~tool_length', 0.0)

    if model == 'iiwa7':
      self.l02 = 0.34
      self.l24 = 0.4
    elif model == 'iiwa14':
      self.l02 = 0.36
      self.l24 = 0.42
    else:
      logerr('unknown robot model')
      return

    self.l46 = 0.4
    self.l6E = 0.126 + tool_length

    self.tr = 0.0
    self.v = 1.0

    self.joint_names = ['{}_joint_1'.format(self.robot_name),
                        '{}_joint_2'.format(self.robot_name),
                        '{}_joint_3'.format(self.robot_name),
                        '{}_joint_4'.format(self.robot_name),
                        '{}_joint_5'.format(self.robot_name),
                        '{}_joint_6'.format(self.robot_name),
                        '{}_joint_7'.format(self.robot_name)]

    joint_states_sub = Subscriber('joint_states', JointState, self.jointStatesCb, queue_size = 1)
    command_pose_sub = Subscriber('command/CartesianPose', PoseStamped, self.commandPoseCb, queue_size = 1)
    command_pose_lin_sub = Subscriber('command/CartesianPoseLin', PoseStamped, self.commandPoseLinCb, queue_size = 1)
    redundancy_sub = Subscriber('command/redundancy', Float64, self.redundancyCb, queue_size = 1)
    joint_position_sub = Subscriber('command/JointPosition', JointPosition, self.jointPositionCb, queue_size = 1)

    self.state_pose_pub = Publisher('state/CartesianPose', PoseStamped, queue_size = 1)
    self.joint_trajectory_pub = Publisher(
        '{}_trajectory_controller/command'.format(hardware_interface), JointTrajectory, queue_size = 1)

    path_parameters_configuration_srv = Service(
        'configuration/setSmartServoLimits', SetSmartServoJointSpeedLimits, self.handlePathParametersConfiguration)
    path_parameters_lin_configuration_srv = Service(
        'configuration/setSmartServoLinLimits', SetSmartServoLinSpeedLimits, self.handlePathParametersLinConfiguration)
    smart_servo_configuration_srv = Service(
        'configuration/ConfigureControlMode', ConfigureControlMode, self.handleSmartServoConfiguration)

    spin()

  def jointPositionCb(self, msg):
    self.publishJointPositionCommand(
        [msg.position.a1, msg.position.a2, msg.position.a3, msg.position.a4, msg.position.a5, msg.position.a6, msg.position.a7])

  def handleSmartServoConfiguration(self, request):
    return ConfigureControlModeResponse(True, '')

  def handlePathParametersConfiguration(self, request):
    loginfo('setting path parameters')

    v = request.joint_relative_velocity

    if v >= 0.0 and v <= 1.0:
      self.v = linearlyMap(v, 0.0, 1.0, 2.0, 0.5)
      return SetSmartServoJointSpeedLimitsResponse(True, '')
    else:
      return SetSmartServoJointSpeedLimitsResponse(False, '')

  def handlePathParametersLinConfiguration(self, request):
    loginfo('setting path parameters linear')

    v = request.max_cartesian_velocity.linear.x

    if v >= 0.0 and v <= 1000.0:
      self.v = linearlyMap(v, 0.0, 1000.0, 2.0, 0.5)
      return SetSmartServoLinSpeedLimitsResponse(True, '')
    else:
      return SetSmartServoLinSpeedLimitsResponse(False, '')

  def redundancyCb(self, msg):
    self.tr = msg.data

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
          header = Header(
            frame_id = '{}_link_0'.format(self.robot_name)),
          pose = Pose(
            position = Point(
              x = H0E[0,3], y = H0E[1,3], z = H0E[2,3]),
            orientation = Quaternion(
              x = q0E[0], y = q0E[1], z = q0E[2], w = q0E[3]))))

  def commandPoseCb(self, msg):
    T0 = clock()

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

    s = norm(p260)

    if s > self.l24 + self.l46:
      logwarn('invalid pose command')
      return

    (tys, tzs) = rr(p260)
    tp24z0 = 1/(2.0 * s) * (self.l24**2 - self.l46**2 + s**2)
    tp240 = matrix([[-sqrt(self.l24**2 - tp24z0**2)], [0.0], [tp24z0]])
    p240 = Ryz(tys, tzs) * Rz(self.tr) * tp240
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
    RE6 = R60.T * RE0
    t[6] = arctan2(RE6[1,0], RE6[0,0])

    self.publishJointPositionCommand(t)

    logdebug('timing: %s ms', 1.0e3 * (clock() - T0))

  def commandPoseLinCb(self, msg):
    self.commandPoseCb(msg)

  def publishJointPositionCommand(self, t):
    jtp = JointTrajectoryPoint()
    jtp.positions = t
    jtp.time_from_start = rospy.Duration.from_sec(self.v)
    jt = JointTrajectory()
    jt.joint_names = self.joint_names
    jt.points.append(jtp)
    self.joint_trajectory_pub.publish(jt)

if __name__ == "__main__":
  ik = IiwaSunrise()
