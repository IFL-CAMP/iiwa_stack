/**
 * Copyright (C) 2016 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided
 * that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package de.tum.in.camp.kuka.ros;

import geometry_msgs.Pose;

import java.util.Arrays;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix4d;
import javax.vecmath.Quat4d;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.time.TimeProvider;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;

/**
 * This class helps building iiwa_msgs ROS messages, it's a collection of methods to build the messages from
 * the current state of a LBR iiwa Robot. For Cartesian messages, it's possible to pass a reference frames, if
 * no reference frames is passed, the flange frame is used.
 */
public class MessageGenerator {

  private static String baseFrameID;
  private static final String baseFrameIDSuffix = "_link_0";
  private static String[] joint_names;

  private static double[] last_position;
  private static long last_position_time_ns = 0;

  // Objects to create ROS messages
  private NodeConfiguration nodeConf = NodeConfiguration.newPrivate();
  private MessageFactory messageFactory = nodeConf.getTopicMessageFactory();
  private TimeProvider time;

  public MessageGenerator(String robotName, TimeProvider timeProvider) {
    baseFrameID = robotName + baseFrameIDSuffix; // e.g. if robotName == iiwa,
                                                 // then baseFrameID =
                                                 // iiwa_link_0

    // e.g. if robotName == iiwa, the joints are iiwa_joint_1, iiwa_joint_2, ...
    joint_names = new String[] { robotName + "_joint_1", robotName + "_joint_2", robotName + "_joint_3", robotName + "_joint_4", robotName + "_joint_5", robotName + "_joint_6",
        robotName + "_joint_7" };
    time = timeProvider;
  }

  /**
   * Builds a geometry_msgs.PoseStamped message given a LBR iiwa Robot.
   * <p>
   * The Cartesian position will be the obtained from current Flange frame, the message header is set to
   * current time, poses will be relative to the robot base frame.<br>
   * 
   * @param currentPose : the PoseStamped message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   */
  public void getCurrentCartesianPose(iiwa_msgs.CartesianPose currentPose, LBR robot) {
    getCurrentCartesianPose(currentPose, robot, robot.getFlange());
  }

  /**
   * Builds a geometry_msgs.PoseStamped message given a LBR iiwa Robot and a frame of reference.
   * <p>
   * The Cartesian position will be the obtained from the given Frame, the message header is set to current
   * time, poses will be relative to the robot base frame.<br>
   * 
   * @param currentPose : the PoseStamped message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   * @param frame : reference frame to set the values of the Cartesian position.
   */
  public void getCurrentCartesianPose(iiwa_msgs.CartesianPose currentPose, LBR robot, ObjectFrame frame) {
    Frame cartesianFrame = robot.getCurrentCartesianPosition(frame);
    Transformation transform = cartesianFrame.transformationFromWorld();

    currentPose.getPoseStamped().getHeader().setFrameId(baseFrameID);
    currentPose.getPoseStamped().getHeader().setStamp(time.getCurrentTime());

    Conversions.kukaTransformationToRosPose(transform, currentPose.getPoseStamped().getPose());

    LBRE1Redundancy redundancy = (LBRE1Redundancy) cartesianFrame.getRedundancyInformationForDevice(robot);
    currentPose.getRedundancy().setE1(redundancy.getE1());
    currentPose.getRedundancy().setStatus(redundancy.getStatus());
    currentPose.getRedundancy().setTurn(redundancy.getTurn());
  }

  /**
   * Builds a geometry_msgs.WrenchStamped message given a LBR iiwa Robot.
   * <p>
   * The wrench values will refer to the Flange frame, the message header is set to current time and according
   * frame name.<br>
   * 
   * @param currentWrench : the WrenchStamped message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   */
  public void getCurrentCartesianWrench(iiwa_msgs.CartesianWrench currentWrench, LBR robot) {
    getCurrentCartesianWrench(currentWrench, robot, robot.getFlange());
  }

  /**
   * Builds a geometry_msgs.WrenchStamped message given a LBR iiwa Robot and a frame of reference.
   * <p>
   * The wrench values will refer to the given frame, the message header is set to current time and according
   * frame name.<br>
   * 
   * @param currentWrench : the WrenchStamped message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   * @param frame : reference frame the wrench refers to.
   */
  public void getCurrentCartesianWrench(iiwa_msgs.CartesianWrench currentWrench, LBR robot, ObjectFrame frame) {
    currentWrench.getHeader().setFrameId(frame.getName());
    currentWrench.getHeader().setStamp(time.getCurrentTime());

    ForceSensorData forceData = robot.getExternalForceTorque(frame);

    currentWrench.getWrench().getForce().setX(forceData.getForce().getX());
    currentWrench.getWrench().getForce().setY(forceData.getForce().getY());
    currentWrench.getWrench().getForce().setZ(forceData.getForce().getZ());

    currentWrench.getWrench().getTorque().setX(forceData.getTorque().getX());
    currentWrench.getWrench().getTorque().setY(forceData.getTorque().getY());
    currentWrench.getWrench().getTorque().setZ(forceData.getTorque().getZ());

    currentWrench.getInaccuracy().getForce().setX(forceData.getForceInaccuracy().getX());
    currentWrench.getInaccuracy().getForce().setY(forceData.getForceInaccuracy().getY());
    currentWrench.getInaccuracy().getForce().setZ(forceData.getForceInaccuracy().getZ());

    currentWrench.getInaccuracy().getTorque().setX(forceData.getTorqueInaccuracy().getX());
    currentWrench.getInaccuracy().getTorque().setY(forceData.getTorqueInaccuracy().getY());
    currentWrench.getInaccuracy().getTorque().setZ(forceData.getTorqueInaccuracy().getZ());
  }

  /**
   * Builds an iiwa_msgs.JointPosition message given a LBR iiwa Robot.
   * <p>
   * The message header is set to current time.<br>
   * 
   * @param currentJointPosition : the JointPosition message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   */
  public void getCurrentJointPosition(iiwa_msgs.JointPosition currentJointPosition, LBR robot) {
    double[] position = robot.getCurrentJointPosition().getInternalArray();
    currentJointPosition.getHeader().setStamp(time.getCurrentTime());
    Conversions.vectorToJointQuantity(position, currentJointPosition.getPosition());
  }

  /**
   * Builds a iiwa_msgs.JointPositionVelocity message given a LBR iiwa Robot.
   * <p>
   * The message header is set to current time.<br>
   * 
   * @param currentJointPositionVelocity : the JointPositionVelocity message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   */
  public void getCurrentJointPositionVelocity(iiwa_msgs.JointPositionVelocity currentJointPositionVelocity, LBR robot) {
    double[] position = robot.getCurrentJointPosition().getInternalArray();
    currentJointPositionVelocity.getHeader().setStamp(time.getCurrentTime());

    Conversions.vectorToJointQuantity(position, currentJointPositionVelocity.getPosition());
    Conversions.vectorToJointQuantity(computeVelocity(robot), currentJointPositionVelocity.getVelocity());
  }

  /**
   * Builds a iiwa_msgs.JointVelocity message given a LBR iiwa Robot.
   * <p>
   * 
   * @param currentJointVelocity : the JointVelocity message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   */
  public void getCurrentJointVelocity(iiwa_msgs.JointVelocity currentJointVelocity, LBR robot) {
    Conversions.vectorToJointQuantity(computeVelocity(robot), currentJointVelocity.getVelocity());
  }

  // TODO: doc
  private double[] computeVelocity(LBR robot) {
    double[] position = robot.getCurrentJointPosition().getInternalArray();
    long position_time_ns = System.nanoTime();
    double[] velocity = new double[robot.getJointCount()];

    if (last_position_time_ns != 0) {
      for (int i = 0; i < robot.getJointCount(); i++)
        velocity[i] = (position[i] - last_position[i]) / ((double) (position_time_ns - last_position_time_ns) / 1000000000);
    }
    last_position = position;
    last_position_time_ns = position_time_ns;

    return velocity;
  }

  /**
   * Builds a iiwa_msgs.JointStiffness message given a LBR iiwa Robot.
   * <p>
   * The message header is set to current time.
   * 
   * @param currentJointStiffness : the JointStiffness message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   * @param motion : the current robot motion, used to get the stiffness values.
   */
  public void getCurrentJointStiffness(iiwa_msgs.JointStiffness currentJointStiffness, LBR robot, SmartServo motion) {
    if (motion == null) return;

    double[] stiffness = new double[robot.getJointCount()];

    try {
      stiffness = ((JointImpedanceControlMode) motion.getMode()).getStiffness();
    }
    catch (Exception e) {
      Logger.error("Asking for joint stiffness while not in joint impedance mode!");
      return;
    }

    currentJointStiffness.getHeader().setStamp(time.getCurrentTime());
    Conversions.vectorToJointQuantity(stiffness, currentJointStiffness.getStiffness());
  }

  /**
   * Builds a iiwa_msgs.JointDamping message given a LBR iiwa Robot.
   * <p>
   * The message header is set to current time.
   * 
   * @param currentJointDamping : the JointDamping message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   * @param motion : the current robot motion, used to get the damping values.
   */
  public void getCurrentJointDamping(iiwa_msgs.JointDamping currentJointDamping, LBR robot, SmartServo motion) {
    if (motion == null) return;

    double[] damping = new double[robot.getJointCount()];

    try {
      damping = ((JointImpedanceControlMode) motion.getMode()).getDamping();
    }
    catch (Exception e) {
      Logger.error("Asking for joint damping while not in joint impedance mode!");
      return;
    }

    currentJointDamping.getHeader().setStamp(time.getCurrentTime());
    Conversions.vectorToJointQuantity(damping, currentJointDamping.getDamping());
  }

  /**
   * Builds a iiwa_msgs.JointTorque message given a LBR iiwa Robot.
   * <p>
   * The message header is set to current time.<br>
   * 
   * @param currentJointTorque : the JointTorque message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   */
  public void getCurrentJointTorque(iiwa_msgs.JointTorque currentJointTorque, LBR robot) {
    double[] torque = robot.getMeasuredTorque().getTorqueValues();
    currentJointTorque.getHeader().setStamp(time.getCurrentTime());
    Conversions.vectorToJointQuantity(torque, currentJointTorque.getTorque());
  }

  /**
   * Builds a iiwa_msgs.JointTorque message containing the external torque applied to the given LBR iiwa
   * Robot.
   * <p>
   * The message header is set to current time.<br>
   * 
   * @param currentExternalJointTorque : the JointTorque message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   */
  public void getCurrentExternalJointTorque(iiwa_msgs.JointTorque currentExternalJointTorque, LBR robot) {
    double[] torque = robot.getExternalTorque().getTorqueValues();
    currentExternalJointTorque.getHeader().setStamp(time.getCurrentTime());
    Conversions.vectorToJointQuantity(torque, currentExternalJointTorque.getTorque());
  }

  /**
   * Builds a sensor_msgs.JointState message given a LBR iiwa Robot.
   * <p>
   * <b>No velocity information currently available</b>
   * 
   * @param currentJointState : the JointState message that will be created.
   * @param robot : an iiwa Robot, its current state is used to set the values of the message.
   */
  public void getCurrentJointState(sensor_msgs.JointState currentJointState, LBR robot) {

    currentJointState.getHeader().setStamp(time.getCurrentTime());
    currentJointState.setName(Arrays.asList(joint_names));
    currentJointState.setPosition(robot.getCurrentJointPosition().getInternalArray());
    currentJointState.setEffort(robot.getMeasuredTorque().getTorqueValues());
  }

  /**
   * Create a ROS Message of the given type.
   * 
   * @param typeString : type of the ROS message to build, e.g. PoseStamped._TYPE
   * @return created ROS message
   */
  public <T extends org.ros.internal.message.Message> T buildMessage(String typeString) {
    return messageFactory.newFromType(typeString);
  }

  /**
   * Adds one to the current sequence number of the message Header
   * 
   * @param h : message Header
   */
  public void incrementSeqNumber(std_msgs.Header h) {
    h.setSeq(h.getSeq() + 1);
  }

  /**
   * Checks if all the component of a CartesianQuantity are greater than the given value.
   * 
   * @param quantity
   * @param value
   * @return
   */
  public boolean isCartesianQuantityGreaterThan(iiwa_msgs.CartesianQuantity quantity, int value) {
    return (quantity.getX() > value && quantity.getY() > value && quantity.getZ() > value && quantity.getA() > value && quantity.getB() > value && quantity.getC() > value);
  }

  /**
   * Checks if all the component of a CartesianQuantity are greater or equal than the given value.
   * 
   * @param quantity
   * @param value
   * @return
   */
  public boolean isCartesianQuantityGreaterEqualThan(iiwa_msgs.CartesianQuantity quantity, int value) {
    return (quantity.getX() >= value && quantity.getY() >= value && quantity.getZ() >= value && quantity.getA() >= value && quantity.getB() >= value && quantity.getC() >= value);
  }

  /**
   * Checks if all the component of a JointQuantity are greater than the given value.
   * 
   * @param value
   * @return
   */
  public boolean isJointQuantityGreaterThan(iiwa_msgs.JointQuantity quantity, int value) {
    return (quantity.getA1() > value && quantity.getA2() > value && quantity.getA3() > value && quantity.getA4() > value && quantity.getA5() > value && quantity.getA6() > value && quantity
        .getA7() > value);
  }

  /**
   * 
   * Checks if all the component of a JointQuantity are greater or equal than the given value.
   * 
   * @param value
   * @return
   */
  public boolean isJointQuantityGreaterEqualThan(iiwa_msgs.JointQuantity quantity, int value) {
    return (quantity.getA1() >= value && quantity.getA2() >= value && quantity.getA3() >= value && quantity.getA4() >= value && quantity.getA5() >= value
        && quantity.getA6() >= value && quantity.getA7() >= value);
  }

  public geometry_msgs.Pose getPose(Matrix4d mat) {
    Matrix3d base = new Matrix3d(mat.getM00(), mat.getM01(), mat.getM02(), mat.getM10(), mat.getM11(), mat.getM12(), mat.getM20(), mat.getM21(), mat.getM22());
    Quat4d q = new Quat4d();
    q.set(base);

    Pose result = buildMessage(Pose._TYPE);
    result.getOrientation().setX(q.getX());
    result.getOrientation().setY(q.getY());
    result.getOrientation().setZ(q.getZ());
    result.getOrientation().setW(q.getW());
    result.getPosition().setX(mat.getM03());
    result.getPosition().setY(mat.getM13());
    result.getPosition().setZ(mat.getM23());

    return result;
  }

  /**
   * Returns the current timestamp.
   * 
   * @return
   */
  public org.ros.message.Time getCurrentTime() {
    return time.getCurrentTime();
  }
}
