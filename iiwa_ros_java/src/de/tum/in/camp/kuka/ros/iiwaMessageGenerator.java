/**  
 * Copyright (C) 2016-2017 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
 * Technische Universität München
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultät für Informatik / I16, Boltzmannstraße 3, 85748 Garching bei München, Germany
 * http://campar.in.tum.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

package de.tum.in.camp.kuka.ros;

import geometry_msgs.Pose;
import geometry_msgs.Quaternion;

import java.util.Arrays;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.time.TimeProvider;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixBuilder;
import com.kuka.roboticsAPI.geometricModel.math.MatrixRotation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/**
 * This class helps building iiwa_msgs ROS messages,
 * it's a collection of methods to build the messages from the current state of a LBR iiwa Robot.
 * For Cartesian messages, it's possible to pass a reference frames, if no reference frames is passed, the flange frame is used.
 */
public class iiwaMessageGenerator {

	private static String baseFrameID;
	private static final String baseFrameIDSuffix = "_link_0";
	private static String[] joint_names;

	private static double[] last_position;
	private static long last_position_time_ns = 0;

	// Objects to create ROS messages
	private NodeConfiguration nodeConf = NodeConfiguration.newPrivate();
	private MessageFactory messageFactory = nodeConf.getTopicMessageFactory();
	private TimeProvider time = iiwaConfiguration.getTimeProvider();

	public iiwaMessageGenerator(String robotName) {
		baseFrameID = robotName + baseFrameIDSuffix; // e.g. if robotName == iiwa, then baseFrameID = iiwa_link_0

		// e.g. if robotName == iiwa, the joints are iiwa_joint_1, iiwa_joint_2, ...
		joint_names = new String[]{
				robotName+"_joint_1", 
				robotName+"_joint_2",
				robotName+"_joint_3",
				robotName+"_joint_4",
				robotName+"_joint_5",
				robotName+"_joint_6",
				robotName+"_joint_7"
		};
	}

	/**
	 * Builds a geometry_msgs.PoseStamped message given a LBR iiwa Robot.<p>
	 * The Cartesian position will be the obtained from current Flange frame,
	 * the message header is set to current time, poses will be relative to the robot base frame.<br>
	 * @param currentPose : the PoseStamped message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 */
	public void getCurrentCartesianPose(geometry_msgs.PoseStamped currentPose, LBR robot) {
		getCurrentCartesianPose(currentPose, robot, robot.getFlange());
	}

	/**
	 * Builds a geometry_msgs.PoseStamped message given a LBR iiwa Robot and a frame of reference.<p>
	 * The Cartesian position will be the obtained from the given Frame,
	 * the message header is set to current time, poses will be relative to the robot base frame.<br>
	 * @param currentPose : the PoseStamped message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param frame : reference frame to set the values of the Cartesian position.
	 */
	public void getCurrentCartesianPose(geometry_msgs.PoseStamped currentPose, LBR robot, ObjectFrame frame) {
		Transformation transform = robot.getCurrentCartesianPosition(frame).transformationFromWorld();

		currentPose.getHeader().setFrameId(baseFrameID);  
		currentPose.getHeader().setStamp(time.getCurrentTime());

		kukaTransformationToRosPose(transform, currentPose.getPose());
	}


	/**
	 * Builds a geometry_msgs.WrenchStamped message given a LBR iiwa Robot.<p>
	 * The wrench values will refer to the Flange frame,
	 * the message header is set to current time and according frame name.<br>
	 * @param currentWrench : the WrenchStamped message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 */
	public void getCurrentCartesianWrench(geometry_msgs.WrenchStamped currentWrench, LBR robot) {
		getCurrentCartesianWrench(currentWrench, robot, robot.getFlange());
	}

	/**
	 * Builds a geometry_msgs.WrenchStamped message given a LBR iiwa Robot and a frame of reference.<p>
	 * The wrench values will refer to the given frame,
	 * the message header is set to current time and according frame name.<br> 
	 * @param currentWrench : the WrenchStamped message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param frame : reference frame the wrench refers to.
	 */
	public void getCurrentCartesianWrench(geometry_msgs.WrenchStamped currentWrench, LBR robot, ObjectFrame frame) {
		currentWrench.getHeader().setFrameId(frame.getName());
		currentWrench.getHeader().setStamp(time.getCurrentTime());

		currentWrench.getWrench().getForce().setX(robot.getExternalForceTorque(frame).getForce().getX());
		currentWrench.getWrench().getForce().setY(robot.getExternalForceTorque(frame).getForce().getY());
		currentWrench.getWrench().getForce().setZ(robot.getExternalForceTorque(frame).getForce().getZ());

		currentWrench.getWrench().getTorque().setX(robot.getExternalForceTorque(frame).getTorque().getX());
		currentWrench.getWrench().getTorque().setY(robot.getExternalForceTorque(frame).getTorque().getY());
		currentWrench.getWrench().getTorque().setZ(robot.getExternalForceTorque(frame).getTorque().getZ());

		// TODO : should we also add these: 
		//robot.getExternalForceTorque(frame).getForceInaccuracy();
		//robot.getExternalForceTorque(frame).getTorqueInaccuracy();
		// to give an estimation of the accuracy of the force/torque?
	}

	/**
	 * Builds an iiwa_msgs.JointPosition message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.<br>
	 * @param currentJointPosition : the JointPosition message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 */
	public void getCurrentJointPosition(iiwa_msgs.JointPosition currentJointPosition, LBR robot) {
		double[] position = robot.getCurrentJointPosition().getInternalArray();
		currentJointPosition.getHeader().setStamp(time.getCurrentTime());
		vectorToJointQuantity(position, currentJointPosition.getPosition());
	}

	/**
	 * Builds a iiwa_msgs.JointPositionVelocity message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.<br>
	 * @param currentJointPositionVelocity : the JointPositionVelocity message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 */
	public void getCurrentJointPositionVelocity(iiwa_msgs.JointPositionVelocity currentJointPositionVelocity, LBR robot) {		
		double[] position = robot.getCurrentJointPosition().getInternalArray();
		currentJointPositionVelocity.getHeader().setStamp(time.getCurrentTime());

		vectorToJointQuantity(position, currentJointPositionVelocity.getPosition());
		vectorToJointQuantity(computeVelocity(robot), currentJointPositionVelocity.getVelocity());
	}

	/**
	 * Builds a iiwa_msgs.JointVelocity message given a LBR iiwa Robot.<p>
	 * @param currentJointVelocity : the JointVelocity message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 */
	public void getCurrentJointVelocity(iiwa_msgs.JointVelocity currentJointVelocity, LBR robot) {
		vectorToJointQuantity(computeVelocity(robot), currentJointVelocity.getVelocity());
	}

	private double[] computeVelocity(LBR robot) {
		double[] position = robot.getCurrentJointPosition().getInternalArray();
		long position_time_ns = System.nanoTime();
		double[] velocity = new double[robot.getJointCount()];  

		if (last_position_time_ns != 0) {
			for (int i = 0; i < robot.getJointCount(); i++)
				velocity[i] = (position[i] - last_position[i]) / ((double)(position_time_ns - last_position_time_ns) / 1000000000);
		}
		last_position = position;
		last_position_time_ns = position_time_ns;

		return velocity;
	}

	/**
	 * Builds a iiwa_msgs.JointStiffness message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.
	 * @param currentJointStiffness : the JointStiffness message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param motion : the current robot motion, used to get the stiffness values.
	 */
	public void getCurrentJointStiffness(iiwa_msgs.JointStiffness currentJointStiffness, LBR robot, SmartServo motion) {
		if (motion == null) 
			return;

		double[] stiffness = new double[robot.getJointCount()];

		try {
			stiffness = ((JointImpedanceControlMode) motion.getMode()).getStiffness();
		} catch (Exception e) {
			System.out.println("ERROR: asking for joint stiffness while not in joint impedance mode!");
			return;
		}

		currentJointStiffness.getHeader().setStamp(time.getCurrentTime());
		vectorToJointQuantity(stiffness, currentJointStiffness.getStiffness());
	}

	/**
	 * Builds a iiwa_msgs.JointDamping message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.
	 * @param currentJointDamping : the JointDamping message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param motion : the current robot motion, used to get the damping values.
	 */
	public void getCurrentJointDamping(iiwa_msgs.JointDamping currentJointDamping, LBR robot, SmartServo motion) {
		if (motion == null) 
			return;

		double[] damping = new double[robot.getJointCount()];

		try {
			damping = ((JointImpedanceControlMode) motion.getMode()).getDamping();
		} catch (Exception e) {
			System.out.println("ERROR: asking for joint damping while not in joint impedance mode!");
			return;
		}

		currentJointDamping.getHeader().setStamp(time.getCurrentTime());
		vectorToJointQuantity(damping, currentJointDamping.getDamping());
	}

	/**
	 * Builds a iiwa_msgs.JointTorque message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.<br>
	 * @param currentJointTorque : the JointTorque message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 */
	public void getCurrentJointTorque(iiwa_msgs.JointTorque currentJointTorque, LBR robot) {
		double[] torque = robot.getMeasuredTorque().getTorqueValues();
		currentJointTorque.getHeader().setStamp(time.getCurrentTime());
		vectorToJointQuantity(torque, currentJointTorque.getTorque());		
	}

	/**
	 * Builds a sensor_msgs.JointState message given a LBR iiwa Robot.<p>
	 * <b>No velocity information currently available</b>
	 * @param currentJointState : the JointState message that will be created.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 */
	public void getCurrentJointState(sensor_msgs.JointState currentJointState, LBR robot) {

		currentJointState.getHeader().setStamp(time.getCurrentTime());
		currentJointState.setName(Arrays.asList(joint_names));
		currentJointState.setPosition(robot.getCurrentJointPosition().getInternalArray());
		currentJointState.setEffort(robot.getMeasuredTorque().getTorqueValues());
	}

	// Conversions

	/**
	 * Generates a iiwa_msgs.JointQuantity message from a double vector
	 * @param vec : double vector
	 * @param q : resulting JointQuantity message
	 */
	public void vectorToJointQuantity(double[] vec, iiwa_msgs.JointQuantity q) {
		q.setA1((float) vec[0]);
		q.setA2((float) vec[1]);
		q.setA3((float) vec[2]);
		q.setA4((float) vec[3]);
		q.setA5((float) vec[4]);
		q.setA6((float) vec[5]);
		q.setA7((float) vec[6]);
	}

	/**
	 * Generate a double vector from a iiwa_msgs.JointQuantity message
	 * @param q : a JointQuantity message
	 * @return a double vector
	 */
	public double[]  jointQuantityToVector(iiwa_msgs.JointQuantity q) {
		double[] ret = new double[7];

		ret[0] = q.getA1();
		ret[1] = q.getA2();
		ret[2] = q.getA3();
		ret[3] = q.getA4();
		ret[4] = q.getA5();
		ret[5] = q.getA6();
		ret[6] = q.getA7();

		return ret;
	}

	/**
	 * Generates a MatrixRotation from a Quaternion
	 * @param x
	 * @param y
	 * @param z
	 * @param w
	 * @return a MatrixRotation
	 */
	public static MatrixRotation quatToMatrix(double x, double y, double z, double w) {
		return quatToMatrix((float) x, (float) y, (float) z, (float) w);
	}

	/**
	 * Generates a MatrixRotation from a Quaternion
	 * @param x
	 * @param y
	 * @param z
	 * @param w
	 * @return a MatrixRotation
	 */
	public final static MatrixRotation quatToMatrix(float x, float y, float z, float w) {
		double sqw = w*w;
		double sqx = x*x;
		double sqy = y*y;
		double sqz = z*z;

		MatrixBuilder mb = new MatrixBuilder();

		// invs (inverse square length) is only required if quaternion is not already normalised
		double invs = 1 / (sqx + sqy + sqz + sqw);
		mb.setElement00(( sqx - sqy - sqz + sqw)*invs) ; // since sqw + sqx + sqy + sqz =1/invs*invs
		mb.setElement11((-sqx + sqy - sqz + sqw)*invs);
		mb.setElement22((-sqx - sqy + sqz + sqw)*invs);

		double tmp1 = x*y;
		double tmp2 = z*w;
		mb.setElement10(2.0 * (tmp1 + tmp2)*invs);
		mb.setElement01(2.0 * (tmp1 - tmp2)*invs);

		tmp1 = x*z;
		tmp2 = y*w;
		mb.setElement20(2.0 * (tmp1 - tmp2)*invs);
		mb.setElement02(2.0 * (tmp1 + tmp2)*invs);

		tmp1 = y*z;
		tmp2 = x*w;
		mb.setElement21(2.0 * (tmp1 + tmp2)*invs);
		mb.setElement12(2.0 * (tmp1 - tmp2)*invs);

		return MatrixRotation.of(mb.toMatrix());
	}

	/**
	 * Generates a quaternion from a Matrix.<p>
	 * Mercilessly copied <url>from https://github.com/libgdx/libgdx/blob/master/gdx/src/com/badlogic/gdx/math/Quaternion.java</url>
	 * @param matrix : the starting matrix
	 * @param quaternion : the resulting quaternion
	 */
	public void matrixToQuat(Matrix matrix, Quaternion quaternion) {

		double xx = matrix.getElement00();
		double xy = matrix.getElement01();
		double xz = matrix.getElement02();
		double yx = matrix.getElement10();
		double yy = matrix.getElement11();
		double yz = matrix.getElement12();
		double zx = matrix.getElement20();
		double zy = matrix.getElement21();
		double zz = matrix.getElement22();

		double x,y,z,w; // return

		final double t = xx + yy + zz;

		// we protect the division by s by ensuring that s>=1
		if (t >= 0) { // |w| >= .5
			float s = (float)Math.sqrt(t + 1); // |s|>=1 ...
			w = 0.5f * s;
			s = 0.5f / s; // so this division isn't bad
			x = (zy - yz) * s;
			y = (xz - zx) * s;
			z = (yx - xy) * s;
		} else if ((xx > yy) && (xx > zz)) {
			float s = (float)Math.sqrt(1.0 + xx - yy - zz); // |s|>=1
			x = s * 0.5f; // |x| >= .5
			s = 0.5f / s;
			y = (yx + xy) * s;
			z = (xz + zx) * s;
			w = (zy - yz) * s;
		} else if (yy > zz) {
			float s = (float)Math.sqrt(1.0 + yy - xx - zz); // |s|>=1
			y = s * 0.5f; // |y| >= .5
			s = 0.5f / s;
			x = (yx + xy) * s;
			z = (zy + yz) * s;
			w = (xz - zx) * s;
		} else {
			float s = (float)Math.sqrt(1.0 + zz - xx - yy); // |s|>=1
			z = s * 0.5f; // |z| >= .5
			s = 0.5f / s;
			x = (xz + zx) * s;
			y = (zy + yz) * s;
			w = (yx - xy) * s;
		}

		quaternion.setX(x);
		quaternion.setY(y);
		quaternion.setZ(z);
		quaternion.setW(w);
	}

	/**
	 * Converts a geometry_msgs.Pose message to a Transformation object in KUKA APIs
	 * @param rosPose : starting Pose
	 * @return resulting Transformation
	 */
	public Transformation rosPoseToKukaTransformation(geometry_msgs.Pose rosPose) {
		if (rosPose == null)
			return null;

		float tx = (float) rosPose.getPosition().getX()*1000;
		float ty = (float) rosPose.getPosition().getY()*1000;
		float tz = (float) rosPose.getPosition().getZ()*1000;

		float x = (float) rosPose.getOrientation().getX();
		float y = (float) rosPose.getOrientation().getY();
		float z = (float) rosPose.getOrientation().getZ();
		float w = (float) rosPose.getOrientation().getW();

		MatrixRotation rot = iiwaMessageGenerator.quatToMatrix(x, y, z, w);
		Vector transl = Vector.of(tx, ty, tz);

		return Transformation.of(transl, rot);
	}

	/**
	 * Converts a geometry_msgs.Pose message to a Frame object in KUKA APIs
	 * @param rosPose : starting Pose
	 * @return resulting Frame
	 */
	public Frame rosPoseToKukaFrame(geometry_msgs.Pose rosPose) {	
		return new Frame(rosPoseToKukaTransformation(rosPose));
	}

	/**
	 * Converts a Transformation object from KUKA APIs to a geometry_msgs.Pose message
	 * @param kukaTransf : starting Trasnformation
	 * @param pose : resulting Pose
	 */
	public void kukaTransformationToRosPose(Transformation kukaTransf, Pose pose) {
		pose.getPosition().setX(kukaTransf.getX()/1000); 
		pose.getPosition().setY(kukaTransf.getY()/1000);
		pose.getPosition().setZ(kukaTransf.getZ()/1000);

		Matrix rotationMatrix = kukaTransf.getRotationMatrix();
		matrixToQuat(rotationMatrix, pose.getOrientation());
	}

	/**
	 * Converts an iiwa_msgs.JointQuantity to a JointPosition in KUKA APIs
	 * @param rosJointPos : starting JointQuantity
	 * @param kukaJointPos : resulting JointPosition
	 */
	public void rosJointQuantityToKuka(iiwa_msgs.JointQuantity rosJointPos, JointPosition kukaJointPos) {
		rosJointQuantityToKuka(rosJointPos, kukaJointPos, 1.0);
	}

	/**
	 * Converts an iiwa_msgs.JointQuantity to a JointPosition in KUKA APIs
	 * @param rosJointPos : starting JointQuantity
	 * @param kukaJointPos : resulting JointPosition
	 * @param scaleFactor : each element of rosJointPos with be scaled using this value 
	 */
	public void rosJointQuantityToKuka(iiwa_msgs.JointQuantity rosJointPos, JointPosition kukaJointPos, double scaleFactor) {
		kukaJointPos.set(
				rosJointPos.getA1()*scaleFactor,
				rosJointPos.getA2()*scaleFactor,
				rosJointPos.getA3()*scaleFactor,
				rosJointPos.getA4()*scaleFactor,
				rosJointPos.getA5()*scaleFactor,
				rosJointPos.getA6()*scaleFactor,
				rosJointPos.getA7()*scaleFactor
				);
	}

	/**
	 * Converts an iiwa_msgs.JointQuantity to a double vector.
	 * @param rosJointQuant : starting jointQuantity
	 * @return a double vector
	 */
	public double[] rosJointQuantityToArray(iiwa_msgs.JointQuantity rosJointQuant) {
		double[] ret = {
				rosJointQuant.getA1(),
				rosJointQuant.getA2(),
				rosJointQuant.getA3(),
				rosJointQuant.getA4(),
				rosJointQuant.getA5(),
				rosJointQuant.getA6(),
				rosJointQuant.getA7()
		};
		return ret;

	}

	/**
	 * Create a ROS Message of the given type.
	 * @param typeString : type of the ROS message to build, e.g. PoseStamped._TYPE
	 * @return created ROS message
	 */
	public <T extends org.ros.internal.message.Message> T buildMessage(String typeString) {
		return messageFactory.newFromType(typeString);
	}

	/**
	 * Adds one to the current sequence number of the message Header
	 * @param h : message Header
	 */
	public void incrementSeqNumber(std_msgs.Header h) {
		h.setSeq(h.getSeq()+1);
	}
	
	/**
	 * Checks if all the component of a CartesianQuantity are greater than the given value.
	 * @param quantity
	 * @param value
	 * @return
	 */
	public boolean isCartesianQuantityGreaterThan(iiwa_msgs.CartesianQuantity quantity, int value) {
		return (quantity.getX() > value && 
				quantity.getY() > value &&
				quantity.getZ() > value &&
				quantity.getA() > value &&
				quantity.getB() > value &&
				quantity.getC() > value);
	}
	
	/**
	 * Checks if all the component of a CartesianQuantity are greater or equal than the given value.
	 * @param quantity
	 * @param value
	 * @return
	 */
	public boolean isCartesianQuantityGreaterEqualThan(iiwa_msgs.CartesianQuantity quantity, int value) {
		return (quantity.getX() >= value && 
				quantity.getY() >= value &&
				quantity.getZ() >= value &&
				quantity.getA() >= value &&
				quantity.getB() >= value &&
				quantity.getC() >= value);
	}

	/**
	 * Checks if all the component of a JointQuantity are greater than the given value.
	 * @param value
	 * @return
	 */
	public boolean isJointQuantityGreaterThan(iiwa_msgs.JointQuantity quantity, int value) {
		return (quantity.getA1() > value && 
				quantity.getA2() > value &&
				quantity.getA3() > value &&
				quantity.getA4() > value &&
				quantity.getA5() > value &&
				quantity.getA6() > value &&
				quantity.getA7() > value);
	}
	
	/**
	 * 
	 * Checks if all the component of a JointQuantity are greater or equal than the given value.
	 * @param value
	 * @return
	 */
	public boolean isJointQuantityGreaterEqualThan(iiwa_msgs.JointQuantity quantity, int value) {
		return (quantity.getA1() >= value && 
				quantity.getA2() >= value &&
				quantity.getA3() >= value &&
				quantity.getA4() >= value &&
				quantity.getA5() >= value &&
				quantity.getA6() >= value &&
				quantity.getA7() >= value);
	}

}
