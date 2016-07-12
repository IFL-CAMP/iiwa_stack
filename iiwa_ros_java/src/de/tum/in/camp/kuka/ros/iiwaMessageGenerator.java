 /**  
 * Copyright (C) 2016 Salvatore Virga - salvo.virga@tum.de, Marco Esposito - marco.esposito@tum.de
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

// IIWAROS import
package de.tum.in.camp.kuka.ros;

// ROS import
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;

import java.util.Arrays;

import org.ros.message.MessageFactory;
import org.ros.node.NodeConfiguration;
import org.ros.time.TimeProvider;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixBuilder;
import com.kuka.roboticsAPI.geometricModel.math.MatrixRotation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/**
 * This class helps the building of iiwa_msgs ROS messages,
 * it gives a collection of methods to build the messages from the current state of a LBR iiwa Robot.
 * For Cartesian messages, it's possible to pass a reference frames to acquire the message values.
 * If no reference frames is passed, the flange frame is used.
 */
public class iiwaMessageGenerator {
	private static final String baseFrameID = "iiwa_link_0"; // TODO: overridable for multi-robot configurations
	
	final String[] joint_names = {
			  "iiwa_joint_1", 
			  "iiwa_joint_2",
			  "iiwa_joint_3",
			  "iiwa_joint_4",
			  "iiwa_joint_5",
			  "iiwa_joint_6",
			  "iiwa_joint_7"
			};

	// Objects to create ROS messages
	private NodeConfiguration nodeConf = NodeConfiguration.newPrivate();
	private MessageFactory messageFactory = nodeConf.getTopicMessageFactory();
	private TimeProvider time = iiwaConfiguration.getTimeProvider();
	
	public iiwaMessageGenerator() {}

	/**
	 * Builds a CartesianPosition message given a LBR iiwa Robot.<p>
	 * The Cartesian position will be the obtained from current Flange frame,
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built CartesianPosition message.
	 */
	public void getCurrentCartesianPose(geometry_msgs.PoseStamped pps, LBR robot) {
		getCurrentCartesianPose(pps, robot, robot.getFlange());
	}

	/**
	 * Builds a CartesianPosition message given a LBR iiwa Robot and a frame of reference.<p>
	 * The Cartesian position will be the obtained from the given Frame,
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param frame : reference frame to set the values of the Cartesian position.
	 * @return built CartesianPosition message.
	 */
	public void getCurrentCartesianPose(geometry_msgs.PoseStamped pps, LBR robot, ObjectFrame frame) {
		Transformation transform = robot.getCurrentCartesianPosition(frame).transformationFromWorld();

		pps.getHeader().setFrameId(baseFrameID);  
		pps.getHeader().setStamp(time.getCurrentTime());

		kukaTransformationToRosPose(transform, pps.getPose());
	}

	/**
	 * Builds a CartesianVelocity message given a LBR iiwa Robot.<p>
	 * The <b>velocity will be set to zero</b>, 
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built CartesianVelocity message.
	 */
	//	public iiwa_msgs.CartesianVelocity buildCartesianVelocity(LBR robot) {
	//		return buildCartesianVelocity(robot, robot.getFlange());
	//	}

	/**
	 * Builds a CartesianVelocity message given a LBR iiwa Robot and a frame of reference.<p>
	 * The <b>velocity will be set to zero</b>, 
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param frame : reference frame the velocity refers to.
	 * @return built CartesianVelocity message.
	 */
	//	public iiwa_msgs.CartesianVelocity buildCartesianVelocity(LBR robot, ObjectFrame frame) {
	////		double[] velocity = new double[9];
	//
	//		// Velocity is set to zero!
	//		// TODO: Compute Cartesian velocity !
	//
	//		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
	//		header.setFrameId(frame.getName());
	//		header.setStamp(time.getCurrentTime());
	//
	//		iiwa_msgs.CartesianVelocity cv = messageFactory.newFromType(iiwa_msgs.CartesianVelocity._TYPE);
	//		
	////		iiwa_msgs.CartesianQuantity cq = messageFactory.newFromType(iiwa_msgs.CartesianQuantity._TYPE);
	//		
	////		cv.setVelocity(velocity);
	//		cv.setHeader(header);
	//
	//		return cv;
	//	}

	/**
	 * Builds a CartesianVelocity message given a LBR iiwa Robot.<p>
	 * The wrench values will refer to the Flange frame,
	 * the message header is set to current time and according frame name.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built CartesianWrench message.
	 */
	public void getCurrentCartesianWrench(geometry_msgs.WrenchStamped ws, LBR robot) {
		getCurrentCartesianWrench(ws, robot, robot.getFlange());
	}

	/**
	 * Builds a CartesianVelocity message given a LBR iiwa Robot and a frame of reference.<p>
	 * The wrench values will refer to the given frame,
	 * the message header is set to current time and according frame name.<br> 
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @param frame : reference frame the wrench refers to.
	 * @return built CartesianWrench message.
	 */
	public void getCurrentCartesianWrench(geometry_msgs.WrenchStamped ws, LBR robot, ObjectFrame frame) {
		ws.getHeader().setFrameId(frame.getName()); // TODO : should this be baseFrameID
		ws.getHeader().setStamp(time.getCurrentTime());
		
		ws.getWrench().getForce().setX(robot.getExternalForceTorque(frame).getForce().getX());
		ws.getWrench().getForce().setY(robot.getExternalForceTorque(frame).getForce().getY());
		ws.getWrench().getForce().setZ(robot.getExternalForceTorque(frame).getForce().getZ());

		ws.getWrench().getTorque().setX(robot.getExternalForceTorque(frame).getTorque().getX());
		ws.getWrench().getTorque().setY(robot.getExternalForceTorque(frame).getTorque().getY());
		ws.getWrench().getTorque().setZ(robot.getExternalForceTorque(frame).getTorque().getZ());
	}

	/**
	 * Builds a JointPosition message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built JointPosition message.
	 */
	public void getCurrentJointPosition(iiwa_msgs.JointPosition jp, LBR robot) {
		double[] position = robot.getCurrentJointPosition().getInternalArray();

		jp.getHeader().setStamp(time.getCurrentTime());

		vectorToJointQuantity(position, jp.getPosition());
	}
	
	/**
	 * Builds a JointPositionVelocity message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built JointPositionVelocity message.
	 */
	public void getCurrentJointPositionVelocity(iiwa_msgs.JointPositionVelocity jpv, LBR robot) {
		double[] position = robot.getCurrentJointPosition().getInternalArray();
		double[] velocity = new double[7];  // TODO: read current joint velocity

		jpv.getHeader().setStamp(time.getCurrentTime());

		vectorToJointQuantity(position, jpv.getPosition());
		vectorToJointQuantity(velocity, jpv.getVelocity());
	}

	/**
	 * Builds a JointStiffness message given a LBR iiwa Robot.<p>
	 * The <b>stiffness will be set to zero</b>,<br>
	 * the message header is set to current time.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built JointStiffness message.
	 */
	public void getCurrentJointStiffness(iiwa_msgs.JointStiffness js, LBR robot, SmartServo motion) {
		if (motion == null) 
			return;

		double[] stiffness = new double[7];

		try {
			stiffness = ((JointImpedanceControlMode) motion.getMode()).getStiffness();
		} catch (Exception e) {
			// ups, our control mode is not joint impedance
			System.out.println("ERROR: asking for joint stiffness while not in impedance!");
			return;
		}

		js.getHeader().setStamp(time.getCurrentTime());

		vectorToJointQuantity(stiffness, js.getStiffness());
	}

	/**
	 * Builds a JointDamping message given a LBR iiwa Robot.<p>
	 * The <b>damping will be set to zero</b>,<br>
	 * the message header is set to current time.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built JointDamping message.
	 */
	public void getCurrentJointDamping(iiwa_msgs.JointDamping jd, LBR robot, SmartServo motion) {
		if (motion == null) 
			return;

		double[] damping = new double[7];

		try {
			damping = ((JointImpedanceControlMode) motion.getMode()).getDamping();
		} catch (Exception e) {
			// ups, our control mode is not joint impedance
			System.out.println("ERROR: asking for joint damping while not in impedance!");
			return;
		}

		jd.getHeader().setStamp(time.getCurrentTime());

		vectorToJointQuantity(damping, jd.getDamping());
	}

	/**
	 * Builds a JointTorque message given a LBR iiwa Robot.<p>
	 * The message header is set to current time.<br>
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built JointTorque message.
	 */
	public void getCurrentJointTorque(iiwa_msgs.JointTorque jt, LBR robot) {
		double[] torque = robot.getMeasuredTorque().getTorqueValues();

		jt.getHeader().setStamp(time.getCurrentTime());

		vectorToJointQuantity(torque, jt.getTorque());
	}

	/**
	 * Builds a JointVelocity message given a LBR iiwa Robot.<p>
	 * The <b>velocity will be set to zero</b>,<br>
	 * the message header is set to current time.
	 * @param robot : an iiwa Robot, its current state is used to set the values of the message.
	 * @return built JointVelocity message.
	 */
	//	public iiwa_msgs.JointVelocity buildJointVelocity(LBR robot) {
	//		double[] velocity = new double[7];
	//		// TODO: build velocity vector
	//
	//		std_msgs.Header header = messageFactory.newFromType(std_msgs.Header._TYPE);
	//		header.setFrameId("Robot");
	//		header.setStamp(time.getCurrentTime());
	//		
	//		iiwa_msgs.JointQuantity a = vectorToJointQuantity(velocity);
	//		
	//		iiwa_msgs.JointVelocity jv = messageFactory.newFromType(iiwa_msgs.JointVelocity._TYPE);
	//		jv.setHeader(header);
	//		jv.setVelocity(a);
	//		return jv;
	//	}
	
	// TODO : doc
	public void getCurrentJointState(sensor_msgs.JointState js, LBR robot) {
		
		js.getHeader().setStamp(time.getCurrentTime());
		
		js.setName(Arrays.asList(joint_names));
		
		js.setPosition(robot.getCurrentJointPosition().getInternalArray());
		
		js.setEffort(robot.getMeasuredTorque().getTorqueValues());
		
		// TODO: velocity
	}

	// Conversions

	public void vectorToJointQuantity(double[] torque, iiwa_msgs.JointQuantity q) {
		q.setA1((float) torque[0]);
		q.setA2((float) torque[1]);
		q.setA3((float) torque[2]);
		q.setA4((float) torque[3]);
		q.setA5((float) torque[4]);
		q.setA6((float) torque[5]);
		q.setA7((float) torque[6]);
	}

	public double[]  jointQuantityToVector(iiwa_msgs.JointQuantity a) {
		double[] ret = new double[7];

		ret[0] = a.getA1();
		ret[1] = a.getA2();
		ret[2] = a.getA3();
		ret[3] = a.getA4();
		ret[4] = a.getA5();
		ret[5] = a.getA6();
		ret[6] = a.getA7();

		return ret;
	}

	public static MatrixRotation quatToMatrix(double x, double y, double z, double w) {
		return quatToMatrix((float) x, (float) y, (float) z, (float) w);
	}

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

	public void matrixToQuat(Matrix matrix, Quaternion quaternion) {

		// mercilessly copied from https://github.com/libgdx/libgdx/blob/master/gdx/src/com/badlogic/gdx/math/Quaternion.java
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

	public Transformation rosPoseToKukaTransformation(Pose rosPose) {
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

	public void kukaTransformationToRosPose(Transformation kukaTransf, Pose pose) {
		pose.getPosition().setX(kukaTransf.getX()/1000); 
		pose.getPosition().setY(kukaTransf.getY()/1000);
		pose.getPosition().setZ(kukaTransf.getZ()/1000);

		Matrix rotationMatrix = kukaTransf.getRotationMatrix();
		matrixToQuat(rotationMatrix, pose.getOrientation());
	}

	// KUKA uses JointPosition for speed also...
	public void rosJointQuantityToKuka(iiwa_msgs.JointQuantity rosJointPos, JointPosition kukaJointPos) {
		kukaJointPos.set(
			rosJointPos.getA1(),
			rosJointPos.getA2(),
			rosJointPos.getA3(),
			rosJointPos.getA4(),
			rosJointPos.getA5(),
			rosJointPos.getA6(),
			rosJointPos.getA7()
		);
	}

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

	public <T extends org.ros.internal.message.Message> T buildMessage(String typeString) {
		return messageFactory.newFromType(typeString);
	}
	
	public void incrementSeqNumber(std_msgs.Header h) {
		h.setSeq(h.getSeq()+1);
	}

}
