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

package de.tum.in.camp.kuka.ros;

// ROS imports
import geometry_msgs.PoseStamped;
import iiwa_msgs.CartesianQuantity;
import iiwa_msgs.ConfigureSmartServoRequest;
import iiwa_msgs.ConfigureSmartServoResponse;
import iiwa_msgs.JointQuantity;
import iiwa_msgs.SmartServoMode;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.net.URI;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ros.exception.ServiceException;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceResponseBuilder;

import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/*
 * This application allows to command the robot using SmartServo motions.
 */
public class ROSSmartServo extends ROSBaseApplication {

	private Lock configureSmartServoLock = new ReentrantLock();

	private iiwaMessageGenerator helper; //< Helper class to generate iiwa_msgs from current robot state.
	private iiwaSubscriber subscriber; //< IIWARos Subscriber.

	// Configuration of the subscriber ROS node.
	private NodeConfiguration nodeConfSubscriber;

	private JointPosition jp;
	private JointPosition jv;

	@Override
	protected void configureNodes(URI uri) {
		// Configuration for the Subscriber.
		nodeConfSubscriber = NodeConfiguration.newPublic(iiwaConfiguration.getRobotIp());
		nodeConfSubscriber.setTimeProvider(iiwaConfiguration.getTimeProvider());
		nodeConfSubscriber.setNodeName(iiwaConfiguration.getRobotName() + "/iiwa_subscriber");
		nodeConfSubscriber.setMasterUri(uri);
	}

	@Override
	protected void addNodesToExecutor(NodeMainExecutor nodeMainExecutor) {
		subscriber = new iiwaSubscriber(robot, iiwaConfiguration.getRobotName());

		// Configure the callback for the SmartServo service inside the subscriber class.
		subscriber.setConfigureSmartServoCallback(new ServiceResponseBuilder<iiwa_msgs.ConfigureSmartServoRequest, 
				iiwa_msgs.ConfigureSmartServoResponse>() {
			@Override
			public void build(ConfigureSmartServoRequest req,
					ConfigureSmartServoResponse resp) throws ServiceException {
				// we can change the parameters if it is the same type of control strategy
				// otherwise we have to stop the motion, replace it and start it again
				try {
					if (req.getMode().getMode() == -1
							|| (motion.getMode() != null	&& isSameControlMode(motion.getMode(), req.getMode()))) {
						if (req.getMode().getMode() != -1)
							motion.getRuntime().changeControlModeSettings(buildMotionControlMode(req.getMode()));
						if (req.getMode().getRelativeVelocity() > 0)
							motion.setJointVelocityRel(req.getMode().getRelativeVelocity());
					} else {
						configureSmartServoLock.lock();

						SmartServo oldmotion = motion;
						if (tool != null)
							ServoMotion.validateForImpedanceMode(tool);
						else
							ServoMotion.validateForImpedanceMode(robot);
						motion = configureSmartServoMotion(req.getMode());
						toolFrame.moveAsync(motion);
						oldmotion.getRuntime().stopMotion();

						configureSmartServoLock.unlock();
					}
				} catch (Exception e) {
					resp.setSuccess(false);
					if (e.getMessage() != null) {
						StringWriter sw = new StringWriter();
						PrintWriter pw = new PrintWriter(sw);
						e.printStackTrace(pw);
						resp.setError(e.getClass().getName() + ": " + e.getMessage() + ", " + sw.toString());
					} else {
						resp.setError("because I hate you :)");
					}
					return;
				}
				resp.setSuccess(true);

				getLogger().info("Changed SmartServo configuration!");
				getLogger().info("Mode: " + motion.getMode().toString());
			}
		});

		// Execute the subscriber node.
		nodeMainExecutor.execute(subscriber, nodeConfSubscriber);
	}

	@Override
	protected void initializeApp() {
		helper = new iiwaMessageGenerator(iiwaConfiguration.getRobotName());
		jp = new JointPosition(robot.getJointCount());
		jv = new JointPosition(robot.getJointCount());
	}

	public static class UnsupportedControlModeException extends RuntimeException {
		private static final long serialVersionUID = 1L;
		public UnsupportedControlModeException() { super(); }
		public UnsupportedControlModeException(String message) { super(message); }
		public UnsupportedControlModeException(String message, Throwable cause) { super(message, cause); }
		public UnsupportedControlModeException(Throwable cause) { super(cause); }
	}

	/**
	 * Given the parameters from the SmartServo service, it builds up the new control mode to use.
	 * @param params : parameters from the SmartServo service
	 * @return resulting control mode
	 */
	public IMotionControlMode buildMotionControlMode(iiwa_msgs.SmartServoMode params) {
		IMotionControlMode cm = null;

		switch (params.getMode()) {
		case iiwa_msgs.SmartServoMode.CARTESIAN_IMPEDANCE: {
			CartesianImpedanceControlMode ccm = new CartesianImpedanceControlMode();

			CartesianQuantity stiffness = params.getCartesianStiffness().getStiffness();
			if (stiffness.getX() >= 0)
				ccm.parametrize(CartDOF.X).setStiffness(stiffness.getX());
			if (stiffness.getY() >= 0)
				ccm.parametrize(CartDOF.Y).setStiffness(stiffness.getY());
			if (stiffness.getZ() >= 0)
				ccm.parametrize(CartDOF.Z).setStiffness(stiffness.getZ());
			if (stiffness.getA() >= 0)
				ccm.parametrize(CartDOF.A).setStiffness(stiffness.getA());
			if (stiffness.getB() >= 0)
				ccm.parametrize(CartDOF.B).setStiffness(stiffness.getB());
			if (stiffness.getC() >= 0)
				ccm.parametrize(CartDOF.C).setStiffness(stiffness.getC());

			CartesianQuantity damping = params.getCartesianDamping().getDamping();
			if (damping.getX() > 0)
				ccm.parametrize(CartDOF.X).setDamping(damping.getX());
			if (damping.getY() > 0)
				ccm.parametrize(CartDOF.Y).setDamping(damping.getY());
			if (damping.getZ() > 0)
				ccm.parametrize(CartDOF.Z).setDamping(damping.getZ());
			if (damping.getA() > 0)
				ccm.parametrize(CartDOF.A).setDamping(damping.getA());
			if (damping.getB() > 0)
				ccm.parametrize(CartDOF.B).setDamping(damping.getB());
			if (damping.getC() > 0)
				ccm.parametrize(CartDOF.C).setDamping(damping.getC());

			// TODO: add stiffness along axis
			if (params.getNullspaceStiffness() >= 0)
				ccm.setNullSpaceStiffness(params.getNullspaceStiffness());
			if (params.getNullspaceDamping() > 0)
				ccm.setNullSpaceDamping(params.getNullspaceDamping());

			cm = ccm;
			break;
		}

		case iiwa_msgs.SmartServoMode.JOINT_IMPEDANCE: {
			JointImpedanceControlMode jcm = new JointImpedanceControlMode(7);

			JointQuantity stiffness = params.getJointStiffness().getStiffness();
			jcm.setStiffness(helper.jointQuantityToVector(stiffness));

			JointQuantity damping = params.getJointDamping().getDamping();
			if (damping.getA1() > 0 && damping.getA2() > 0 && damping.getA3() > 0 && damping.getA4() > 0
					&& damping.getA5() > 0 && damping.getA6() > 0 && damping.getA7() > 0)
				jcm.setDamping(helper.jointQuantityToVector(damping));

			cm = jcm;
			break;
		}

		case iiwa_msgs.SmartServoMode.CONSTANT_FORCE : {
			CartesianSineImpedanceControlMode cscm = new CartesianSineImpedanceControlMode();
			CartDOF direction = null;
			switch (params.getConstantForceDirection()) {
			case iiwa_msgs.SmartServoMode.X :
				direction = CartDOF.X;
				break;
			case iiwa_msgs.SmartServoMode.Y :
				direction = CartDOF.Y;
				break;
			case iiwa_msgs.SmartServoMode.Z :
				direction = CartDOF.Z;
				break;
			default:
				getLogger().error("Wrong direction given, use [1,2,3] for directions [X,Y,Z] respectively.");
				break;
			}

			if (direction != null) {
				cscm = CartesianSineImpedanceControlMode.createDesiredForce(direction, params.getConstantForce(), params.getConstantForceStiffness());
				cm = cscm;
			}
			break;
		}

		default:				
			getLogger().error("Control Mode not supported.");
			throw new UnsupportedControlModeException();  // this should just not happen
		}

		if (cm != null)
			return cm;
		else
			throw new UnsupportedControlModeException();
	}

	public SmartServo configureSmartServoMotion(iiwa_msgs.SmartServoMode ssm) {
		SmartServo mot = new SmartServo(robot.getCurrentJointPosition());
		mot.setMinimumTrajectoryExecutionTime(20e-3); //TODO : parametrize
		mot.setTimeoutAfterGoalReach(300); //TODO : parametrize
		motion.getRuntime().activateVelocityPlanning(true);

		configureSmartServoMotion(ssm, mot);
		return mot;
	}

	public void configureSmartServoMotion(iiwa_msgs.SmartServoMode ssm, SmartServo mot) {
		if (mot == null)
			return; // TODO: exception?

		if (ssm.getRelativeVelocity() > 0)
			mot.setJointVelocityRel(ssm.getRelativeVelocity());
		mot.setMode(buildMotionControlMode(ssm));
	}

	/**
	 * Checks if a SmartServoMode is of the same type as a MotionControlMode from KUKA APIs
	 * @return boolean
	 */
	public boolean isSameControlMode(IMotionControlMode kukacm, SmartServoMode roscm) {		
		String roscmname = null;
		switch (roscm.getMode()) {
		case SmartServoMode.CARTESIAN_IMPEDANCE:
			roscmname = "CartesianImpedanceControlMode";
			break;
		case SmartServoMode.JOINT_IMPEDANCE:
			roscmname = "JointImpedanceControlMode";
			break;
		case SmartServoMode.CONSTANT_FORCE:
			roscmname = "CartesianSineImpedanceControlMode";
			break;
		}
		String kukacmname = kukacm.getClass().getSimpleName();

		return roscmname.equals(kukacmname);
	}

	@Override
	protected void beforeControlLoop() { 
		motion.getRuntime().activateVelocityPlanning(true);  // TODO: do this whenever appropriate
	}

	@Override
	protected void controlLoop() {
		if (subscriber.currentCommandType != null) {
			configureSmartServoLock.lock(); // the service could stop the motion and restart it

			switch (subscriber.currentCommandType) {
			case CARTESIAN_POSE: {
				PoseStamped commandPosition = subscriber.getCartesianPose(); // TODO: check that frame_id is consistent
				Transformation tr = helper.rosPoseToKukaTransformation(commandPosition.getPose());
				Frame destinationFrame = new Frame(tr);

				if (robot.isReadyToMove()) 
					//motion.getRuntime().setDestination(tr);
					motion.getRuntime().setDestination(destinationFrame);
			}
			break;
			case JOINT_POSITION: {
				/*
				 * This will acquire the last received JointPosition command from the commanding ROS node.
				 * If the robot can move, then it will move to this new position.
				 */
				iiwa_msgs.JointPosition commandPosition = subscriber.getJointPosition();
				helper.rosJointQuantityToKuka(commandPosition.getPosition(), jp);

				if (robot.isReadyToMove()) 
					motion.getRuntime().setDestination(jp);
			}
			break;
			case JOINT_POSITION_VELOCITY: { // TODO : do we want to keep this?
				/*
				 * This will acquire the last received JointPositionVelocity command from the commanding ROS node.
				 * If the robot can move, then it will move to this new position.
				 */
				iiwa_msgs.JointPositionVelocity commandPositionVelocity = subscriber.getJointPositionVelocity();
				helper.rosJointQuantityToKuka(commandPositionVelocity.getPosition(), jp);
				helper.rosJointQuantityToKuka(commandPositionVelocity.getVelocity(), jv);

				if (robot.isReadyToMove()) 
					motion.getRuntime().setDestination(jp, jv);
			}
			break;

			default:
				throw new UnsupportedControlModeException();
			}

			configureSmartServoLock.unlock();
		}

	}
}