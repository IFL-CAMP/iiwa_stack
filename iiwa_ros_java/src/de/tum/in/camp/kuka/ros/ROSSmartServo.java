/** Copyright (C) 2015 Salvatore Virga - salvo.virga@tum.de
 * Technische Universitaet Muenchen
 * Chair for Computer Aided Medical Procedures and Augmented Reality
 * Fakultaet fuer Informatik / I16, Boltzmannstrasse 3, 85748 Garching bei Muenchen, Germany
 * http://campar.in.tum.de
 * 
 * LICENSE :
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * @author Salvatore Virga
 * 
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
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

/*
 * This example shows how to monitor and change the state of the robot.
 * The current state is published into a (monitor) ROS node, while another (command) node receives
 * messages containing the a new position for the robot.
 * The example uses a SmartServo motion to move the robot to the new commanded position.
 * 
 * Only the Joint Position of the robot is published (current position)
 * and received (new position) in this example,
 * but any other of its property included in the iiwa_msgs ROS package can be published
 * and received in the same way.
 */
public class ROSSmartServo extends ROSBaseApplication {

	private Lock configureSmartServoLock = new ReentrantLock();

	private iiwaMessageGenerator helper; //< Helper class to generate iiwa_msgs from current robot state.
	private iiwaSubscriber subscriber; //< IIWARos Subscriber.

	// ROS Configuration and Node execution objects. Two different configurations are needed
	// for the Publisher and the Subscriber.
	private NodeConfiguration nodeConfSubscriber;
	
	private JointPosition jp = new JointPosition(7);
	private JointPosition jv = new JointPosition(7);
	
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
		
		// SmartServo configuration service callback
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
						ServoMotion.validateForImpedanceMode(robot);
						motion = configureSmartServoMotion(req.getMode());
						robot.moveAsync(motion);  // TODO: use toolFrame.moveAsync()
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
				getLogger().info("Mode: "+motion.toString());
			}
		});
		
		nodeMainExecutor.execute(subscriber, nodeConfSubscriber);
	}

	@Override
	protected void initializeApp() {
		helper = new iiwaMessageGenerator();

		
	}

	public static class UnsupportedControlModeException extends RuntimeException {
		private static final long serialVersionUID = 1L;
		public UnsupportedControlModeException() { super(); }
		public UnsupportedControlModeException(String message) { super(message); }
		public UnsupportedControlModeException(String message, Throwable cause) { super(message, cause); }
		public UnsupportedControlModeException(Throwable cause) { super(cause); }
	}

	public IMotionControlMode buildMotionControlMode(iiwa_msgs.SmartServoMode params) {
		IMotionControlMode cm;

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

		default: {
			throw new UnsupportedControlModeException();  // this should just not happen
		}
		}

		return cm;
	}

	public SmartServo configureSmartServoMotion(iiwa_msgs.SmartServoMode ssm) {
		SmartServo mot = new SmartServo(robot.getCurrentJointPosition());
		mot.setMinimumTrajectoryExecutionTime(20e-3);
		mot.setTimeoutAfterGoalReach(300);
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
	
	public boolean isSameControlMode(IMotionControlMode kukacm, SmartServoMode roscm) {		
		String roscmname = null;
		switch (roscm.getMode()) {
		case SmartServoMode.CARTESIAN_IMPEDANCE:
			roscmname = "CartesianImpedanceControlMode";
			break;
		case SmartServoMode.JOINT_IMPEDANCE:
			roscmname = "JointImpedanceControlMode";
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

				if (robot.isReadyToMove()) 
					motion.getRuntime().setDestination(tr);
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
			case JOINT_POSITION_VELOCITY: {
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