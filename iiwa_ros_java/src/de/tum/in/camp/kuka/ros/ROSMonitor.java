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

//ROS imports
import java.net.URI;

import org.ros.node.NodeMainExecutor;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

/*
 * This example shows how to monitor the state of the robot, publishing it into ROS nodes.
 * Only the Joint Position of the robot is published in this example,
 * but any other of its property included in the iiwa_msgs ROS package can be published in the same way.
 */
public class ROSMonitor extends ROSBaseApplication {

	// gravity compensation stuff
	private IUserKeyBar gravcompKeybar;
	private IUserKey gravCompKey;
	private IUserKeyListener gravCompKeyList;
	private boolean gravCompEnabled = false;
	private boolean gravCompSwitched = false;
	
	protected JointImpedanceControlMode controlMode; 
	
	@Override
	protected void addNodesToExecutor(NodeMainExecutor nodeMainExecutor) { }
	
	@Override
	protected void configureNodes(URI uri) { }

	@Override
	protected void initializeApp() {
		// Gravity compensation - only in ROSMonitor for safety
		gravcompKeybar = getApplicationUI().createUserKeyBar("Gravcomp");
		gravCompKeyList = new IUserKeyListener() {
			@Override
			public void onKeyEvent(IUserKey key,
					com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent event) {
				if (event == UserKeyEvent.FirstKeyDown) {
					gravCompEnabled = true;
					gravCompSwitched = true;
				} else if (event == UserKeyEvent.SecondKeyDown) {
					gravCompEnabled = false;
					gravCompSwitched = true;
				}
			};
		};
		gravCompKey = gravcompKeybar.addDoubleUserKey(0, gravCompKeyList, true);
		gravCompKey.setText(UserKeyAlignment.TopMiddle, "ON");
		gravCompKey.setText(UserKeyAlignment.BottomMiddle, "OFF");
		gravcompKeybar.publish();
	}

	@Override
	protected void beforeControlLoop() {
		SmartServo oldmotion = motion;
		motion = new SmartServo(robot.getCurrentJointPosition());
		motion.setMinimumTrajectoryExecutionTime(20e-3);
		motion.setJointVelocityRel(configuration.getDefaultRelativeJointSpeed());
		motion.setTimeoutAfterGoalReach(300);
		controlMode = new JointImpedanceControlMode(robot.getJointCount());
		toolFrame.moveAsync(motion.setMode(controlMode));
		oldmotion.getRuntime().stopMotion();

	}

	@Override
	protected void controlLoop() {
		if (gravCompEnabled) {
			if (gravCompSwitched) {
				gravCompSwitched = false;
				getLogger().warn("Enabling gravity compensation");
				controlMode.setStiffnessForAllJoints(0);
				controlMode.setDampingForAllJoints(0.7);
				motion.getRuntime().changeControlModeSettings(controlMode);
			}

			/*
			 * Continuously updating the commanded position to the current robot
			 * position If this is not done, when setting the stiffness back to
			 * a high value the robot will go back to that position at full
			 * speed: do not try that at home!!
			 */
			motion.getRuntime().setDestination(robot.getCurrentJointPosition());
		} else {
			if (gravCompSwitched) {
				gravCompSwitched = false;
				getLogger().warn("Disabling gravity compensation");
				controlMode.setStiffnessForAllJoints(1500);
				motion.getRuntime().changeControlModeSettings(controlMode);
				motion.getRuntime().setDestination(robot.getCurrentJointPosition());
			}
		}
	}



}
