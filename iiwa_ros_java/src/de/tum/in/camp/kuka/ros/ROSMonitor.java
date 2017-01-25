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
 * This application allows to turn on/off a pseudo Gravity Compensation mode via buttons on the SmartPad.
 * It extends the ROSBaseApplication, so it also publishes the current state of the robot on ROS topics.
 */
public class ROSMonitor extends ROSBaseApplication {

	// Gravity compensation keys for the SmartPad toolbar
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
		motion.setJointVelocityRel(configuration.getDefaultRelativeJointVelocity());
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
				controlMode.setStiffnessForAllJoints(1500); // TODO : max is 5000
				motion.getRuntime().changeControlModeSettings(controlMode);
				motion.getRuntime().setDestination(robot.getCurrentJointPosition());
			}
		}
	}
}
