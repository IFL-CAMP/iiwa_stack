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
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import java.net.URI;

//KUKA imports
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;

/*
 * This example shows how to monitor the state of the robot, publishing it into ROS nodes.
 * Only the Joint Position of the robot is published in this example,
 * but any other of its property included in the iiwa_msgs ROS package can be published in the same way.
 */
public class ROSMonitor extends RoboticsAPIApplication {

	private LBR robot;
	private iiwaMessageGenerator helper; //< Helper class to generate iiwa_msgs from current robot state.
	private iiwaPublisher publisher; //< IIWARos Publisher.

	// TODO: change the following IP addresses according to your setup.
	private String masterUri = "http://160.69.69.100:11311"; //< IP address of ROS core to talk to.
	private String host = "160.69.69.69"; //< IP address of the IIWA Cabinet.

	// ROS Configuration and Node execution objects.
	private URI uri;
	private NodeConfiguration nodeConfPublisher;
	private NodeMainExecutor nodeExecutor;

	// Message to publish.
	private iiwa_msgs.JointPosition currentPosition;

	private boolean debug = false;

	public void initialize() {
		robot = getContext().getDeviceFromType(LBR.class);
		helper = new iiwaMessageGenerator();
		publisher = new iiwaPublisher(robot,"iiwa");

		try {
			// Set the configuration parameters of the ROS node to create.
			uri = new URI(masterUri);
			nodeConfPublisher = NodeConfiguration.newPublic(host);
			nodeConfPublisher.setNodeName("iiwa_publisher");
			nodeConfPublisher.setMasterUri(uri);
		}
		catch (Exception e) {
			if (debug) getLogger().info("Node Configuration failed.");
			getLogger().info(e.toString());
		}

		try {
			// Start the Publisher node with the set up configuration.
			nodeExecutor = DefaultNodeMainExecutor.newDefault();
			nodeExecutor.execute(publisher, nodeConfPublisher);
			if (debug) getLogger().info("ROS Node initialized.");
		}
		catch(Exception e) {
			if (debug) getLogger().info("Node Executor failed.");
			getLogger().info(e.toString());
		}
	}

	public void run() {

		// The run loop
		getLogger().info("Starting the ROS Monitor loop...");
		try {
			while(true) { 

				/*
				 * This will build a JointPosition message with the current robot state.
				 * Set that message to be published and then publish it if there's a subscriber listening.
				 * Any other of the set methods for iiwa_msgs included in the published can be used at the same time,
				 * one just needs to build the message and set it to the publisher.
				 */
				currentPosition = helper.buildJointPosition(robot);
				publisher.setJointPosition(currentPosition);
				//published.setJointTorque(aJointTorqueMessage);
				//published.setCartesianRotation(aCartesianRotationMessage);
				publisher.publish();
			} 
		}
		catch (InterruptedException e) {
			getLogger().info("ROS loop aborted. " + e.toString());
		} finally {
			if (nodeExecutor != null) {
				nodeExecutor.shutdownNodeMain(publisher);
				if (debug)getLogger().info("ROS Node terminated.");
			}
			getLogger().info("ROS loop has ended. Application terminated.");
		}
	}

	@Override
	public void dispose() {
		// The Publisher node is killed.
		if (nodeExecutor != null) {
			nodeExecutor.shutdownNodeMain(publisher);
			getLogger().info("ROS nodes have been terminated by Garbage Collection.");
		}
		super.dispose();
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		ROSMonitor app = new ROSMonitor();
		app.runApplication();
	}
}
