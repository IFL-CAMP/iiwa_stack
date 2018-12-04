package de.tum.in.camp.kuka.ros;

import javax.annotation.PostConstruct;

import org.ros.node.NodeMainExecutor;

public interface ROSTool {
	/**
	 * This method is called after construction is done.
	 */
	public void initialize(Configuration configuration, NodeMainExecutor mainExecutor);
	
	/**
	 * This method is called periodically. Implement reaction to incoming commands here.
	 */
	public void moveTool();
	
	/**
	 * This method is called periodically. Publish TF or other status data here.
	 * @throws InterruptedException 
	 */
	public void publishCurrentState() throws InterruptedException;
}
