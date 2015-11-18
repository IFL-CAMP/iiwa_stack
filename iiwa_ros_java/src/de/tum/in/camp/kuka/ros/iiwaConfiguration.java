package de.tum.in.camp.kuka.ros;

import java.util.concurrent.Semaphore;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;

public class iiwaConfiguration extends AbstractNodeMain {
	
	// Name to use to build the name of the ROS topics
	private String iiwaName = "iiwa";
	private ConnectedNode node;
	private ParameterTree params;
	private String toolName = "";
	
	private Semaphore initSemaphore = new Semaphore(0);
	
	public iiwaConfiguration(String robotName) {
		iiwaName = robotName;
	}
	
	/**
	 * Set the name to use to compose the ROS topics' names for the publishers. <p>
	 * e.g. setIIWAName("dummy"), the topics' names will be "dummy/state/...". <br>
	 * The creation of the nodes is performed when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.
	 * @param newName : the new name to use for ROS topics.
	 */
	public void setIIWAName(String newName) {
		iiwaName = newName;
	}
	
	/**
	 * Returns the current name used to compose the ROS topics' names for the publishers. <p>
	 * e.g. returning "dummy" means that the topics' names will be "dummy/state/...". <br>
	 * The creation of the nodes is performed when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.
	 * @return the current name to use for ROS topics.
	 */
	public String getIIWAName() {
		return iiwaName;
	}

	/**
	 * @see org.ros.node.NodeMain#getDefaultNodeName()
	 */
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(iiwaName + "/configuration");
	}
	
	/**
	 * This method is called when the <i>execute</i> method from a <i>nodeMainExecutor</i> is called.<br>
	 * Do <b>NOT</b> manually call this. <p> 
	 * @see org.ros.node.AbstractNodeMain#onStart(org.ros.node.ConnectedNode)
	 */
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		node = connectedNode;
		initSemaphore.release();
	}
	
	public void waitForInitialization() throws InterruptedException {
		initSemaphore.acquire();
	}
	
	private ParameterTree getParameterTree() {
		if (initSemaphore.availablePermits() > 0)
			System.out.println("waitForInitialization not called before using parameters!");
		return node.getParameterTree();
	}
	
	public String getToolName() {
		params = getParameterTree();
		toolName = params.getString(iiwaName + "/toolName", "");
		return toolName;
	}

}
