/** Copyright (C) 2015 Marco Esposito - marco.esposito@tum.de
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
 * @author Marco Esposito
 * 
 */

package de.tum.in.camp.kuka.ros;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Semaphore;

import org.ros.exception.ParameterNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;

public class iiwaConfiguration extends AbstractNodeMain {
	
	// Name to use to build the name of the ROS topics
	private String iiwaName = "iiwa";
	private ConnectedNode node;
	private ParameterTree params;
	
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
		return getStringParameter("toolName");
	}
	
	public class ToolbarSpecification {
		String name;
		String button1id;
		String button2id;
	}
	
	// one of the dirtiest things I did in my life. but I can't see a better way
	public List<ToolbarSpecification> getToolbarSpecifications() {
		List<ToolbarSpecification> ret = new ArrayList<ToolbarSpecification>();
		List<?> param = getListParameter("toolbarSpecifications");
		
		while ((param.get(0)) == "spec") {
			ToolbarSpecification ts = new ToolbarSpecification();
			ts.name = (String) param.get(1);
			ts.button1id = (String) param.get(2);
			ts.button2id = (String) param.get(3);
			
			param.remove(0);
			param.remove(1);
			param.remove(2);
			param.remove(3);
		}
		
		return ret;
	}
	
	public String getStringParameter(String argname) {
		params = getParameterTree();
		String ret = null;
		try {
			ret = params.getString(iiwaName + "/" + argname, "");			
		} catch (ParameterNotFoundException e) {
			// TODO
		}
		
		return ret;
	}
	
	public List<?> getListParameter(String argname) {
		List<?> args = new ArrayList<String>();
		params = getParameterTree();
		try {
			args = params.getList(iiwaName + "/" + argname);
			if (args == null || args.size() == 0) {
				return null;
			}
		} catch (ParameterNotFoundException e) {
			return null;
		}
		return args;
	}

}
