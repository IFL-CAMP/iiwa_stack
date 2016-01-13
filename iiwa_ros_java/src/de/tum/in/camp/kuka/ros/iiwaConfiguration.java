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
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Semaphore;

import org.ros.exception.ParameterNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;

import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;

public class iiwaConfiguration extends AbstractNodeMain {
	
	// Name to use to build the name of the ROS topics
	private String iiwaName = "iiwa";
	private ConnectedNode node;
	private ParameterTree params;
	
	// used to wait until we are connected to the ROS master and params are available
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
		public String name;
		public String[] buttonIDs;
	}
	
	public void setupToolbars(IApplicationUI appUI, 
			final iiwaPublisher publisher, 
			List<IUserKey> generalKeys, 
			List<IUserKeyListener> generalKeyLists, 
			List<IUserKeyBar> generalKeyBars) {
		List<ToolbarSpecification> ts = getToolbarSpecifications();
		if (ts != null) {
			for (final ToolbarSpecification t: ts) {
				IUserKeyBar generalKeyBar = appUI.createUserKeyBar(t.name);
				
				for (int i = 0; i < t.buttonIDs.length; i++) {
					final String buttonID = t.buttonIDs[i];
					IUserKey generalKey;
					if (buttonID.contains(",")) {
						// double button
						final String[] singleButtonIDs = buttonID.split(",");
						
						IUserKeyListener generalKeyList = new IUserKeyListener() {
							@Override
							public void onKeyEvent(IUserKey key, com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent event) {
								if (event == UserKeyEvent.FirstKeyDown) {
									publisher.publishButtonPressed(t.name+"_"+singleButtonIDs[0]);
								} else if (event == UserKeyEvent.FirstKeyUp) {
									publisher.publishButtonReleased(t.name+"_"+singleButtonIDs[0]);
								} else if (event == UserKeyEvent.SecondKeyDown) {
									publisher.publishButtonPressed(t.name+"_"+singleButtonIDs[1]);
								} else if (event == UserKeyEvent.SecondKeyUp) {
									publisher.publishButtonReleased(t.name+"_"+singleButtonIDs[1]);
								}
							}
						};
						generalKeyLists.add(generalKeyList);
						
						generalKey = generalKeyBar.addDoubleUserKey(i, generalKeyList, false);
						generalKey.setText(UserKeyAlignment.TopMiddle, singleButtonIDs[0]);
						generalKey.setText(UserKeyAlignment.BottomMiddle, singleButtonIDs[1]);
						generalKeys.add(generalKey);
					} else {
						// single button
						IUserKeyListener generalKeyList = new IUserKeyListener() {
							@Override
							public void onKeyEvent(IUserKey key, com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent event) {
								if (event == UserKeyEvent.KeyDown) {
									publisher.publishButtonPressed(t.name+"_"+buttonID);
								} else if (event == UserKeyEvent.KeyUp) {
									publisher.publishButtonReleased(t.name+"_"+buttonID);
								} 
							}
						};
						generalKeyLists.add(generalKeyList);
						
						generalKey = generalKeyBar.addUserKey(i, generalKeyList, false);
						generalKey.setText(UserKeyAlignment.TopMiddle, buttonID);
						generalKeys.add(generalKey);
					}
				}
				
				generalKeyBars.add(generalKeyBar);
			}	
			for (IUserKeyBar kb  : generalKeyBars)
				kb.publish();
		}
	}

	// one of the dirtiest things I did in my life. but I can't see a better way
	public List<ToolbarSpecification> getToolbarSpecifications() {
		List<ToolbarSpecification> ret = new ArrayList<ToolbarSpecification>();
		List<?> rawParam = getListParameter("toolbarSpecifications");
		
		if (rawParam == null)
			return null;
		
		@SuppressWarnings("unchecked")
		List<String> stringParam = new LinkedList<String>((Collection<? extends String>) rawParam);
		
		while (stringParam.size() > 0 && (stringParam.get(0)).equals("spec")) {
			ToolbarSpecification ts = new ToolbarSpecification();
			stringParam.remove(0);
			ts.name = (String) stringParam.get(0);
			stringParam.remove(0);
			List<String> buttons = new LinkedList<String>();
			while (stringParam.size() > 0 && !(stringParam.get(0)).equals("spec")) {
				buttons.add(stringParam.get(0));
				stringParam.remove(0);
			}
			if (buttons.size() == 0) // toolbar name but no buttons; TODO: log
				continue;
			ts.buttonIDs = buttons.toArray(new String[buttons.size()]);
			ret.add(ts);
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
		List<?> args = new LinkedList<String>();  // supports remove
		params = getParameterTree();
		try {
			args = params.getList(iiwaName + "/" + argname);
			if (args == null) {
				return null;
			}
		} catch (ParameterNotFoundException e) {
			return null;
		}
		return args;
	}

}
