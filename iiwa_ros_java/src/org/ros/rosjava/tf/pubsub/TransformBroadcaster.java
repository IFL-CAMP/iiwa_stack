/* 
 * Copyright 2011 Heuristic Labs, LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *   
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.ros.rosjava.tf.pubsub;

import java.util.ArrayList;

import com.google.common.base.Preconditions;

import geometry_msgs.TransformStamped;
import tf2_msgs.TFMessage;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.message.MessageFactory;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 * @brief This is a simple class to provide sendTransform() (akin to rospy and roscpp versions); it handles creation of publisher and advertising for you.
 */

public class TransformBroadcaster {

	protected final ConnectedNode node;
	protected Publisher<TFMessage> pub;
	protected MessageFactory messageFactory;
	
	public TransformBroadcaster(ConnectedNode node) {
		this.node = node;
		this.messageFactory = this.node.getTopicMessageFactory();
		advertise();
	}
	
	protected void advertise() {
		Preconditions.checkNotNull(node);
		this.pub = node.newPublisher("/tf", TFMessage._TYPE);
		this.pub.setLatchMode(true);
		node.getLog().debug("TransformBroadcaster advertised on /tf.");
	}

	public void sendTransform(
									String parentFrame, String childFrame,
									long t, // in nanoseconds
									double v_x, double v_y, double v_z,
									double q_x, double q_y, double q_z, double q_w // quaternion
	) {
		
		// WARN if quaternion not normalized, and normalize it
		// WARN if time is in the future, or otherwise looks funky (negative? more than a year old?)
		Preconditions.checkNotNull(node);

//		Vector3d v = new Vector3d(v_x, v_y, v_z);
//		Quat4d q = new Quat4d(q_w, q_x, q_y, q_z);
//		StampedTransform tx = new StampedTransform(t, parentFrame, childFrame,);
//		geometry_msgs.TransformStamped txMsg = TransformFactory.tx2msg(tx);
		
		geometry_msgs.TransformStamped txMsg = messageFactory.newFromType(geometry_msgs.TransformStamped._TYPE);

		txMsg.getHeader().setStamp(org.ros.message.Time.fromNano(t));
		txMsg.getHeader().setFrameId(parentFrame);
		txMsg.setChildFrameId(childFrame);
		// TODO: invert transform, if it is not cool (have to add tfTree here, then...)

		geometry_msgs.Vector3 translation = messageFactory.newFromType(geometry_msgs.Vector3._TYPE);
		translation.setX(v_x);
		translation.setY(v_y);
		translation.setZ(v_z);
		txMsg.getTransform().setTranslation(translation);

		geometry_msgs.Quaternion rotation = messageFactory.newFromType(geometry_msgs.Quaternion._TYPE);
		rotation.setX(q_x);
		rotation.setY(q_y);
		rotation.setZ(q_z);
		rotation.setW(q_w);
		txMsg.getTransform().setRotation(rotation);

		TFMessage msg = messageFactory.newFromType(TFMessage._TYPE);
		msg.setTransforms(new ArrayList<TransformStamped>(1));
		msg.getTransforms().add(txMsg);

		this.pub.publish(msg);
	}
}
