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

import java.util.Collection;
import java.util.Observable;

import org.jgrapht.event.GraphListener;
import org.ros.message.MessageListener;
//import org.ros.node.Node;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.ros.rosjava.tf.StampedTransform;
import org.ros.rosjava.tf.TransformBuffer;
import org.ros.rosjava.tf.TransformFactory;
import org.ros.rosjava.tf.TransformTree;
import tf2_msgs.TFMessage;

import com.google.common.base.Preconditions;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 */
public class TransformListener extends Observable {
	
	protected final ConnectedNode node;
	protected final TransformTree tfTree;

	private Subscriber<TFMessage> tfSubscriber;
	private Subscriber<TFMessage> staticTfSubscriber;

	public TransformListener(ConnectedNode node) {
		this.node = node;
		this.tfTree = new TransformTree();
		subscribe();
	}
	
	public void subscribe() {
		Preconditions.checkNotNull(node);

	    tfSubscriber = node.newSubscriber("/tf", TFMessage._TYPE);
	    tfSubscriber.addMessageListener(new MessageListener<TFMessage>() {
	      @Override
	      public void onNewMessage(final TFMessage message) {
	    	  Collection<StampedTransform> transforms = TransformFactory.fromTfMessage(message);
	    	  tfTree.add(transforms);
	    	  //setChanged(); // observable stuff
	    	  //notifyObservers(); // observable stuff
	      }
	    });

		staticTfSubscriber = node.newSubscriber("/tf_static", TFMessage._TYPE);
		staticTfSubscriber.addMessageListener(new MessageListener<TFMessage>() {
			@Override
			public void onNewMessage(final TFMessage message) {
				Collection<StampedTransform> transforms = TransformFactory.fromTfMessage(message);
				tfTree.add(transforms);
				//setChanged(); // observable stuff
				//notifyObservers(); // observable stuff
			}
		});
	}

	public void addListener(GraphListener<String,TransformBuffer> listener) {
		tfTree.getGraph().addGraphListener(listener);
	}
	
	public TransformTree getTree() {
		return tfTree;
	}
		
}

