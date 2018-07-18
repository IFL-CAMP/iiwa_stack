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

package org.ros.rosjava.tf.adt;

import java.util.Collection;
import java.util.HashMap;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.rosjava.tf.StampedTransform;
import org.ros.rosjava.tf.TransformBuffer;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 * @brief partial implementation of a TransformDatabase
 * 
 * first, locate path through transform tree: "map->odom", "odom->laser"
 * (making sure it is valid at time t)
 * then, compose the transformations from each hop in path
 * (interpolating to time t as necessary)
 */
public abstract class AbstractTransformDatabase implements TransformDatabase {
	
	
	HashMap<String,TransformBuffer> buffers;
	
	public AbstractTransformDatabase() {
		buffers = new HashMap<String,TransformBuffer>();
	}
	
	@Override
	public void add(StampedTransform f) {
		// TODO: if one exists in reverse order, invert it
		TransformBuffer buff = null;
		if(buffers.containsKey(f.getId())) {
			buff = buffers.get(f.getId());
		} else {
			buff = new TransformBuffer(f.parentFrame, f.childFrame);
			buffers.put( f.getId(), buff );
		}
		buff.put(f);		
	}

	@Override
	public void add(Collection<StampedTransform> transforms) {
		for(StampedTransform tx : transforms) add(tx);
	}
	
	public static String frameNamesToTransformId(String parentFrame, String childFrame) {
		return parentFrame + "->" + childFrame;
	}
	
}
