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

package org.ros.rosjava.tf;

import java.util.Map;
import java.util.TreeMap;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

// TODO: switch to different map structure, since we always add newer transfers and tree becomes unbalanced
// TODO: we can infer whether or not transform is static or dynamic! use that to decide if canTransform(), and for extrapolation (forward and backward)
/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 * @brief stores a history of a single transform (e.g., "parent->child"); backed by a TreeMap for fast lookups (lg N)
 */
public class TransformBuffer {

	public final TreeMap<Long,StampedTransform> history;

	public final String parentFrame;
	public final String childFrame;
	public final int maxCapacity;

	public static final int DEFAULT_MAX_CAPACITY = 100; // a sensible default?
	
	public TransformBuffer(String parentFrame, String childFrame) {
		this.parentFrame = parentFrame;
		this.childFrame = childFrame;
		this.maxCapacity = DEFAULT_MAX_CAPACITY;
		history = new TreeMap<Long,StampedTransform>();
	}
	
	public TransformBuffer(String parentFrame, String childFrame, int maxCapacity) {
		this.parentFrame = parentFrame;
		this.childFrame = childFrame;
		this.maxCapacity = maxCapacity;
		history = new TreeMap<Long,StampedTransform>();
	}

	protected TransformBuffer(String parentFrame, String childFrame, int maxCapacity, TreeMap<Long, StampedTransform> history) {
		this.parentFrame = parentFrame;
		this.childFrame = childFrame;
		this.maxCapacity = maxCapacity;
		this.history = history;
	}

	public void put(StampedTransform tx) {
		assert(tx.parentFrame == this.parentFrame && tx.childFrame == this.childFrame);
		//assert(tx.timestamp > ) // make sure it's new
		history.put(new Long(tx.timestamp), tx);

		while (history.size() > maxCapacity) {
			history.remove(history.firstEntry().getKey());
		}
	}
	
	/**
	 * Lookup a transformation to a given time.
	 * Use t=0 to get the most recent transformation.
	 * @param t
	 * @return
	 */
	public Transform lookupTransform(long t) {
		if (t == 0) {
			return history.lastEntry().getValue().clone();
		}
		else if(t < history.firstKey()) {
			System.err.println("No transformation found for time "+t);
			return new Transform(Transform.frameNames2transformId(parentFrame, childFrame),
					"tfb" + t, new Vector3d(0,0,0), new Quat4d(0,0,0,1));
		}
		else if(t > history.lastKey()) {
			System.out.println("No transformation found for time "+t+". Using most recent transformation");
			// TODO: wait for next pose update, then return interpolated result
			return history.lastEntry().getValue().clone();
		}
		else if(history.containsKey(t)) { // if we have it exactly,
			return history.get(t).clone(); // just send it back
		}
		else { // if not, have to interpolate between enclosing two
			StampedTransform f0 = history.floorEntry(t).getValue();
			StampedTransform f1 = history.ceilingEntry(t).getValue();
			long t0 = f0.timestamp;
			long t1 = f1.timestamp;
			double alpha = ((double) (t - t0)) / (t1 - t0); // relative weight of f0 (and weight of f1 = 1 - weight of f0)
			return StampedTransform.interpolate(f0, f1, alpha);
		}
	}

	public String getId() { return Transform.frameNames2transformId(parentFrame, childFrame); }
	
	public int size() { return history.size(); }
	
	public boolean isValidAt(long t) {
		if(history.containsKey(t)) return true;
		else if(t > history.firstKey() && t < history.lastKey()) return true;
		else return false;
	}
	
	// TODO: add tolerance version
	//public boolean isValidAt(long t, long tolerance) {}
	
	public long mostRecentTime() {
		return history.lastKey();
	}
	
	public StampedTransform mostRecentTransform() {
		return history.lastEntry().getValue();
	}

	public String toString() {
		return "TransformBuffer("+parentFrame+" -> "+childFrame+"; "+history.size()+" saved transformations between "+history.firstEntry().getKey()+" and "+history.lastEntry().getKey()+")";
	}
}
