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

import javax.vecmath.Vector3d;
import javax.vecmath.Quat4d;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 * @brief transform with a timestamp
 */
public class StampedTransform extends Transform {
	
	public long timestamp;
		
	public StampedTransform(
								long timestamp,
								String parentFrame, String childFrame,
								Vector3d translation, Quat4d rotation
							)
	{
		super(parentFrame, childFrame, translation, rotation);
		this.timestamp = timestamp;
	}
	
	// f = alpha * f0 + (1-alpha) * f1
	public static Transform interpolate(StampedTransform f0, StampedTransform f1, double alpha) {
//		assert(f0.parentFrame == f1.parentFrame && f0.childFrame == f1.childFrame);
//		assert(alpha >= 0 && alpha <= 1);
		// linear interp on translation
		Vector3d v = new Vector3d();
		v.interpolate(f0.translation, f1.translation, alpha);
		// slerp on quaternion
		Quat4d q = new Quat4d();
		q.interpolate(f0.rotation, f1.rotation, alpha);
		return new Transform(f0.parentFrame, f0.childFrame, v, q);
	}

	@Override
	public StampedTransform clone() {
		return new StampedTransform(
								timestamp,
								parentFrame, childFrame, // immutable
								(Vector3d) translation.clone(),
								(Quat4d) rotation.clone()
				
				);
	}

}
