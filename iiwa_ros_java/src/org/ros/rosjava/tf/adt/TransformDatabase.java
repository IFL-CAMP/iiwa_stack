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

import org.ros.rosjava.tf.StampedTransform;
import org.ros.rosjava.tf.Transform;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 * @brief provides buffering of transforms over time
 */
public interface TransformDatabase {
	
	public void add(StampedTransform f);
	
	public void add(Collection<StampedTransform> transforms);
	
	public Transform lookupTransformBetween(String frame1, String frame2, long t);
	
}
