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

import javax.vecmath.Matrix4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Quat4d;

import com.google.common.base.Strings;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 * @brief basic object representing transformation between to coordinate frames (e.g., "parent->child")
 */
public class Transform {

	public String parentFrame;
	public String childFrame;

	public Vector3d translation;
	public Quat4d rotation;

	// constructs new identity transform
	public Transform(String parentFrame, String childFrame) {
		this.parentFrame = parentFrame;
		this.childFrame = childFrame;
		this.translation = new Vector3d(0, 0, 0);
		this.rotation = new Quat4d(0, 0, 0, 1);
	}

	public Transform(
			String parentFrame, String childFrame,
			Vector3d translation, Quat4d rotation
	) {
		this.parentFrame = parentFrame;
		this.childFrame = childFrame;
		this.translation = translation;
		this.rotation = rotation;

		//System.out.println(toString());
	}

	// compose(frame1->frame2, frame2->frame3) yields frame1->frame3
	// in-place version
	public void compose(Transform g) {
		assert (this.childFrame == g.parentFrame);
		this.childFrame = g.childFrame;

		// switch to matrix form for calculations
		// maybe not as clean, but I forget how to do this with quaternions!		
		Matrix4d A = this.asMatrix();
		Matrix4d B = g.asMatrix();
		A.mul(B); // in-place

		this.set(A);
	}

	// compose(frame1->frame2, frame2->frame3) yields frame1->frame3
	public static Transform compose(Transform f, Transform g) {

		assert (f.childFrame == g.parentFrame);
		Transform h = new Transform(f.parentFrame, g.childFrame);

		// switch to matrix form for calculations
		// maybe not as clean, but I forget how to do this with quaternions!		
		Matrix4d A = f.asMatrix();
		Matrix4d B = g.asMatrix();
		A.mul(B); // in-place

		h.set(A);

		return h;

	}

	public void invert() {
		// switch to matrix form for inversion
		// maybe not as clean, but I forget how to do this with quaternions!
		Matrix4d M = this.asMatrix();
		M.invert(); // in-place
		this.set(M);
		String temp = this.parentFrame;
		this.parentFrame = this.childFrame;
		this.childFrame = temp;
	}

	// returns view of this transformation as a 4x4 matrix M such that x' = Mx for 4x1 vectors x in homogeneous coordinates
	// note that modifying this matrix won't modify underlying transformation
	public Matrix4d asMatrix() {
		return new Matrix4d(this.rotation, this.translation, 1.0d); // no scaling, so scale = 1
	}

	public void set(Matrix4d M) {
		M.get(this.translation); // populates translation vector from matrix M
		M.get(this.rotation); // populates rotation quaternion from matrix M
	}

	public static String frameNames2transformId(String parentFrame, String childFrame) {
		return parentFrame + "->" + childFrame;
	}

	public String getId() {
		return frameNames2transformId(parentFrame, childFrame);
		//return parentFrame + "->" + childFrame;
	}

	public String toString() {
		return String.format("{\n\tLink: %s -> %s,\n\tPosition: (%.2f, %.2f, %.2f),\n\tRotation: (%.2f, %.2f, %.2f, %.2f)\n}",
				parentFrame, childFrame,
				translation.x, translation.y, translation.z,
				rotation.x, rotation.y, rotation.z, rotation.w
		);
/*
		String s = "{ ";
		s += getId();
		s += "\n";
		s += "( " + translation.x + ", " + translation.y + ", " + translation.z + " )";
		s += "( " + rotation.x + ", " + rotation.y + ", " + rotation.z + ", " + rotation.w + " )";
		s += " }";
		return s;
		*/
	}

	// checks if the 6dof mathematical transform is the same;
	// frame names and timestamps don't matter
	public boolean equals(Transform tx) {
		return
				tx.translation.equals(this.translation) &&
						tx.rotation.equals(this.rotation);
	}

	@Override
	public Transform clone() {
		//System.out.println("cloning");

		return new Transform(
				parentFrame, childFrame, // immutable
				(Vector3d) translation.clone(),
				(Quat4d) rotation.clone()
		);
	}
}
