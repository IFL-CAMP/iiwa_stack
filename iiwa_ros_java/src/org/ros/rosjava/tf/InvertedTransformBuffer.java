package org.ros.rosjava.tf;

import java.util.TreeMap;

public class InvertedTransformBuffer extends TransformBuffer {
	public InvertedTransformBuffer(TransformBuffer tx) {
		super(tx.childFrame, tx.parentFrame, tx.maxCapacity, tx.history);
	}

	public void put(StampedTransform tx) {
		assert(tx.parentFrame == this.childFrame && tx.childFrame == this.parentFrame);
		//assert(tx.timestamp > ) // make sure it's new
		history.put(new Long(tx.timestamp), tx);
	}

	public Transform lookupTransform(long t) {
		Transform tx = super.lookupTransform(t).clone();
		//System.out.println("regular transformation: "+tx);
		tx.invert();
		//System.out.println("inverted transformation: "+tx);
		return tx;
	}

	public StampedTransform mostRecentTransform() {
		StampedTransform tx = history.lastEntry().getValue().clone();
		tx.invert();
		return tx;
	}
}
