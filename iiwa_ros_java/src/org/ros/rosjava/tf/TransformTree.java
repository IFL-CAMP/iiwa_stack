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

import java.util.List;
import java.util.Vector;

import org.jgrapht.graph.ListenableDirectedGraph;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.SimpleDirectedGraph;
import org.ros.rosjava.tf.adt.AbstractTransformDatabase;

/**
 * @author nick@heuristiclabs.com (Nick Armstrong-Crews)
 * @brief a specific data structure to support fast lookups
 */
public class TransformTree extends AbstractTransformDatabase {

	//protected SimpleDirectedGraph<String,TransformBuffer> graph;
	protected ListenableDirectedGraph<String,TransformBuffer> graph;
	
	public TransformTree() {
		super();
		//graph = new SimpleDirectedGraph<String,TransformBuffer>(TransformBuffer.class);
		graph = new ListenableDirectedGraph<String,TransformBuffer>(new SimpleDirectedGraph<String,TransformBuffer>(TransformBuffer.class));
	}

	@Override
	public void add(StampedTransform tx) {
		
		synchronized(graph) {

			if(!graph.containsVertex(tx.parentFrame))
				graph.addVertex(tx.parentFrame);
			if(!graph.containsVertex(tx.childFrame))
				graph.addVertex(tx.childFrame);

			
			TransformBuffer txBuff;
			if(!graph.containsEdge(tx.parentFrame, tx.childFrame)) {
				//graph.addEdge(tx.parentFrame, tx.childFrame, new TransformBuffer(tx.parentFrame, tx.childFrame));
				txBuff = new TransformBuffer(tx.parentFrame, tx.childFrame);
				txBuff.put(tx);
				graph.addEdge(tx.parentFrame, tx.childFrame, txBuff);
			}
			else {
				txBuff = graph.getEdge(tx.parentFrame, tx.childFrame);
				txBuff.put(tx);
			}
	
			/*
			// TODO: check for childFrame->parentFrame, invert it and add (and print warning)
			else if((txBuff = graph.getEdge(tx.childFrame, tx.parentFrame)) != null) {
				tx.invert();
				txBuff.put(tx);
			}
			 */
		}

	}

	// TODO: consider using other algorithm, e.g. D*, to dynamically update paths as new transforms are added
	// TODO: consider caching discovered paths
	// TODO: handle backwards traversal of edges (i.e., inversion of transforms)
	@Override
	public Transform lookupTransformBetween(String frame1, String frame2, long t) {
		//System.out.println("looking up path between "+frame1+" and "+frame2+" at "+t);

		if (!graph.containsVertex(frame1) || !graph.containsVertex(frame2)) {
			return null;
		}

		GraphPath<String,TransformBuffer> path = TransformTreePathLookup.findPathBetween(graph, frame1, frame2);

		if(path != null && path.getEdgeList().size() > 0) {
			Vector<Transform> txPath = new Vector<Transform>(path.getEdgeList().size());
			for( TransformBuffer txBuff : path.getEdgeList() ) {
				Transform tx = txBuff.lookupTransform(t);
				txPath.add(tx);
			}

			return TransformTree.collapseTransformPath(txPath);
		}
		else {
			return null;
		}
	}
	
	public static Transform collapseTransformPath(List<Transform> path) {
		assert(path != null);
		assert(path.size() > 0);

		Transform txAccum = path.remove(0); // start with first transform

		//int i = 0;
		//System.out.println("path["+(i++)+"]: "+txAccum);
		for (Transform tx : path) {
			//System.out.println("path["+(i++)+"]: "+tx);
			//txAccum = Transform.compose(txAccum, tx);
			txAccum.compose(tx);
		}
		return txAccum;
	}
	
	public boolean canTransform(String frame1, String frame2) {
		// NOTE: have to first check frames exist as vertices,
		//       otherwise TransformTreePathLookup.findPathBetween dies
		if (!graph.containsVertex(frame1)) {
			return false;
		}
		else if (!graph.containsVertex(frame2)) {
			return false;
		}
		else {
			GraphPath<String,TransformBuffer> path = TransformTreePathLookup.findPathBetween(graph, frame1, frame2);
			//return false;
			return path != null && path.getEdgeList().size() > 0;
		}
	}
	
	public boolean canTransform(String frame1, String frame2, long t) {
		GraphPath<String,TransformBuffer> path = TransformTreePathLookup.findPathBetween(graph, frame1, frame2);
		if(path == null || path.getEdgeList().size() == 0) {
			return false;
		}
		for (TransformBuffer txBuff : path.getEdgeList() ) {
			if(!txBuff.isValidAt(t)) return false;
		}
		return true;
	}
	
	public Transform lookupMostRecent(String frame1, String frame2) {
		GraphPath<String,TransformBuffer> path = TransformTreePathLookup.findPathBetween(graph, frame1, frame2);

		if (path != null && path.getEdgeList().size() > 0) {
			long mostRecentTime = Long.MAX_VALUE;

			for (TransformBuffer txBuff : path.getEdgeList() ) {
				mostRecentTime = Math.min(mostRecentTime, txBuff.mostRecentTime());
			}
			
			Vector<Transform> txPath = new Vector<Transform>(path.getEdgeList().size());
			for( TransformBuffer txBuff : path.getEdgeList() ) {
				txPath.add(txBuff.lookupTransform(mostRecentTime));
			}

			return TransformTree.collapseTransformPath(txPath);
		}
		else {
			return null;
		}
	}
		
	public ListenableDirectedGraph<String,TransformBuffer> getGraph() {
		return graph;
	}
	
}
