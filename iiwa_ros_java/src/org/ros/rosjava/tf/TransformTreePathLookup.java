package org.ros.rosjava.tf;

import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.graph.GraphPathImpl;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class TransformTreePathLookup {
	Graph<String, TransformBuffer> treeGraph;

	public TransformTreePathLookup(Graph<String, TransformBuffer> treeGraph) {
		this.treeGraph = treeGraph;
	}

	public GraphPath<String,TransformBuffer> findPathBetween(String source, String target) {
		if (!treeGraph.containsVertex(source) || !treeGraph.containsVertex(target)) {
			return null;
		}

		List<String> sourceRootPath = getPathRoot(source);
		List<String> targetRootPath = getPathRoot(target);

		List<TransformBuffer> edgeList = new ArrayList<TransformBuffer>();
		List<String> vertexList = new ArrayList<String>();

		String prev = null;
		String sharedParent = null;

		for (String vertex : sourceRootPath) {
			vertexList.add(vertex);

			if (prev != null) {
				TransformBuffer edge = treeGraph.getEdge(vertex, prev);
				edgeList.add(new InvertedTransformBuffer(edge));
			}

			prev = vertex;

			if (targetRootPath.contains(vertex)) {
				sharedParent = vertex;
				break;
			}
		}

		if (sharedParent == null) {
			System.err.println("No connection found between "+source+" and "+target);
			return null;
		}

		boolean skip = true;
		for (int i = targetRootPath.size()-1; i>=0; i--) {
			String vertex = targetRootPath.get(i);
			if (skip) {
				if (vertex.equals(sharedParent)) {
					skip = false;
					// found first know node.
					// Continue nevertheless, as we need two vertices to query an edge
				}

				continue;
			}

			vertexList.add(vertex);
			edgeList.add(treeGraph.getEdge(prev, vertex));

			prev = vertex;
		}

		//System.out.println("SharedParent: "+ sharedParent);
		//System.out.println(vertexList);
		//System.out.println(edgeList);

		return new GraphPathImpl<String, TransformBuffer>(treeGraph, source, target, edgeList, edgeList.size());
	}

	public List<String> getPathRoot(String vertex) {
		Set<TransformBuffer> upEdges = getIncomingEdgesOf(vertex);
		List<String> result = new ArrayList<String>();
		result.add(vertex);

		if (upEdges.size() == 0) {
			return result;
		}
		else if (upEdges.size() == 1) {
			TransformBuffer edgeToParnet = upEdges.iterator().next();
			result.addAll(getPathRoot(edgeToParnet.parentFrame));
			return result;
		}
		else {
			System.err.println("Error: Frame "+vertex+" has more than one parent!");
			return null;
		}
	}

	public Set<TransformBuffer> getIncomingEdgesOf(String vertex) {
		Set<TransformBuffer> allEdges = treeGraph.edgesOf(vertex);
		Set<TransformBuffer> edges = new HashSet<TransformBuffer>();
		
		for(TransformBuffer txBuffer : allEdges) {
			if(txBuffer.childFrame.equals(vertex)) {
				edges.add(txBuffer);	
			}
		}
		
		return edges;
	}
	
	public static GraphPath<String,TransformBuffer> findPathBetween(Graph<String, TransformBuffer> graph, String source, String target) {
		TransformTreePathLookup treePathLookup = new TransformTreePathLookup(graph);
		return treePathLookup.findPathBetween(source, target);
	}
}
