/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

 * File Name : Graph.cpp

 * Purpose :

 * Creation Date : 15-04-2014

 * Last Modified : Wed 16 Apr 2014 11:11:01 AM EDT

 * Created By : Michael Christen

 _._._._._._._._._._._._._._._._._._._._._.*/
#include "Graph.h"

std::map<int, G_Node > getGraphFromSkeleton(
		Image<double> &im) {
	std::map<int, G_Node> graph;
	std::vector<int> neighbors;

	//Traverse each element in case graphs aren't all connected
	for(size_t i = 0; i < im.size(); ++i) {
		if(im.isValid(i)) {
			//Insert into graph
			graph[i] = G_Node(i);
			//Just get immediate neighbors for now
			neighbors = im.getBlockNeighborIds(i,8);
			for(size_t j = 0; j < neighbors.size(); ++j) {
				int id = neighbors[j];
				//If valid, add edge
				if(im.isValid(id)) {
					graph[i].addNode(id);
				}
			}
		}
	}
	return graph;
}

std::vector<int> getEndPoints(
		Image<double> &im,
		std::map<int,G_Node> &graph, 
		int num_pts) {
	std::vector<int> endPoints;
	//Find lowest point near 320 X
	int start;
	int closest_y_dist = 240;
	int closest_x_dist = 320;
	int min_id = 0;
	for(auto it = graph.begin(); it != graph.end();
			++it) {
		int id = it->first;
		int x  = im.getX(id);
		int y  = im.getY(id);
		int x_dist = abs(320-x);
		int y_dist = abs(240-y);
		if(y_dist < closest_y_dist) {
			min_id = id;
			//min_y = y;
			closest_y_dist = y_dist;
			closest_x_dist = x_dist;
		} else if(y_dist == closest_y_dist) {
			if(x_dist < closest_x_dist) {
				min_id = id;
				closest_x_dist = x_dist;
			}
		}
	}
	start = min_id;
	endPoints.push_back(start);
	double prev_time, cur_time;
	//Get end points
	for(int i = 0; i < num_pts; ++i) {
		clearDist(graph);
		graph[start].min_dist = 0;
		for(int j = 0; j < i; ++j) {
			graph[endPoints[j]].min_dist = 0;
		}
		prev_time = utime_now()/1000000.0;
		int id = dijkstra(graph,im,start);
		cur_time = utime_now()/1000000.0;
		//printf("Dijkstra time = %f*20=%f\n",cur_time-prev_time, (cur_time-prev_time)*20);
		endPoints.push_back(id);
	}
	/*
	if(!endPoints.empty()) {
		printf("FOUND IT size: %d\n", endPoints.size());
	}
	*/
	return endPoints;
}

void clearDist(std::map<int, G_Node> &graph) {
	for(auto it = graph.begin(); it != graph.end(); ++it) {
		it->second.min_dist = DBL_MAX;
	}
}

int dijkstra(
		std::map<int, G_Node> & graph,
		Image<double> &d_transf,
		int start
		) {
	std::vector<bool> visited = std::vector<bool>(d_transf.size(),false);
	visited[start] = true;
	int numVisited = 1;
	G_Node temp = graph[start];
	//Visit all of the nodes
	while(numVisited != graph.size()) {
		//Change distances if needed
		for(int i = 0; i < temp.nodes.size(); ++i) {
			int id = temp.nodes[i];
			if(!visited[id]) {
				//Might want to compute actual euclidean dist
				double dist = temp.min_dist + 1;
				if(dist < graph[temp.nodes[i]].min_dist) {
					graph[temp.nodes[i]].min_dist =
						dist;
				}
			}
		}
		//Find smallest distance
		double min_dist = DBL_MAX;
		int min_id = -1;
		for(auto it = graph.begin(); it != graph.end(); ++it) {
			if(it->second.min_dist < min_dist && 
					!visited[it->first])  {
				min_dist = it->second.min_dist;
				min_id   = it->first;
			}
		}
		if(min_id < 0) {
			/*
			printf("numVisits: %d, size: %d\n",
					numVisited, graph.size());
					*/
			break;
		}
		assert(min_id >= 0);
		visited[min_id] = true;
		numVisited ++;
		temp = graph[min_id];
	}
	//Now that all of the closest distances are computed,
	//Return the largest one
	double max_dist = 0;
	int max_id;
	for(auto it = graph.begin(); it != graph.end(); ++it) {
		double dist = it->second.min_dist;
	    if(	dist > max_dist && dist != DBL_MAX) {
			max_dist = dist;
			max_id   = it->first;
		}
	}
	return max_id;
}


