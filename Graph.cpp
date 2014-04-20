/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

 * File Name : Graph.cpp

 * Purpose :

 * Creation Date : 15-04-2014

 * Last Modified : Sun 20 Apr 2014 09:05:30 AM EDT

 * Created By : Michael Christen

 _._._._._._._._._._._._._._._._._._._._._.*/
#include "Graph.h"
#include "body.h"

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
			neighbors = im.getBlockNeighborIds(i,15);
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

void getBodyFromEndPoints(state_t * state,
		Image<double> &im,
		std::map<int, G_Node> & graph,
		std::vector<int> & points) {

	//Calculate paths
	int midpoint = points[0];
	if(points.begin() != points.end()) {
		points.erase(points.begin()+0);
	}
	//Get lowest points and call those the feet
	int lowest_id = 0;
	int lowest_val = 0;
	for(int i = 0; i < points.size(); ++i) {
		if(im.getY(points[i]) > lowest_val) {
			lowest_val = im.getY(points[i]);
			lowest_id  = points[i];
		}
	}
	//Get similar y
	int closest_id = 0; 
	int closest_val = 480;
	for(int i = 0; i < points.size(); ++i) {
		if(points[i] == lowest_id) {
			continue;
		}
		int dist = abs(im.getY(points[i]) - lowest_val);
		if(dist < closest_val) {
			closest_id = points[i];
			closest_val = dist;
		}
	}
	int left_is_lowest = im.getX(closest_id) > im.getX(lowest_id);
	int left_foot = left_is_lowest ? lowest_id : closest_id; 
	int right_foot = left_is_lowest ? closest_id : lowest_id;
	auto it = std::find(points.begin(),points.end(),left_foot);
	if(it != points.end()) {
		points.erase(it);
	}
	it = std::find(points.begin(),points.end(),right_foot);
	if(it != points.end()) {
		points.erase(it);
	}
	//Head is in the middle
	int middleX = im.getX(midpoint);
	int middleY = im.getY(midpoint);
	int closest_x = 320;
	int closest_y = 0;
	closest_id = points.front();
	for(int i = 0; i < points.size(); ++i) {
		int xDist = abs(im.getX(points[i])-middleX);
		int yDist = abs(im.getY(points[i])-middleY);
		if(xDist < closest_x && im.getY(points[i]) < middleY) {
			closest_x = xDist;
			closest_y = yDist;
			closest_id = points[i];
		}
	}
	int head = closest_id;
	it = std::find(points.begin(),points.end(),head);
	if(it != points.end()) {
		points.erase(it);
	}
	int left_wrist, right_wrist;
	bool got_wrists  = false;
	if(points.size() > 1) {
		got_wrists = true;
		int left_is_first = im.getX(points[0]) < im.getX(points[1]);
		left_wrist = left_is_first ? 
			points[0] : points[1];
		right_wrist = left_is_first ? 
			points[1] : points[0];
		it = std::find(points.begin(),points.end(),left_wrist);
		if(it != points.end()) {
			points.erase(it);
		}
		it = std::find(points.begin(),points.end(),right_wrist);
		if(it != points.end()) {
			points.erase(it);
		}
	}

	if(got_wrists) {
		int start = midpoint;
		clearDist(graph);
		graph[start].min_dist = 0;
		//Perform dijkstra again to get dists
		dijkstra(graph,im,start);
		int parent = left_wrist;
		int oldParent = parent;
		while(graph.find(parent) != graph.end() && parent != start) {
			parent = graph[parent].parent;
			if(parent == oldParent) {
				break;
			}
			oldParent = parent;
			points.push_back(parent);
		}
		int left_elbow, left_shoulder;
		left_elbow    = points[points.size()/3];
		left_shoulder = points[5*points.size()/8];
		state->pts = points;
		//state->pts = points;
		//Assign
		state->body.setJoint(MIDPOINT, getReal(state->depth,midpoint));
		state->body.setJoint(HEAD, getReal(state->depth,head));
		state->body.setJoint(LWRIST, getReal(state->depth,right_wrist));
		//state->body.setJoint(LELBOW, getReal(state->depth,points[5]));
		//state->body.setJoint(LSHOULDER, getReal(state->depth,points[5]));
		state->body.setJoint(RWRIST, getReal(state->depth,left_wrist));
		state->body.setJoint(RELBOW, getReal(state->depth,left_elbow));
		state->body.setJoint(RSHOULDER, getReal(state->depth,left_shoulder));
		state->body.setJoint(RFOOT, getReal(state->depth,left_foot));
		state->body.setJoint(LFOOT, getReal(state->depth,right_foot));
	}
}

void getBodyPoints(state_t * state,
		Image<double> &d_transf,
		std::map<int, G_Node> & graph) {

	std::vector<int> points =
		getEndPoints(d_transf, graph, 5);
	//state->pts = points;
	getBodyFromEndPoints(
			state,
			d_transf,
			graph, points);
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
	clearDist(graph);
	for(int i = 0; i < num_pts; ++i) {
		//clearDist(graph);
		for(int j = 0; j <= i; ++j) {
			graph[endPoints[j]].min_dist = 0;
		}
		prev_time = utime_now()/1000000.0;
		int id = dijkstra(graph,im,start);
		cur_time = utime_now()/1000000.0;
		//printf("Dijkstra time = %f*20=%f\n",cur_time-prev_time, (cur_time-prev_time)*20);
		start = id;
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
 #define dNode std::pair<int, double>

class dNodeComp {
	public:
		bool operator() (const dNode &a, const dNode &b) const{
			return a.second < b.second;
		}
};

int dijkstra(
		std::map<int, G_Node> & graph,
		Image<double> &d_transf,
		int start
		)  {
	//std::vector<bool> visited = std::vector<bool>(d_transf.size(),false);
	std::priority_queue<dNode, std::vector<dNode>, dNodeComp> pQ;

	//visited[start] = true;
	int numVisited = 1;
	//G_Node temp = graph[start];
	//Visit all of the nodes
	pQ.push(dNode(start, 0.0));
	while(!pQ.empty()) {
		dNode min = pQ.top();
		pQ.pop();
		G_Node temp = graph[min.first];
		//Change distances if needed
		for(int i = 0; i < temp.nodes.size(); ++i) {
			int id = temp.nodes[i];
	//		if(!visited[id]) {
				//Might want to compute actual euclidean dist
				double dist = temp.min_dist + 1;
				if(dist < graph[temp.nodes[i]].min_dist) {
					graph[temp.nodes[i]].min_dist =
						dist;
					graph[temp.nodes[i]].parent = min.first;
					pQ.push(dNode(temp.nodes[i],dist));
				}
	//		}
		}
		/*
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
			printf("numVisits: %d, size: %d\n",
					numVisited, graph.size());
			break;
		}
		assert(min_id >= 0);
		visited[min_id] = true;
		numVisited ++;
		temp = graph[min_id];
		*/
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


