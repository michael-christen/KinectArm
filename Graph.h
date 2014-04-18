#ifndef __GRAPH__H__
#define __GRAPH__H__
#include "Image.h"
#include "pixel.h"
#include "Blob.h"
#include "Gradient.h"
#include "Line.h"
#include "joint.h"
#include "body.h"
#include "vision_state.h"
#include "common/timestamp.h"
#include<climits>
#include<map>
#include<vector>
#include<queue>
#include<algorithm>
#include<float.h>

class G_Node {
	public:
		int id;
		double min_dist;
		int parent;
		std::vector<int> nodes;
		G_Node() {
			id = 0;
			nodes = std::vector<int>();
		}
		G_Node(int i) {
			id = i;
			nodes = std::vector<int>();
		};
		void addNode(int i) {
			nodes.push_back(i);
		}
};

std::map<int, G_Node> getGraphFromSkeleton(Image<double> &d_transf);

std::vector<int> getEndPoints(
		Image<double> &d_transf,
		std::map<int,G_Node> & graph, 
		int num_pts);

void getBodyPoints(state_t * state,
		Image<double> &d_transf,
		std::map<int, G_Node> & graph);

void getBodyFromEndPoints(state_t * state,
		Image<double> &d_transf,
		std::map<int, G_Node> & graph,
		std::vector<int> & points);

void clearDist(std::map<int, G_Node> &graph);

//Gets farthest point
int dijkstra(
		std::map<int, G_Node> & graph,
		Image<double> &d_transf,
		int start
		);

#endif

