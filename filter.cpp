/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

	* File Name : filter.cpp
	* Purpose :
	* Creation Date : 29-03-2014
	* Last Modified : Fri 11 Apr 2014 10:51:02 PM EDT
	* Created By : Michael Christen

_._._._._._._._._._._._._._._._._._._._._.*/

#include<iostream>
#include<cmath>
#include<string>
#include<vector>
#include<queue>
#include<algorithm>
#include<cassert>

#include "filter.h"


void filter_front(Image<uint16_t> & im) {
	//Find closest pixel that isn't 0
	uint16_t min_depth = 0xffff;
	//For some reason these values showed up at areas where they
	//shouldn't, filtering them out seemed to make things work much
	//better
	std::vector<uint16_t> NOISE_DEPTHS; 
	NOISE_DEPTHS.push_back(0x203);
	NOISE_DEPTHS.push_back(0x208);
	NOISE_DEPTHS.push_back(0x20f);
	NOISE_DEPTHS.push_back(0x2c0);
	NOISE_DEPTHS.push_back(0x2d8);
	NOISE_DEPTHS.push_back(0x472);
	NOISE_DEPTHS.push_back(0x4b2);
	NOISE_DEPTHS.push_back(0x6e0);
	NOISE_DEPTHS.push_back(0x780);
	NOISE_DEPTHS.push_back(0x9f8);
	NOISE_DEPTHS.push_back(513);
	NOISE_DEPTHS.push_back(517);
	int      min_id    = 0;
	for(size_t i = 0; i < im.size(); ++i) {
		uint16_t depth = im.get(i);
		if(depth >= MIN_ALLOWED_DEPTH &&
				depth < min_depth) {
			//If it's a noise value, ignore it
			if(std::find(NOISE_DEPTHS.begin(), NOISE_DEPTHS.end(),
						depth) != NOISE_DEPTHS.end()) {
				continue;
			}
			min_id = i;
			min_depth = depth;
		}
	}
	printf("Min depth: %d\n", min_depth);
	blob_merging(im, min_id);
}

void blob_merging(Image<uint16_t> &im, int start) {
	int id;
	uint32_t px;
	std::queue<int> search;
	std::vector<int> passed;
	std::vector<bool> visited   =
		std::vector<bool>(im.h()*im.w(), false);
	std::vector<bool> pass_vect =
		std::vector<bool>(im.h()*im.w(), false);
	static std::vector<bool> neighbor_search =
		std::vector<bool>(8,true);
	search.push(start);
	while(!search.empty()) {
		//printf("passed_size: %d\n",passed.size());
		size_t cur_id = search.front();
		search.pop();
		if(cur_id < 0 || cur_id >= im.size()) {
			assert(0);
			assert(cur_id >= 0 && cur_id < im.size());
		}
		if( visited[cur_id]) {
			continue;
		}
		passed.push_back(cur_id);
		pass_vect[cur_id] = true;
		visited[cur_id] = true;
		int x = cur_id % im.w();
		int y = cur_id / im.w();
		std::vector<int> neighbors = im.getNeighborIds(x, y,
				neighbor_search, px_close_enough);
		for(size_t i = 0; i < neighbors.size(); ++i) {
			id = neighbors[i];
			px = im.get(id);
			if(!visited[id] && 
					px_close_enough(px, im.get(cur_id)) &&
					px >= MIN_ALLOWED_DEPTH) {
				search.push(id);
			}
		}
	}
	//Mark
	//for(size_t i = 0; i < passed.size(); ++i) {
	//	id = passed[i];
	//	im->buf[id] = 0xff00ff00;
	//}
	//Filter out everything else
	for(size_t i = 0; i < im.size(); ++i) {
		if(!pass_vect[i]) {
			im.invalidate(i);
		}
	}
}


std::vector<int> getNeighbors(image_u32_t *im, int x, int y) {
	int id;
	uint32_t px;
	uint16_t depth, cur_depth;
	std::vector<int> neighbors;
	cur_depth  = im->buf[im->stride*y + x];
	//Searching *'s
	//***
	//*.*
	//***
	for(int j = -1; j <= 1; ++j) {
		//edge
		if(j + y < 0 || j + y > im->height) {
			continue;
		}
		for(int i = -1; i <= 1; ++i) {
			//edge
			if(i + x < 0 || i + x > im->width) {
				continue;
			}
			//only search *'s
			if(j == 0 && i == 0) {
				continue;
			}
			id = im->stride*(y+j) + (x+i);

			if(id < 0 || id >= im->height*im->stride) {
				continue;
			}
			px = im->buf[id];
			depth = get_px_depth(px);
			if(px_close_enough(cur_depth,depth)) {
				neighbors.push_back(id);
			}
		}
	}
	return neighbors;
}

bool px_close_enough(uint16_t depth_0, uint16_t depth_1) {
	return abs(depth_0 - depth_1) < 700;
}

bool grad_close_enough(Gradient cur, Gradient other) {
	return (cur.mag() > 70 && other.mag() > 70) && 
		//0.7 works well for image, but depth is a little too
		//jittery
		fabs(getThetaDist(cur.angle(),other.angle())) < 0.15;
}

bool get_all(uint16_t a, uint16_t b) {
	return true;
}


std::vector<double> get_dist_transform(Image<uint16_t> & im) {
	std::vector<double> dist   = std::vector<double>(im.size(),0);
	std::vector<bool> visited  = std::vector<bool>(im.size(), false);
	std::vector<bool> new_visited = std::vector<bool>(im.size(), false);
	std::vector<bool> neighbor = std::vector<bool>(8, true);
	//Initialize large gradients to visited
	for(int i = 0; i < im.size(); ++i) {
		if(im.gradient[i].mag() > 50) {
			visited[i] = true;
		}
	}
	//Leave far edges alone
	bool done = false;
	int num_times = 0;
	new_visited = visited;
	while(!done) {
		num_times ++;
		done = true;
		for(int x = 1; x < im.w()-1; ++x) {
			for(int y = 1; y < im.h()-1; ++y) {
				if(!visited[im.id(x,y)] && im.isValid(x,y)) {
					std::vector<int> neighborIds = 
						im.getNeighborIds(x,y,neighbor,get_all);
					//Get min dist
					double min_d = 999999999;
					bool found_min = false;
					for(size_t i = 0; i < neighborIds.size(); ++i) {
						int id = neighborIds[i];
						if(visited[id] && dist[id] < min_d) {
							found_min = true;
							min_d = dist[id];
						}
					}
					if(found_min) {
						new_visited[im.id(x,y)] = true;
						dist[im.id(x,y)]    = min_d+1;
					} else {
						done = false;
					}
				} else {
					new_visited[im.id(x,y)] = true;
				}
			}
		}
		visited = new_visited;
	}
	printf("num_times = %d\n",num_times);
	return dist;
}

double getThetaDist(double from, double to) {

	from = fmod(from, 2*M_PI);
	to   = fmod(to, 2*M_PI);
	double difference = fmod(to - from, 2*M_PI);
	double sn = sign(difference);
	if(fabs(difference) > M_PI) {
		difference = -sn*M_PI + fmod(difference, M_PI);
	}
	return difference;
}
double sign(double val) {
	return val < 0  ? -1.0 : 1.0;
}

