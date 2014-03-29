/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

	* File Name : filter.cpp
	* Purpose :
	* Creation Date : 29-03-2014
	* Last Modified : Sat 29 Mar 2014 03:15:31 PM EDT
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


void filter_front(image_u32_t *im) {
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
	int      min_id    = 0;
	for(int y = 0; y < im->height; ++y) {
		for(int x = 0; x < im->width; ++x) {
			int id = im->stride*y + x;
			uint32_t px = im->buf[id];
			uint16_t depth = get_px_depth(px);
			if(depth >= MIN_ALLOWED_DEPTH && depth < min_depth) {
				//If it's a noise value, ignore it
				if(std::find(NOISE_DEPTHS.begin(), NOISE_DEPTHS.end(),
							depth) != NOISE_DEPTHS.end()) {
					continue;
				}
				min_id = id;
				min_depth = depth;
			}
		}
	}
	//min_id = im->width/2 + im->height/2 * im->stride;
	printf("Min depth: %x\n", min_depth);
	blob_merging(im, min_id);
}

void blob_merging(image_u32_t *im, int start) {
	int id;
	uint32_t px;
	std::queue<int> search;
	std::vector<int> passed;
	std::vector<bool> visited   = std::vector<bool>(im->height*im->stride, false);
	std::vector<bool> pass_vect = std::vector<bool>(im->height*im->stride, false);
	search.push(start);
	while(!search.empty()) {
		//printf("passed_size: %d\n",passed.size());
		int cur_id = search.front();
		search.pop();
		if(cur_id < 0 || cur_id >= im->height*im->stride) {
			assert(0);
			assert(cur_id >= 0 && cur_id < im->height*im->stride);
		}
		if( visited[cur_id]) {
			continue;
		}
		passed.push_back(cur_id);
		pass_vect[cur_id] = true;
		visited[cur_id] = true;
		int x = cur_id % im->stride;
		int y = cur_id / im->stride;
		std::vector<int> neighbors = getNeighbors(im, x, y);
		for(size_t i = 0; i < neighbors.size(); ++i) {
			id = neighbors[i];
			px = im->buf[id];
			if(!visited[id] && 
					px_close_enough(
						get_px_depth(px),
						get_px_depth(im->buf[cur_id])
					) &&
					get_px_depth(px) >= MIN_ALLOWED_DEPTH) {
				search.push(id);
			}
		}
	}
	//Mark
	/*
	for(size_t i = 0; i < passed.size(); ++i) {
		id = passed[i];
		im->buf[id] = 0xff00ff00;
	}
	*/
	//Filter out everything else
	for(int y = 0; y < im->height; ++y) {
		for(int x = 0; x < im->width; ++x) {
			int id = im->stride*y + x;
			if(!pass_vect[id]) {
				im->buf[id] = 0xffffffff;
			}
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
				break;
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
