#ifndef __FILTER__H__
#define __FILTER__H__
#include "kinect_handle.h"
#include "Image.h"
#include "Blob.h"
#include "Gradient.h"
#include<vector>
#include<queue>

#define MIN_ALLOWED_DEPTH 0x01ff

void filter_front(Image<uint16_t> & im);

//BFS to merge stuff into an object, if close, from 
void blob_merging(Image<uint16_t> &im, int start_id);

//returns true if depth_0 is close enough to depth_1
bool px_close_enough(uint16_t depth_0, uint16_t depth_1);

double getThetaDist(double from, double to);
double sign(double val);

bool grad_close_enough(Gradient cur, Gradient other);

//Get id's of valid neighbors @ (x,y)
/*
std::vector<int> getNeighbors(image_u32_t *im, int x, int y);
*/
template <typename T>
std::vector<Blob<Gradient>> get_gradient_blobs(Image<T> &im);

template <typename T>
Blob<Gradient> get_gradient_blob(Image<T> &im, int start_id);

template <typename T>
Blob<Gradient> get_gradient_blob(Image<T> &im, 
		std::vector<bool> &visited, int start_id) {
	Blob<Gradient> blob;
	std::queue<int> search;
	static std::vector<bool> neighbor_search =
		std::vector<bool>(8,true);

	search.push(start_id);
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
		int x = im.getX(cur_id);
		int y = im.getY(cur_id);
		visited[cur_id] = true;
		blob.push(x,y,im.gradient[cur_id]);
		im.set(cur_id,0);
		std::vector<int> neighbors = im.getNeighborIds(x, y,
				neighbor_search, grad_close_enough);
		for(size_t i = 0; i < neighbors.size(); ++i) {
			int id = neighbors[i];
			if(!visited[id]) {
				search.push(id);
			}
		}
	}
	return blob;
}

template <typename T>
std::vector<Blob<Gradient>> get_gradient_blobs(Image<T> &im) {
	std::vector<Blob<Gradient>> blobs;
	std::vector<bool> visited   =
		std::vector<bool>(im.h()*im.w(), false);
	for(size_t i = 0; i < im.size(); ++i) {
		if(im.gradient[i].mag() > 70 && !visited[i]) {
			Blob<Gradient> blob = get_gradient_blob(im,visited,i);
			if(blob.size() > 10) {
				blobs.push_back(blob);
			}
		}
	}
	printf("num_blobs = %d\n",blobs.size());
	return blobs;
}
#endif
