#ifndef __FILTER__H__
#define __FILTER__H__
#include "kinect_handle.h"
#include "Image.h"
#include<vector>

#define MIN_ALLOWED_DEPTH 0x01ff

void filter_front(Image<uint16_t> & im);

//BFS to merge stuff into an object, if close, from 
void blob_merging(Image<uint16_t> &im, int start_id);

//returns true if depth_0 is close enough to depth_1
bool px_close_enough(uint16_t depth_0, uint16_t depth_1);

//Get id's of valid neighbors @ (x,y)
/*
std::vector<int> getNeighbors(image_u32_t *im, int x, int y);
*/


#endif
