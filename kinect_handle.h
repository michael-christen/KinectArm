#ifndef __KINECT_HANDLE__H__
#define __KINECT_HANDLE__H__
#include "common/image_util.h"
#include "image.h"
#include "pixel.h"
#include <vector>

void update_im_from_vect(const std::vector<uint8_t> & k_data,
		image_u32_t *im); 

image_u32_t *im_from_vect(const std::vector<uint8_t> & k_data); 

void make_depth_viewable(image_u32_t *im);

uint16_t get_px_depth(uint32_t px);



#endif
