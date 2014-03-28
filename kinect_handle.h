#ifndef __KINECT_HANDLE__H__
#define __KINECT_HANDLE__H__
#include "common/image_util.h"
#include "image.h"
#include "pixel.h"
#include <vector>

void update_im_from_vect(const std::vector<uint8_t> & k_data,
		image_u32_t *im); 

image_u32_t *im_from_vect(const std::vector<uint8_t> & k_data); 



#endif
