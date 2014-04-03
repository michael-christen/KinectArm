/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

      * File Name : kinect_handle.cpp

         * Purpose :

	    * Creation Date : 27-03-2014

	       * Last Modified : Thu 03 Apr 2014 03:40:47 PM EDT

	          * Created By : Michael Christen

		     _._._._._._._._._._._._._._._._._._._._._.*/
#include "kinect_handle.h"

void update_im_from_vect(const std::vector<uint8_t> & k_data, 
		image_u32_t *im) {
	int v_width = 640;
	int v_height= 480;
	printf("i_w: %d,v_w: %d, i_h: %d, v_h: %d\n",
			im->width, v_width, im->height, v_height);
	assert(im->width == v_width && im->height == v_height);
	for(int y = 0; y < v_height; ++y) {
		for(int x = 0; x < v_width; ++x) {
			im->buf[im->stride*y + x] = get_px(
					k_data[3*v_width*y+3*x + 0],
					k_data[3*v_width*y+3*x + 1],
					k_data[3*v_width*y+3*x + 2],
					0xff
			);
		}
	}
}


image_u32_t *im_from_vect(const std::vector<uint8_t> & k_data) {
	int v_width = 640;
	int v_height= 480;
	image_u32_t *im = image_u32_create(v_width, v_height);
	update_im_from_vect(k_data, im);
	return im;
};

uint32_t depthToIm(uint16_t depth, bool valid, Gradient gr) {
	uint8_t  scaled_down;
	double   tmp;
	//Not sure how to fix?
	uint16_t MAX_DEPTH_VAL = 0x1fff;
	tmp = (depth+0.0)/(MAX_DEPTH_VAL+0.0);
	tmp *= 0xff;
	scaled_down = 0xff - (uint8_t)tmp;
	if(valid) {
		return get_px(scaled_down, scaled_down, scaled_down, 0xFF);
	}
	return 0xFF000000;
}

uint32_t videoToIm(uint32_t video, bool valid, Gradient gr) {
	if(valid) {
		return dist_to_grey(gr.mag());
		return video;
	}
	return 0xFF000000;
}

double   videoToGrad(uint32_t px) {
	double h,s,v;
	RGBtoHSV(px,&h,&s,&v);
	//printf("px:%x -> V:%f\n",px,v);
	return 255.0*v;
	//return (double) (0xFFFFFF & px);
}
double   depthToGrad(uint16_t depth) {
	return (double) depth;
}

void make_depth_viewable(image_u32_t *im) {
	uint16_t val;
	uint8_t  scaled_down;
	double   tmp;
	//Not sure how to fix?
	uint16_t MAX_DEPTH_VAL = 0x1fff;
	int      id;
	for(int y = 0; y < im->height; ++y) {
		for(int x = 0; x < im->width; ++x) {
			id = im->stride*y+x;
			val = 0xffff & im->buf[id];
			tmp = (val+0.0)/(MAX_DEPTH_VAL+0.0);
			tmp *= 0xff;
			scaled_down = 0xff - (uint8_t)tmp;
			im->buf[id] = get_px(scaled_down, scaled_down,
					scaled_down, 0xFF);
		}
	}
}

uint16_t get_px_depth(uint32_t px) {
	return px & 0xffff;
}
	

