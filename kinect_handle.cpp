/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

      * File Name : kinect_handle.cpp

         * Purpose :

	    * Creation Date : 27-03-2014

	       * Last Modified : Sat 29 Mar 2014 12:35:52 AM EDT

	          * Created By : Michael Christen

		     _._._._._._._._._._._._._._._._._._._._._.*/
#include "kinect_handle.h"

void update_im_from_vect(const std::vector<uint8_t> & k_data, 
		image_u32_t *im) {
	int v_width = 640;
	int v_height= 480;
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


void make_depth_viewable(image_u32_t *im) {
	uint16_t val;
	uint8_t  scaled_down;
	double   tmp;
	uint16_t MAX_DEPTH_VAL = 0x0fff;
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

