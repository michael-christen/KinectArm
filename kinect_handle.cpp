/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

      * File Name : kinect_handle.cpp

         * Purpose :

	    * Creation Date : 27-03-2014

	       * Last Modified : Thu 27 Mar 2014 03:58:52 PM EDT

	          * Created By : Michael Christen

		     _._._._._._._._._._._._._._._._._._._._._.*/
#include "kinect_handle.h"

void update_im_from_vect(static std::vector<uint8_t> k_data, 
		image_u32_t *im) {
	int v_width = 640;
	int v_height= 480;
	assert(im->width == v_width && im->height == v_height);
	for(int y = 0; y < v_height; ++y) {
		for(int x = 0; x < v_width; ++x) {
			im->buf[im->stride*y + x] = get_px(
					k_data[v_width*y+x + 0],
					k_data[v_width*y+x + 1],
					k_data[v_width*y+x + 2],
					k_data[v_width*y+x + 3]
			);
		}
	}
}

image_u32_t *im_from_vect(static std::vector<uint8_t> k_data) {
	image_u32_t *im = image_u32_create(v_width, v_height);
	update_im_from_vect(k_data, im);
	return im;
};


