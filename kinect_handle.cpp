/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

      * File Name : kinect_handle.cpp

         * Purpose :

	    * Creation Date : 27-03-2014

	       * Last Modified : Fri 11 Apr 2014 10:52:05 PM EDT

	          * Created By : Michael Christen

		     _._._._._._._._._._._._._._._._._._._._._.*/
#include "kinect_handle.h"

void get_depth(std::vector<uint16_t> & depth) {
	int v_width = 640;
	int v_height= 480;
	pthread_mutex_lock(&gl_backbuf_mutex);
	{
		/*
		if (!got_depth)
			return;
			*/
		//buffer.swap(m_buffer_depth);
		for(int y = 0; y < v_height; ++y) {
			for(int x = 0; x < v_width; ++x) {
				depth[v_width*y + x] = (
						depth_mid[3*v_width*y+3*x + 0] | 
						depth_mid[3*v_width*y+3*x + 1] << 8
						);
			}
		}
		got_depth = false;
	}
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void get_rgb(std::vector<uint32_t> &rgb) {
	int v_width = 640;
	int v_height= 480;
	pthread_mutex_lock(&gl_backbuf_mutex);
	{
		for(int y = 0; y < v_height; ++y) {
			for(int x = 0; x < v_width; ++x) {
				rgb[y*v_width+x] = get_px(
						rgb_mid[3*v_width*y+3*x + 0],
						rgb_mid[3*v_width*y+3*x + 1],
						rgb_mid[3*v_width*y+3*x + 2],
						0xff
						);
			}
		}
	}
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	pthread_mutex_lock(&gl_backbuf_mutex);

	// swap buffers
	assert (rgb_back == rgb);
	rgb_back = rgb_mid;
	freenect_set_video_buffer(dev, rgb_back);
	rgb_mid = (uint8_t*)rgb;

	got_rgb = true;
	//pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	int i;
	uint16_t *depth = (uint16_t*)v_depth;

	pthread_mutex_lock(&gl_backbuf_mutex);
	for (i=0; i<640*480; i++) {
		//printf("yo:%d\n",i);
		//printf("ho:%d\n",depth[i]);
		int pval;
		if(depth[i] >= 2048) {
			pval = 0;
		} else {
			pval = t_gamma[depth[i]];
		}
		depth_mid[3*i+0] = pval & 0xff;
		depth_mid[3*i+1] = (pval & 0xff00) >> 8;
		depth_mid[3*i+2] = 255;
	}
	got_depth = true;
	//pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}



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

uint32_t depthToIm(uint16_t depth, bool valid, Gradient gr, int id) {
	/*
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
	*/
	uint8_t  scaled_down;
	double   tmp;
	/*
	if(!depth) {
		return 0xFFFFFFFF;
	}
	*/
	//double val = gr.mag() > 50 ? 1 : 0;
	/*
	if(d_transf[id]) { 
		printf("D:%f\n",d_transf[id]);
	}
	*/
	//uint32_t px = HSVtoRGB((gr.angle() + M_PI)/M_PI*180,0.5,gr.mag()/255.0);
	double val = d_transf[id] / 100;
	uint32_t px = HSVtoRGB(90,0.5,val);
	//uint32_t px = HSVtoRGB((gr.angle() + M_PI)/M_PI*180,0.5,val);
	return px;
	//if(depth > 200 && depth < 2000 && gr.mag() > 0.01)
	/*
	if(true || gr.mag() > 0.01 && valid)
		return px;
	else 
		return 0xFF000000;
	tmp *= 0xff;
	scaled_down = 0xff - (uint8_t)tmp;
	if(valid) {
		return get_px(scaled_down, scaled_down, scaled_down, 0xFF);
	}
	return 0xFF000000;
	*/
}

uint32_t videoToIm(uint32_t video, bool valid, Gradient gr, int id) {
	if(valid) {
		//return dist_to_grey(gr.mag());
		double h,s,v;
		RGBtoHSV(video,&h,&s,&v);
		//if(s) printf("s:%f\n",s);
		uint32_t px = HSVtoRGB((gr.angle() + M_PI)/M_PI*180,s,gr.mag()/255.0);
		uint8_t r,g,b;
		r = get_red(px);
		g = get_green(px);
		b = get_blue(px);
		//printf("r:%x,g:%x,b:%x\n",r,g,b);
		/*
		if(!video) {
			return 0xFFFFFFFF;
		}
		*/
		return px;
		//return video;
	}
	return 0xFF000000;
}

double   videoToGrad(uint32_t px, bool valid) {
	double h,s,v;
	RGBtoHSV(px,&h,&s,&v);
	//printf("px:%x -> V:%f\n",px,v);
	return 255.0*v;
	//return (double) (0xFFFFFF & px);
}
double   depthToGrad(uint16_t depth, bool valid) {
	/*
	uint16_t MAX_DEPTH_VAL = 0x1fff;
	double diff =  (double) depth / (MAX_DEPTH_VAL+0.0);
	*/
	return valid ? 255.0 : 0.0;
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
	

