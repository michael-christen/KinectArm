#ifndef __KINECT_HANDLE__H__
#define __KINECT_HANDLE__H__
#include "common/image_util.h"
#include "image_helper.h"
#include "Image.h"
#include "pixel.h"
#include "Gradient.h"
#include "vision_state.h"
#include "filter.h"
#include <vector>

//Kinect
//#include <libfreenect.hpp>
#include <libfreenect.h>
extern uint16_t t_gamma[2048];
extern bool got_rgb;
extern bool got_depth;
extern pthread_mutex_t gl_backbuf_mutex;
extern pthread_cond_t gl_frame_cond;
extern uint8_t *depth_mid, *depth_front;
extern uint8_t *rgb_back, *rgb_mid, *rgb_front;
//extern std::vector<double> d_transf;
extern Image<double> d_transf;


void update_im_from_vect(const std::vector<uint8_t> & k_data,
		image_u32_t *im); 

image_u32_t *im_from_vect(const std::vector<uint8_t> & k_data); 

void make_depth_viewable(image_u32_t *im);

uint32_t depthToIm(uint16_t depth, bool valid, Gradient gr, int i);
uint32_t depthToImMarkers(uint16_t depth, bool valid, Gradient gr, int i);
uint32_t videoToIm(uint32_t video, bool valid, Gradient gr, int i);
uint32_t videoToImMarkers(uint32_t video, bool valid, Gradient gr, int i);
double   videoToGrad(uint32_t px, bool valid);
double   depthToGrad(uint16_t depth, bool valid);
double  d_map_to_grad(double dist, bool valid); 
double  d_map_v_grad(double dist, bool valid); 

uint16_t get_px_depth(uint32_t px);

struct DepthPoint {
	uint16_t mm;
	int x, y; 
};

void get_depth(std::vector<uint16_t> & depth);
void get_rgb(std::vector<uint32_t> & rgb);

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp);
void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp);


/*
	MyFreenectDevice(freenect_context *_ctx, int _index)
		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false)
	
	void VideoCallback(void* _rgb, uint32_t timestamp) {
		Mutex::ScopedLock lock(m_rgb_mutex);
		uint8_t* rgb = static_cast<uint8_t*>(_rgb);
		std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
		m_new_rgb_frame = true;
	};
	void DepthCallback(void* _depth, uint32_t timestamp) {
		Mutex::ScopedLock lock(m_depth_mutex);
		uint16_t* depth = static_cast<uint16_t*>(_depth);
		for( unsigned int i = 0 ; i < 640*480 ; i++) {
			int pval = m_gamma[depth[i]];
			m_buffer_depth[3*i+0] = pval & 0xff;
			m_buffer_depth[3*i+1] = (pval & 0xff00) >> 8;
			m_buffer_depth[3*i+2] = 255;
		}
		m_new_depth_frame = true;
	}
	bool getRGB(std::vector<uint32_t> &buffer) {
		Mutex::ScopedLock lock(m_rgb_mutex);
		if (!m_new_rgb_frame)
			return false;
		int v_width = 640;
		int v_height= 480;
		for(int y = 0; y < v_height; ++y) {
			for(int x = 0; x < v_width; ++x) {
				buffer[y*v_width+x] = get_px(
						m_buffer_video[3*v_width*y+3*x + 0],
						m_buffer_video[3*v_width*y+3*x + 1],
						m_buffer_video[3*v_width*y+3*x + 2],
						0xff
						);
			}
		}
		m_new_rgb_frame = false;
		return true;
	}

	bool getDepth(std::vector<uint16_t> &buffer) {
		Mutex::ScopedLock lock(m_depth_mutex);
		int v_width = 640;
		int v_height= 480;
		if (!m_new_depth_frame)
			return false;
		//buffer.swap(m_buffer_depth);
		for(int y = 0; y < v_height; ++y) {
			for(int x = 0; x < v_width; ++x) {
				buffer[v_width*y + x] = (
						m_buffer_depth[3*v_width*y+3*x + 0] | 
						m_buffer_depth[3*v_width*y+3*x + 1] << 8
						);
			}
		}
		m_new_depth_frame = false;
		return true;
	}

private:
	std::vector<uint8_t> m_buffer_depth;
	std::vector<uint8_t> m_buffer_video;
	std::vector<uint16_t> m_gamma;
	Mutex m_rgb_mutex;
	Mutex m_depth_mutex;
	bool m_new_rgb_frame;
	bool m_new_depth_frame;
};
*/
#endif
