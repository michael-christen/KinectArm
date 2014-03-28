#ifndef STATE_H
#define STATE_H

//////////////
// INCLUDES
//////////////

// C Libraries
#include <pthread.h>
#include "time.h"
#include "vx/vx.h"
#include "common/image_util.h"
#include <vector>
#include <stdlib.h>
#include <math.h>

// EECS 467 Libraries
#include "common/getopt.h"

// LCM
#include <lcm/lcm.h>

// Local Includes
//Kinect
#include <libfreenect.hpp>

//////////////
// CONSTANTS
//////////////
#define NUM_LAYERS 1
#define NUM_SERVOS 6

#define ARM_STATUS_CHANNEL "ARM_STATUS"
#define ARM_COMMAND_CHANNEL "ARM_COMMAND"

//////////////
// STRUCTS
//////////////

typedef struct layer_data_t layer_data_t;
typedef struct state_t state_t;
typedef struct getopt_options_t getopt_options_t;


struct getopt_options_t {
    int verbose, no_video, limitKBs, autoCamera, mouseGuidance;
    double decimate;
};

struct layer_data_t {
    int enable;
    const char* name;
    vx_world_t *world;
    vx_layer_t *layer;
    float position[4];

    int (*init)(state_t *state, layer_data_t *layerData);
    int (*displayInit)(state_t *state, layer_data_t *layerData);
    int (*render)(state_t *state, layer_data_t *layerData);
    int (*destroy)(state_t *state, layer_data_t *layerData);
};

class Mutex {
public:
	Mutex() {
		pthread_mutex_init( &m_mutex, NULL );
	}
	void lock() {
		pthread_mutex_lock( &m_mutex );
	}
	void unlock() {
		pthread_mutex_unlock( &m_mutex );
	}

	class ScopedLock
	{
		Mutex & _mutex;
	public:
		ScopedLock(Mutex & mutex)
			: _mutex(mutex)
		{
			_mutex.lock();
		}
		~ScopedLock()
		{
			_mutex.unlock();
		}
	};
private:
	pthread_mutex_t m_mutex;
};

/* thanks to Yoda---- from IRC */
class MyFreenectDevice : public Freenect::FreenectDevice {
public:
	MyFreenectDevice(freenect_context *_ctx, int _index)
		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false)
	{
		for( unsigned int i = 0 ; i < 2048 ; i++) {
			float v = i/2048.0;
			v = pow(v, 3)* 6;
			m_gamma[i] = v*6*256;
		}
	}
	//~MyFreenectDevice(){}
	// Do not call directly even in child
	void VideoCallback(void* _rgb, uint32_t timestamp) {
		Mutex::ScopedLock lock(m_rgb_mutex);
		uint8_t* rgb = static_cast<uint8_t*>(_rgb);
		std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
		m_new_rgb_frame = true;
	};
	// Do not call directly even in child
	void DepthCallback(void* _depth, uint32_t timestamp) {
		Mutex::ScopedLock lock(m_depth_mutex);
		uint16_t* depth = static_cast<uint16_t*>(_depth);
		for( unsigned int i = 0 ; i < 640*480 ; i++) {
			int pval = m_gamma[depth[i]];
			int lb = pval & 0xff;
			switch (pval>>8) {
			case 0:
				m_buffer_depth[3*i+0] = 255;
				m_buffer_depth[3*i+1] = 255-lb;
				m_buffer_depth[3*i+2] = 255-lb;
				break;
			case 1:
				m_buffer_depth[3*i+0] = 255;
				m_buffer_depth[3*i+1] = lb;
				m_buffer_depth[3*i+2] = 0;
				break;
			case 2:
				m_buffer_depth[3*i+0] = 255-lb;
				m_buffer_depth[3*i+1] = 255;
				m_buffer_depth[3*i+2] = 0;
				break;
			case 3:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 255;
				m_buffer_depth[3*i+2] = lb;
				break;
			case 4:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 255-lb;
				m_buffer_depth[3*i+2] = 255;
				break;
			case 5:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 0;
				m_buffer_depth[3*i+2] = 255-lb;
				break;
			default:
				m_buffer_depth[3*i+0] = 0;
				m_buffer_depth[3*i+1] = 0;
				m_buffer_depth[3*i+2] = 0;
				break;
			}
		}
		m_new_depth_frame = true;
	}
	bool getRGB(std::vector<uint8_t> &buffer) {
		Mutex::ScopedLock lock(m_rgb_mutex);
		if (!m_new_rgb_frame)
			return false;
		buffer.swap(m_buffer_video);
		m_new_rgb_frame = false;
		return true;
	}

	bool getDepth(std::vector<uint8_t> &buffer) {
		Mutex::ScopedLock lock(m_depth_mutex);
		if (!m_new_depth_frame)
			return false;
		buffer.swap(m_buffer_depth);
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


struct state_t {
    getopt_options_t  getopt_options;
    vx_application_t app;
    vx_event_handler_t veh;

    volatile int running;
    int displayStarted, displayFinished;

    getopt_t * gopt;	

    lcm_t * lcm;
    pthread_mutex_t lcm_mutex;

    pthread_t lcm_handle_thread;
    pthread_mutex_t layer_mutex;
    pthread_mutex_t running_mutex;
    pthread_t gui_thread;

    int layerCount;
    layer_data_t layers[NUM_LAYERS];

    zhash_t *layer_map; // <display, layer>

	image_u32_t * im;
	image_u32_t * depth;
    pthread_t kinect_thread;
    pthread_mutex_t kinect_mutex;

	MyFreenectDevice * kinect;
};


void* arm_commander(void *data);
#endif
