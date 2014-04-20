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
#include <libfreenect.h>
#include "kinect_handle.h"

// Local Includes
#include "Image.h"
#include "Line.h"
#include "joint.h"
#include "body.h"
//////////////
// CONSTANTS
//////////////
#define NUM_LAYERS 2
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
    int use_markers;
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


struct state_t {
    getopt_options_t  getopt_options;
    vx_application_t app;
    vx_event_handler_t veh;

    volatile int running;
    int displayStarted, displayFinished;

    getopt_t * gopt;	

    lcm_t * lcm;
    pthread_mutex_t lcm_mutex;

    vx_mouse_event_t last_mouse;

    int init_last_mouse, mouseDownSet;
    double mouseDownX, mouseDownY;

    pthread_t lcm_handle_thread;
    pthread_mutex_t layer_mutex;
    pthread_mutex_t running_mutex;
    pthread_t gui_thread;

    int layerCount;
    layer_data_t layers[NUM_LAYERS];

    zhash_t *layer_map; // <display, layer>

	//Image<uint32_t> im;
	Image<uint16_t> depth;
	std::vector<line_t> depth_lines;
	Image<uint32_t> im;
	std::vector<line_t> im_lines;
	std::vector<int> pts;
	/*
	image_u32_t * depth;
	*/
    pthread_t kinect_thread;
    pthread_t kinect_event_thread;
    pthread_mutex_t kinect_mutex;

	//MyFreenectDevice * kinect;
	freenect_context *f_ctx;
	freenect_device  *f_dev;
	freenect_video_format current_format;

    Body body;
    joint_t joints[NUM_JOINTS];

    int send_data, set_hand_dist, set_open_hand, set_closed_hand;

    bool close_left_gripper, close_right_gripper;
};


void* arm_commander(void *data);
#endif
