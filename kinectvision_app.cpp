// General Includes
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>

// LCM
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

// Local Includes
#include "vision_state.h"
#include "eecs467_util.h"
#include "vision_gui.h"
#include "body.h"
#include "kinect_handle.h"

//Kinect
#include <libfreenect.hpp>

static state_t * global_state;
Freenect::Freenect freenect;

static void terminal_signal_handler(int signum)
{
	switch (signum)
	{
		case SIGINT:
		case SIGQUIT:
			//pthread_mutex_lock(&global_state->running_mutex);
			//printf("setting running to 0\n");
			global_state->running = 0;
			//pthread_mutex_unlock(&global_state->running_mutex);
			break;
		default:
			break;
	}
}


void* lcm_handle_loop(void *data) {
	state_t *state = (state_t*) data;

	// ..._subscribe(...)

	while (state->running) {
		// Set up the LCM file descriptor for waiting. This lets us monitor it
		// until somethign is "ready" to happen. In this case, we are ready to
		// receive a message.
		lcm_handle(state->lcm);
	}

	//clean up
	// ..._unsubscribe(...)

	return NULL;
}

void kinect_init(state_t* state) {
	//Initialize kinect
	state->kinect = &freenect.createDevice<MyFreenectDevice>(0);
	state->kinect->startVideo();
	state->kinect->setDepthFormat(FREENECT_DEPTH_REGISTERED);
	//state->kinect->setDepthFormat(FREENECT_DEPTH_MM);
	state->kinect->startDepth();
}

void kinect_destroy(state_t* state) {
	state->kinect->stopVideo();
	state->kinect->stopDepth();
}

void update_kinect(state_t* state) {
	static std::vector<uint8_t> depth(640*480*4);
	static std::vector<uint8_t> rgb(640*480*4);

	pthread_mutex_lock(&state->kinect_mutex);
	{
		state->kinect->updateState();
		state->kinect->getDepth(depth);
		state->kinect->getRGB(rgb);
		update_im_from_vect(rgb, state->im);
		update_im_from_vect(depth, state->depth);
		//printf("Dist: %x\n",state->depth->buf[state->depth->stride*240 + 320]);
	}
	pthread_mutex_unlock(&state->kinect_mutex);
}

void kinect_process(state_t* state){
	update_kinect(state);
	pthread_mutex_lock(&state->kinect_mutex);
	{
		//Do cool processing
	}
	pthread_mutex_unlock(&state->kinect_mutex);
}

void * kinect_analyze(void * data){
	state_t * state = (state_t *) data;
	kinect_init(state);

	while(state->running){
		kinect_process(state);
		usleep(10000);
	}

	//camera_destroy(state);
	return NULL;
}


int main(int argc, char ** argv)
{
	eecs467_init(argc, argv);

	state_t * state = (state_t*) calloc(1, sizeof(state_t));
	global_state = state;
	state->gopt = getopt_create();
	state->app.display_finished = display_finished;
	state->app.display_started = display_started;
	state->app.impl = state;

	state->running = 1;

	lcm_t * lcm = lcm_create (NULL);
	state->lcm = lcm;

	state->im    = image_u32_create(640, 480);
	state->depth = image_u32_create(640, 480);

	//signal(SIGINT, terminal_signal_handler);

	pthread_mutex_init(&state->layer_mutex, NULL);
	pthread_mutex_init(&state->lcm_mutex, NULL);
	pthread_mutex_init(&state->running_mutex, NULL);
	pthread_mutex_init(&state->kinect_mutex, NULL);

	state->layer_map = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_uint64_hash, zhash_uint64_equals);

	getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help");
	//getopt_add_bool(state->gopt, 'v', "verbose", 0, "Show extra debugging output");
	//getopt_add_int (state->gopt, 'l', "limitKBs", "-1", "Remote display bandwidth limit. < 0: unlimited.");
	//getopt_add_double (state->gopt, 'd', "decimate", "0", "Decimate image by this amount before showing in vx");

	if (!getopt_parse(state->gopt, argc, argv, 0) ||
		getopt_get_bool(state->gopt,"help")) {
		getopt_do_usage(state->gopt);
		exit(-1);
	}

	pthread_create(&state->lcm_handle_thread, NULL, lcm_handle_loop, state);
	pthread_create(&state->kinect_thread, NULL, kinect_analyze, state);
	gui_create(state);
	printf("after gui_create\n");

	// clean up
	vx_global_destroy();
    getopt_destroy(state->gopt);

    printf("Exited Cleanly!\n");
    return 0;
}
