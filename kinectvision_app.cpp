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
#include "kinect_handle.h"
#include "filter.h"
#include "Line.h"
#include "Image.h"
#include "blob_detection.h"

//Kinect
#include <libfreenect.hpp>

static state_t * global_state;
//Freenect::Freenect freenect;
uint16_t t_gamma[2048];
bool got_rgb = false;
bool got_depth = false;
pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
uint8_t *depth_mid, *depth_front;
uint8_t *rgb_back, *rgb_mid, *rgb_front;



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
	if(freenect_init(&state->f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		exit(1);
	}
	freenect_select_subdevices(state->f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
	int nr_devices = freenect_num_devices (state->f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	int user_device_number = 0;
	if (nr_devices < 1) {
		freenect_shutdown(state->f_ctx);
		exit(1);
	}

	if (freenect_open_device(state->f_ctx, &state->f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(state->f_ctx);
		exit(1);
	}
	for (int i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}
	//Initialize kinect
	depth_mid = (uint8_t*)malloc(640*480*3);
	depth_front = (uint8_t*)malloc(640*480*3);
	rgb_back = (uint8_t*)malloc(640*480*3);
	rgb_mid = (uint8_t*)malloc(640*480*3);
	rgb_front = (uint8_t*)malloc(640*480*3);


	freenect_set_tilt_degs(state->f_dev,10);
	freenect_set_led(state->f_dev,LED_RED);
	printf("setting up\n");
	freenect_set_depth_callback(state->f_dev, depth_cb);
	freenect_set_video_callback(state->f_dev, rgb_cb);
	freenect_set_video_mode(state->f_dev,
			freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM,state->current_format));
	freenect_set_depth_mode(state->f_dev, freenect_find_depth_mode(
				FREENECT_RESOLUTION_MEDIUM,
				FREENECT_DEPTH_REGISTERED));
	freenect_set_video_buffer(state->f_dev, rgb_back);
	freenect_set_led(state->f_dev,LED_GREEN);

	freenect_start_depth(state->f_dev);
	freenect_set_led(state->f_dev,LED_GREEN);
	freenect_start_video(state->f_dev);
	freenect_set_led(state->f_dev,LED_BLINK_RED_YELLOW);

	/*
	state->kinect = &freenect.createDevice<MyFreenectDevice>(0);
	state->kinect->startVideo();
	state->kinect->setDepthFormat(FREENECT_DEPTH_REGISTERED);
	//state->kinect->setDepthFormat(FREENECT_DEPTH_MM);
	state->kinect->startDepth();
	*/
}

void kinect_destroy(state_t* state) {
	/*
	state->kinect->stopVideo();
	state->kinect->stopDepth();
	*/
	freenect_stop_depth(state->f_dev);
	freenect_stop_video(state->f_dev);
	freenect_close_device(state->f_dev);
	freenect_shutdown(state->f_ctx);
}

//Must be called with a lock on kinect_mutex
//to maintain atomicity
void update_kinect(state_t* state) {
	static std::vector<uint16_t> depth(640*480);
	static std::vector<uint32_t> rgb(640*480);
	get_depth(depth);
	get_rgb(rgb);
	/*
	state->kinect->updateState();
	state->kinect->getDepth(depth);
	state->kinect->getRGB(rgb);
	*/
	//update_im_from_vect(rgb, state->im);
	state->im.update(rgb);
	state->depth.update(depth);
	//update_im_from_vect(depth, state->depth);
	//printf("Dist: %x\n",state->depth->buf[state->depth->stride*240 + 320]);
}

//std::vector<double> d_transf; 
Image<double> d_transf(640,480);
//Practically unimportant
#define MAX_VARIANCE 16000000
void kinect_process(state_t* state){
	pthread_mutex_lock(&state->kinect_mutex);
	{
		//Update
		update_kinect(state);
		//Do cool processing
		double prev_time = utime_now()/1000000.0;

		//Only look at those pixels which are in the foreground
		//of the depth field
		filter_front(state->depth);
		//Filter out image pixels which aren't in foreground
		//state->depth.copyValid(state->im.valid);
		//Compute the gradient of the entire image
		state->im.computeGradient(videoToGrad);
		blurGradient(state->im);
		/*
		state->depth.computeGradient(depthToGrad);
		blurGradient(state->depth);
		blurGradient(state->depth);
		blurGradient(state->depth);
		blurGradient(state->depth);
		*/
		printf("\n\nImage\n");
		std::vector<Blob<Gradient>> im_blobs = get_gradient_blobs(state->im);
		//std::vector<line_t> im_lines;
		state->im_lines.clear();
		for(size_t i = 0; i < im_blobs.size(); ++i) {
			line_t tmp_line = linear_regression(im_blobs[i]);
			if(tmp_line.variance <= MAX_VARIANCE) {
				state->im_lines.push_back(tmp_line);
			}
		}
		/*
		printf("\nDepth\n");
		std::vector<Blob<Gradient>> dp_blobs = get_gradient_blobs(state->depth);
		state->depth_lines.clear();
		for(size_t i = 0; i < dp_blobs.size(); ++i) {
			line_t tmp_line = linear_regression(dp_blobs[i]);
			if(tmp_line.variance <= MAX_VARIANCE) {
				state->depth_lines.push_back(tmp_line);
			}
		}
		*/
		printf("getting d_transf\n");
		get_dist_transform(d_transf, state->depth);
		//dtocs(d_transf, state->depth);
		printf("getting gradient\n");
		d_transf.computeGradient(d_map_to_grad);
		printf("getting threshold\n");
		minc_local_threshold(d_transf);
		printf("done with d_transf\n");
		blurGradient(d_transf);
		/*
		std::vector<line_t> dp_lines = hough_transform(d_transf);
		printf("Num_linos: %d\n",dp_lines.size());
		std::vector<Blob<Gradient>> dp_blobs = get_gradient_blobs(d_transf);
		state->depth_lines.clear();
		printf("NUM_BLOBS: %d\n",dp_blobs.size());
		for(size_t i = 0; i < dp_blobs.size(); ++i) {
			line_t tmp_line = linear_regression(dp_blobs[i]);
			if(tmp_line.variance <= MAX_VARIANCE) {
				state->depth_lines.push_back(tmp_line);
			}
		}
		state->depth_lines = dp_lines;
		*/

		/*
		double pink_hue = 328.0;
		double green_hue = 73.0;
		double yellow_hue = 50.0;
		double blue_hue   = 209.0;
		blob_detection(state->im, green_hue, 0xff039dfc,
				10, 200);
		blob_detection(state->im, blue_hue, 0xff030dfc,
				10, 200);
		blob_detection(state->im, pink_hue, 0xff030d0c,
				10, 200);
		blob_detection(state->im, yellow_hue, 0xff830dfc,
				10, 200);
				*/
		double cur_time = utime_now()/1000000.0;
		printf("TOTAL KINECT PROCESSING = %f\n",cur_time-prev_time);

	}
	pthread_mutex_unlock(&state->kinect_mutex);
}

void * kinect_analyze(void * data){
	state_t * state = (state_t *) data;

	while(state->running){
		//Process callbacks
		kinect_process(state);
		usleep(10000);
	}

	//camera_destroy(state);
	return NULL;
}
void * kinect_event(void * data){
	state_t * state = (state_t *) data;
	while(true) {
		if(state->f_ctx) {
			if(freenect_process_events(state->f_ctx) < 0) {
				break;
			}
		}
	}
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

	state->im    = Image<uint32_t>(640,480);
	state->depth = Image<uint16_t>(640,480);
	//d_transf     = Image<double>(640,480);
    state->current_format = FREENECT_VIDEO_RGB;
	//state->im    = image_u32_create(640, 480);
	/*
	state->depth = image_u32_create(640, 480);
	*/

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

	kinect_init(state);
	pthread_create(&state->lcm_handle_thread, NULL, lcm_handle_loop, state);
	pthread_create(&state->kinect_thread, NULL, kinect_analyze, state);
	pthread_create(&state->kinect_event_thread, NULL, kinect_event,state);
	gui_create(state);
	printf("after gui_create\n");

	// clean up
	vx_global_destroy();
    getopt_destroy(state->gopt);

    printf("Exited Cleanly!\n");
    return 0;
}


