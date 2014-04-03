// General Includes
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>

// LCM
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"
#include "skeleton_joint_t.h"
#include "skeleton_joint_list_t.h"


// Local Includes
#include "arm_state.h"
#include "eecs467_util.h"
#include "arm_gui.h"
#include "body.h"
#include "config_space.h"
#include "image_helper.h"

static int64_t utime_now()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

static state_t * global_state;
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

static void arm_status_handler( const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const dynamixel_status_list_t *msg,
                           void *user) {
	int i;
	state_t *state = (state_t*) user;

	for (i = 0; i < msg->len; i++) {
		state->current_servo_angles[i] = msg->statuses[i].position_radians;
	}
}

static void skeleton_data_handler( const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const skeleton_joint_list_t *msg,
                           void *user) {
	state_t *state = (state_t*) user;

	state->last_body = state->current_body;
	state->current_body = Body(msg);
	state->current_body.getServoAngles(state->target_servo_angles, true);
}

int angles_valid(double angles[]) {
	return 1;
}

void* lcm_handle_loop(void *data) {
	state_t *state = (state_t*) data;

	// ..._subscribe(...)
	dynamixel_status_list_t_subscription_t *arm_sub = dynamixel_status_list_t_subscribe(state->lcm,
                                      ARM_STATUS_CHANNEL,
                                      arm_status_handler,
                                      state);

	skeleton_joint_list_t_subscription_t *skeleton_sub = skeleton_joint_list_t_subscribe(state->lcm,
                                      SKELETON_DATA_CHANNEL,
                                      skeleton_data_handler,
                                      state);

	while (state->running) {
		// Set up the LCM file descriptor for waiting. This lets us monitor it
		// until somethign is "ready" to happen. In this case, we are ready to
		// receive a message.
		lcm_handle(state->lcm);
	}

	//clean up
	// ..._unsubscribe(...)
	dynamixel_status_list_t_unsubscribe(state->lcm, arm_sub);
	skeleton_joint_list_t_unsubscribe(state->lcm, skeleton_sub);

	return NULL;
}

void* arm_commander(void *data) {
	int hz = 30;
	int valid_angles;
	state_t *state = (state_t*) data;

	dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = (dynamixel_command_t*) malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

    while (state->running) {
    	pthread_mutex_lock(&state->servo_angles_mutex);
    	valid_angles = angles_valid(state->target_servo_angles);
    	if (valid_angles) {
	    		for (int id = 0; id < NUM_SERVOS; id++) {
				cmds.commands[id].utime = utime_now();
				cmds.commands[id].position_radians = state->target_servo_angles[id];
				cmds.commands[id].speed = 0.5;
				cmds.commands[id].max_torque = 0.7;
		    }
    	}
	    pthread_mutex_unlock(&state->servo_angles_mutex);

	    if (valid_angles) {
	    	pthread_mutex_lock(&state->lcm_mutex);
		    dynamixel_command_list_t_publish(state->lcm, ARM_COMMAND_CHANNEL, &cmds);
		    pthread_mutex_unlock(&state->lcm_mutex);
	    }
	   
    	usleep(1000000/hz);
	}

	free(cmds.commands);

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
	state->update_arm_cont = 0;
	state->update_arm = 0;

	state->running = 1;

	lcm_t * lcm = lcm_create (NULL);
	state->lcm = lcm;

	//signal(SIGINT, terminal_signal_handler);

	pthread_mutex_init(&state->layer_mutex, NULL);
	pthread_mutex_init(&state->lcm_mutex, NULL);
	pthread_mutex_init(&state->running_mutex, NULL);
	pthread_mutex_init(&state->servo_angles_mutex, NULL);

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
	//pthread_create(&state->gui_thread,  NULL, gui_create, state);
	pthread_create(&state->arm_commander_thread, NULL, arm_commander, state);
	//pthread_join(state->gui_thread, NULL);
	gui_create(state);
	printf("after gui_create\n");

	// clean up
	vx_global_destroy();
    getopt_destroy(state->gopt);

    printf("Exited Cleanly!\n");
    return 0;
}


