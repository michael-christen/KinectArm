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
#include "joint.h"
#include "config_space.h"
#include "rexarm.h"
#include "bounding_box.h"

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
	double angles[NUM_SERVOS];

	for (i = 0; i < msg->len; i++) {
		angles[i] = msg->statuses[i].position_radians;
	}

	state->arm->setCurAngles(angles);
}

static void skeleton_data_handler( const lcm_recv_buf_t *rbuf,
                           const char *channel,
                           const skeleton_joint_list_t *msg,
                           void *user) {
	state_t *state = (state_t*) user;
	double angles[NUM_SERVOS];
	state->arm->getTargetAngles(angles);

	/*for (int i = 0; i < 7; i++) {
		if (i == HEAD || i == RSHOULDER || i == RELBOW || i == RWRIST) {
			switch (i) {
				case HEAD:
					printf("HEAD - ");
				break;

				case RSHOULDER:
					printf("RSHOULDER - ");
				break;

				case RELBOW:
					printf("RELBOW - ");
				break;

				case RWRIST:
					printf("RWRIST - ");
				break;
			}

			printf("%d, %d, %d\n", msg->joints[i].x, msg->joints[i].y, msg->joints[i].z);
		}
	}*/

	state->body->processMsg(msg);

	joint_t lwrist = state->body->getJoint(LWRIST);
	joint_t rshoulder = state->body->getJoint(RSHOULDER);
	double adjX = lwrist.x - rshoulder.x;
	double adjY = -lwrist.y - rshoulder.y;
	double adjZ = lwrist.z - rshoulder.z;

	if (state->set_gripper_cb) {
		state->set_gripper_cb = 0;
		state->controlBoxes[GRIPPER]->setPosition(adjX/20, adjZ/20, adjY/20);
	}

	if (state->set_elbow_cb) {
		state->set_elbow_cb = 0;
		state->controlBoxes[ELBOW]->setPosition(adjX/20, adjZ/20, adjY/20);
	}

	if (state->set_left_rot_cb) {
		state->set_left_rot_cb = 0;
		state->controlBoxes[LEFT_ROT]->setPosition(adjX/20, adjZ/20, adjY/20);
	}

	if (state->set_right_rot_cb) {
		state->set_right_rot_cb = 0;
		state->controlBoxes[RIGHT_ROT]->setPosition(adjX/20, adjZ/20, adjY/20);
	}

	state->body->getServoAngles(angles, true);
	state->arm->setTargetAngles(angles, state->cfs);
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
	double angles[NUM_SERVOS];

	dynamixel_command_list_t cmds;
    cmds.len = NUM_SERVOS;
    cmds.commands = (dynamixel_command_t*) malloc(sizeof(dynamixel_command_t)*NUM_SERVOS);

    while (state->running) {
    	if (state->update_arm_cont) {
	    	state->arm->getTargetAngles(angles);
			for (int id = 0; id < NUM_SERVOS; id++) {
				cmds.commands[id].utime = utime_now();
				cmds.commands[id].position_radians = angles[id];
				cmds.commands[id].speed = 0.5;
				cmds.commands[id].max_torque = 0.7;
		    }

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
	state->arm = new RexArm();
	state->body = new Body();
	state->running = 1;
	state->set_gripper_cb = 0;
	state->set_elbow_cb = 0;
	state->set_left_rot_cb = 0;
	state->set_right_rot_cb = 0;

	for (int i = 0; i < NUM_CONTROL_BOXES; i++) {
		state->controlBoxes[i] = new BoundingBox();
		state->controlBoxes[i]->setDimensions(10, 10, 10);
	}

	lcm_t * lcm = lcm_create (NULL);
	state->lcm = lcm;
	
	BoundingBox floorBoard, base;

	floorBoard.setPosition(0, 0, 0);
	floorBoard.setDimensions(100, 100, 10);
	state->cfs.addBoundingBox(&floorBoard);
	base.setPosition(0, 0, 4);
	base.setDimensions(7, 7, 8);
	state->cfs.addBoundingBox(&base);

	//signal(SIGINT, terminal_signal_handler);

	pthread_mutex_init(&state->layer_mutex, NULL);
	pthread_mutex_init(&state->lcm_mutex, NULL);
	pthread_mutex_init(&state->running_mutex, NULL);

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
	delete state->arm;
	delete state->body;
	for (int i = 0; i < NUM_CONTROL_BOXES; i++) {
		delete state->controlBoxes[i];
	}
	vx_global_destroy();
    getopt_destroy(state->gopt);

    printf("Exited Cleanly!\n");
    return 0;
}


