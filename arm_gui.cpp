//////////////
// INCLUDES
//////////////

// C Libraries
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm.h>
#include <signal.h>

// VX
#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"
#include "vx/vx_types.h"
#include "vx/vx_event.h"
#include "vx/vx_event_handler.h"
#include "vx/default_camera_mgr.h"

// EECS 467 Libraries
#include "common/getopt.h"
#include "common/image_util.h"
#include "common/timestamp.h"
#include "common/matd.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

// Local Includes
#include "arm_gui.h"
#include "arm_state.h"
#include "eecs467_util.h"
#include "body_utility.h"

int displayCount;

double arm_segment_length[5] = {11.5, 10, 10, 9.5, 8};
double arm_segment_width = 5;
double arm_segment_depth = 2.5;

void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
	state_t *state = (state_t*) pl->impl;
	int i;
	int updateServoAngles = 0;
	double angles[NUM_SERVOS];
    if (!strcmp("s0", name)) {
		state->gui_servo_angles[0] = pg_gd(pg, name);
    } else if (!strcmp("s1", name)) {
    	state->gui_servo_angles[1] = pg_gd(pg, name);
    } else if (!strcmp("s2", name)) {
       	state->gui_servo_angles[2] = pg_gd(pg, name);
    } else if (!strcmp("s3", name)) {
    	state->gui_servo_angles[3] = pg_gd(pg, name);
    } else if (!strcmp("s4", name)) {
    	state->gui_servo_angles[4] = pg_gd(pg, name);
    } else if (!strcmp("s5", name)) {
    	state->gui_servo_angles[5] = pg_gd(pg, name);
    } else if (!strcmp("s6", name)) {
    	state->ds->setDSF(pg_gd(pg, name));
    } else if (!strcmp("s7", name)) {
    	state->ds->setTSF(pg_gd(pg, name));
    } else if (!strcmp("but1", name)) {
    	updateServoAngles = 1;
    } else if (!strcmp("but2", name)) {
    	if (!state->update_arm_cont) {
	    	for (i = 0; i < NUM_SERVOS; i++) {
				angles[i] = 0.0;
			}
			state->arm->setTargetAngles(angles, state->cfs);
	    } else {
    		printf("Uncheck \"Update Arm Continuously\" first\n");
	    }
    } else if (!strcmp("but7", name)) {
    	pthread_mutex_lock(&state->fsm_mutex);
    	state->FSM_next_state = FSM_NONE;
    	pthread_mutex_unlock(&state->fsm_mutex);
    } else if (!strcmp("but8", name)) {
    	pthread_mutex_lock(&state->fsm_mutex);
    	state->FSM_next_state = FSM_ARM;
    	pthread_mutex_unlock(&state->fsm_mutex);
    } else if (!strcmp("but9", name)) {
    	pthread_mutex_lock(&state->fsm_mutex);
    	state->FSM_next_state = FSM_WRIST;
    	pthread_mutex_unlock(&state->fsm_mutex);
    } else if (!strcmp("but10", name)) {
    	pthread_mutex_lock(&state->fsm_mutex);
    	state->FSM_next_state = FSM_GRIP;
    	pthread_mutex_unlock(&state->fsm_mutex);
    } else if (!strcmp("but11", name)) {
    	pthread_mutex_lock(&state->fsm_mutex);
    	state->FSM_next_state = FSM_ROTATE;
    	pthread_mutex_unlock(&state->fsm_mutex);
    } else if (!strcmp("butcb", name)) {
    	state->set_cbs = true;
    } else if (!strcmp("cb1", name)) {
        state->update_arm_cont = pg_gb(pg, name);
    } else if (!strcmp("cb2", name)) {
        state->close_gripper = pg_gb(pg, name);
    }

    /*if (state->update_arm_cont || updateServoAngles) {
		state->arm->setTargetAngles(state->gui_servo_angles, state->cfs);
    }*/
}


void display_finished(vx_application_t * app, vx_display_t * disp)
{
	//printf("Top of display finished\n");
	uint64_t i;
	vx_layer_t *value;

	state_t * state = (state_t*) app->impl;

	//printf("disp end: %d\n", disp);

	pthread_mutex_lock(&state->layer_mutex);

	for (i = 0; i < NUM_LAYERS; i++) {
		layer_data_t *layerData = &(state->layers[i]);
		if (layerData->enable == 1) {
			uint64_t key = ((uint64_t) disp) * (i + 1);
			//printf("disp: %zu, key %zu\n", ((uint64_t) disp), key);
			zhash_remove(state->layer_map, &key, NULL, &value);
			//printf("layer being destroyed!\n");
			vx_layer_destroy(value);
		}
	}

	pthread_mutex_unlock(&state->layer_mutex);

	displayCount--;

	//printf("hash table size after remove: %d\n", zhash_size(state->layer_map));
}

void display_started(vx_application_t * app, vx_display_t * disp)
{
	uint64_t i;

	state_t * state = (state_t*) app->impl;

	//printf("disp start: %d\n", disp);

	for (i = 0; i < NUM_LAYERS; i++) {
		layer_data_t *layerData = &(state->layers[i]);
		if (layerData->enable == 1) {
			uint64_t key = ((uint64_t) disp) * (i + 1);
			printf("Layer being created\n");
			if (layerData->init(state, layerData) == 0) {
				printf("Failed to init layer: %s\n", layerData->name);
				return;
			}
			vx_layer_t * layer = vx_layer_create(layerData->world);
			printf("disp: %zu, key %zu\n", ((uint64_t) disp), key);
			layerData->layer = layer;
			vx_layer_set_display(layer, disp);

			pthread_mutex_lock(&state->layer_mutex);
			// store a reference to the world and layer that we associate with each vx_display_t
			zhash_put(state->layer_map, &key, &layer, NULL, NULL);
			pthread_mutex_unlock(&state->layer_mutex);

			layerData->displayInit(state, layerData);
		}
	}
	displayCount++;
	printf("hash table size after insert: %d\n", zhash_size(state->layer_map));
}

int initArmsLayer(state_t *state, layer_data_t *layerData) {
	layerData->world = vx_world_create();
	return 1;
}

int displayInitArmsLayer(state_t *state, layer_data_t *layerData) {
	const float eye[3] = {0, -100, 50};
	const float lookat[3] = {0, 0, 25};
	const float up[3] = {0, 0, 1};

	vx_layer_set_viewport_rel(layerData->layer, layerData->position);
	vx_layer_camera_lookat(layerData->layer, eye, lookat, up, 1);
	return 1;
}

int renderArmsLayer(state_t *state, layer_data_t *layerData) {
	//Draw Grid
	vx_buffer_t *gridBuff = vx_world_get_buffer(layerData->world, "grid");
	vx_buffer_add_back(gridBuff, vxo_grid());

	//Draw Axes
	float axes[12] = {-1000, 0, 0, 0, 0, 0, 0, -1000, 0, 0, 1000, 0};
	vx_resc_t *verts = vx_resc_copyf(axes, 12);
	vx_buffer_add_back(gridBuff, vxo_lines(verts, 4, GL_LINES, vxo_points_style(vx_black, 2.0f)));

	float posAxes[6] = {0, 0, 0, 1000, 0, 0};
	verts = vx_resc_copyf(posAxes, 6);
	vx_buffer_add_back(gridBuff, vxo_lines(verts, 2, GL_LINES, vxo_points_style(vx_green, 2.0f)));
	
	//Draw Config Space
	vx_buffer_t *cfsBuff = vx_world_get_buffer(layerData->world, "cfs");
	state->cfs.draw(cfsBuff, vx_red);

	//Draw Arms
	vx_buffer_t *armBuff = vx_world_get_buffer(layerData->world, "arm");
	state->arm->drawTargetState(armBuff, vx_yellow, state->FSM_state);
	state->arm->drawCurState(armBuff, vx_blue, state->FSM_state);
	
	//Swap buffers
	vx_buffer_swap(gridBuff);
    vx_buffer_swap(cfsBuff);
	vx_buffer_swap(armBuff);
	return 1;
}

int destroyArmsLayer(state_t *state, layer_data_t *layerData) {
	vx_world_destroy(layerData->world);
	return 1;
}

int initSkeletonLayer(state_t *state, layer_data_t *layerData) {
	layerData->world = vx_world_create();
	return 1;
}

int displayInitSkeletonLayer(state_t *state, layer_data_t *layerData) {
	const float eye[3] = {0, 125, 50};
	const float lookat[3] = {0, 0, 25};
	const float up[3] = {0, 0, 1};

	vx_layer_set_viewport_rel(layerData->layer, layerData->position);
	vx_layer_camera_lookat(layerData->layer, eye, lookat, up, 1);
	return 1;
}

int renderSkeletonLayer(state_t *state, layer_data_t *layerData) {
	//Draw Grid
	vx_buffer_t *gridBuff = vx_world_get_buffer(layerData->world, "grid");
	//vx_buffer_add_back(gridBuff, vxo_grid());

	//Draw Axes
	float axes[12] = {-1000, 0, 0, 0, 0, 0, 0, -1000, 0, 0, 1000, 0};
	vx_resc_t *verts = vx_resc_copyf(axes, 12);
	vx_buffer_add_back(gridBuff, vxo_lines(verts, 4, GL_LINES, vxo_points_style(vx_black, 2.0f)));

	float posAxes[6] = {0, 0, 0, 1000, 0, 0};
	verts = vx_resc_copyf(posAxes, 6);
	vx_buffer_add_back(gridBuff, vxo_lines(verts, 2, GL_LINES, vxo_points_style(vx_green, 2.0f)));
	
	//Draw Control Boxes
	vx_buffer_t *cbBuff = vx_world_get_buffer(layerData->world, "cb");

	//Draw Skeleton
	vx_buffer_t *skeletonBuff = vx_world_get_buffer(layerData->world, "skeleton");
	body_draw(state->body, skeletonBuff);

	
	const float* color;
	
	for (int i = 0; i < NUM_CONTROL_BOXES; i++) {
		if (state->controlBoxSelected[i]) {
			color = vx_white;
		} else {
			color = state->controlBoxColor[i];
		}
		state->controlBoxes[i]->draw(cbBuff, color);
	}

	//Swap buffers
	vx_buffer_swap(gridBuff);
	vx_buffer_swap(cbBuff);
	vx_buffer_swap(skeletonBuff);
	return 1;
}

int destroySkeletonLayer(state_t *state, layer_data_t *layerData) {
	vx_world_destroy(layerData->world);
	return 1;
}

void* renderLayers(void *data) {
	state_t * state = (state_t*) data;
	int i;
	printf("Entering render loop\n");
	// Render Loop
	while(state->running) {
		//printf("running!\n");
		for (i = 0; i < NUM_LAYERS; i++) {
			layer_data_t *layer = &(state->layers[i]);
			//printf("layer %d enable %d\n", i, layer->enable);
			pthread_mutex_lock(&state->running_mutex);
			if (state->running) {
				if (layer->enable == 1 && displayCount > 0) {
					if (layer->render(state, layer) == 0) {
						printf("Failed to render layer: %s\n", layer->name);
						return NULL;
					}
				}
			} else {
				break;
			}
			pthread_mutex_unlock(&state->running_mutex);
		}
		// 30 FPS
		usleep(33333);
		//usleep(1000000);
	}
	printf("Entering layer destroy\n");
	// Destroy/Clean up
	for (i = 0; i < NUM_LAYERS; i++) {
		layer_data_t *layer = &(state->layers[i]);
		if (layer->enable == 1) {
			if (layer->destroy(state, layer) == 0) {
				printf("Failed to destroy layer: %s\n", layer->name);
				return NULL;
			}
		}
	}
	printf("returning\n");

	return NULL;
}

// Main Pthread  GUI funtion
void gui_create(state_t *state) {
	pthread_t render_thread;

	vx_remote_display_source_attr_t remote_attr;
	vx_remote_display_source_attr_init(&remote_attr);
	remote_attr.advertise_name = "Kinect Arm";


	// Init layer data structs
	state->layers[0].enable = 1;
	state->layers[0].name = "Arms";
	state->layers[0].position[0] = 0.5f;
	state->layers[0].position[1] = 0.0f;
	state->layers[0].position[2] = 0.5f;
	state->layers[0].position[3] = 1.0f;
	state->layers[0].init = initArmsLayer;
	state->layers[0].displayInit = displayInitArmsLayer;
	state->layers[0].render = renderArmsLayer;
	state->layers[0].destroy = destroyArmsLayer;

	state->layers[1].enable = 1;
	state->layers[1].name = "Skeleton";
	state->layers[1].position[0] = 0.0f;
	state->layers[1].position[1] = 0.0f;
	state->layers[1].position[2] = 0.5f;
	state->layers[1].position[3] = 1.0f;
	state->layers[1].init = initSkeletonLayer;
	state->layers[1].displayInit = displayInitSkeletonLayer;
	state->layers[1].render = renderSkeletonLayer;
	state->layers[1].destroy = destroySkeletonLayer;

	vx_remote_display_source_t * remote = vx_remote_display_source_create_attr(&state->app, &remote_attr);

	// Handles layer init, rendering, and destruction
	parameter_gui_t *pg = pg_create();
    /*pg_add_double_slider(pg, "s0", "S0 (Shoulder Rotation)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s1", "S1 (Shoulder Bend)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s2", "S2 (Elbow)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s3", "S3 (Wrist Bend)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s4", "S4 (Wrist Rotation)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s5", "S5 (Gripper)", -M_PI, M_PI, 0);*/
    pg_add_check_boxes(pg, "cb1", "Send Arm Commands", state->update_arm_cont, "cb2", "Close Gripper", state->close_gripper, NULL);
    pg_add_double_slider(pg, "s6", "DSF", 0, 1, state->ds->getDSF());
    pg_add_double_slider(pg, "s7", "TSF", 0, 1, state->ds->getTSF());
    //pg_add_buttons(pg, "but1", "Update Arm", "but2", "Go To Home", NULL);
    pg_add_buttons(pg, "butcb", "Set Control Boxes", NULL);

	pg_add_buttons(pg, "but7", "FSM-NONE",
    					"but8", "FSM-ARM",
    					"but9", "FSM-WRIST",
    					"but10", "FSM-GRIP",
    					"but11", "FSM-ROTATE",
    					NULL);

    parameter_listener_t *my_listener = (parameter_listener_t*) calloc(1,sizeof(parameter_listener_t*));
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener(pg, my_listener);

	pthread_create(&render_thread, NULL, renderLayers, state);
	eecs467_gui_run(&state->app, pg, 800, 600);
	printf("after gui run***\n");
	pthread_mutex_lock(&state->running_mutex);
	printf("setting runnin to 0\n");
	state->running = 0;
	//pthread_join(&render_thread, NULL);
	//pthread_mutex_unlock(&state->running_mutex);
	vx_remote_display_source_destroy(remote);

	printf("end of gui thread (after destroy)\n");
}
