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

int displayCount;

double arm_segment_length[5] = {11.5, 10, 10, 9.5, 8};
double arm_segment_width = 5;
double arm_segment_depth = 2.5;

void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
	state_t *state = (state_t*) pl->impl;
	int i;
	int updateServoAngles = 0;
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
    } else if (!strcmp("but1", name)) {
    	updateServoAngles = 1;
    } else if (!strcmp("but2", name)) {
    	if (!state->update_arm_cont) {
    		pthread_mutex_lock(&state->servo_angles_mutex);
	    	for (i = 0; i < NUM_SERVOS; i++) {
				state->target_servo_angles[i] = 0;
			}
			pthread_mutex_unlock(&state->servo_angles_mutex);
	    } else {
    		printf("Uncheck \"Update Arm Continuously\" first\n");
	    }
    } else if (!strcmp("cb1", name)) {
        state->update_arm_cont = pg_gb(pg, name);
    }

    if (state->update_arm_cont || updateServoAngles) {
    	pthread_mutex_lock(&state->servo_angles_mutex);
		for (i = 0; i < NUM_SERVOS; i++) {
			state->target_servo_angles[i] = state->gui_servo_angles[i];
		}
		pthread_mutex_unlock(&state->servo_angles_mutex);
    }
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

void draw_arm(vx_buffer_t *buf, double angles[], const float color[]) {
	vx_object_t *segment = vxo_chain(
		// Base
		vxo_mat_rotate_z(angles[0]),
		vxo_mat_scale3(CM_TO_VX, CM_TO_VX, CM_TO_VX),
		vxo_mat_translate3(0, 0, arm_segment_length[0]/2),
		vxo_mat_scale3(arm_segment_width, arm_segment_depth, arm_segment_length[0]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/arm_segment_width, 1/arm_segment_depth, 1/arm_segment_length[0]),
		vxo_mat_translate3(0, 0, arm_segment_length[0]/2),
		// Upper Arm
		vxo_mat_rotate_x(angles[1]),
		vxo_mat_translate3(0, 0, arm_segment_length[1]/2),
		vxo_mat_scale3(arm_segment_width, arm_segment_depth, arm_segment_length[1]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/arm_segment_width, 1/arm_segment_depth, 1/arm_segment_length[1]),
		vxo_mat_translate3(0, 0, arm_segment_length[1]/2),
		// Lower Arm
		vxo_mat_rotate_x(angles[2]),
		vxo_mat_translate3(0, 0, arm_segment_length[2]/2),
		vxo_mat_scale3(arm_segment_width, arm_segment_depth, arm_segment_length[2]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/arm_segment_width, 1/arm_segment_depth, 1/arm_segment_length[2]),
		vxo_mat_translate3(0, 0, arm_segment_length[2]/2),
		// Wrist
		vxo_mat_rotate_x(angles[3]),
		vxo_mat_translate3(0, 0, arm_segment_length[3]/2),
		vxo_mat_scale3(arm_segment_width, arm_segment_depth, arm_segment_length[3]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/arm_segment_width, 1/arm_segment_depth, 1/arm_segment_length[3]),
		vxo_mat_translate3(0, 0, arm_segment_length[3]/2),
		// Gripper
		vxo_mat_rotate_z(angles[4] + M_PI/2),
		// Static Gripper
		vxo_mat_translate3(0, -arm_segment_depth/2, arm_segment_length[4]/2),
		vxo_mat_scale3(arm_segment_width, arm_segment_depth/2, arm_segment_length[4]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/arm_segment_width, 2/arm_segment_depth, 1/arm_segment_length[4]),
		vxo_mat_translate3(0, arm_segment_depth, -arm_segment_length[4]/2),
		// Dynamic Gripper
		vxo_mat_rotate_x(angles[5] - M_PI/2),
		vxo_mat_translate3(0, 0, arm_segment_length[4]/2),
		vxo_mat_scale3(arm_segment_width, arm_segment_depth/2, arm_segment_length[4]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/arm_segment_width, 2/arm_segment_depth, 1/arm_segment_length[4])
	);

	vx_buffer_add_back(buf, segment);

	/*segment = vxo_chain(
		vxo_mat_rotate_z(angles[0]),
		vxo_mat_scale3(CM_TO_VX, CM_TO_VX, CM_TO_VX),
		vxo_mat_translate3(0, 0, arm_segment_length[0]/2 + arm_segment_length[1]/2),
		vxo_mat_scale3(arm_segment_width, arm_segment_depth, arm_segment_length[0]),
		vxo_box(vxo_mesh_style(color))
	);*/
}

int initArmsLayer(state_t *state, layer_data_t *layerData) {
	layerData->world = vx_world_create();
	return 1;
}

int displayInitArmsLayer(state_t *state, layer_data_t *layerData) {
	const float eye[3] = {0, -50, 25};
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
	float axes[12] = {-1000, 0, 0, 1000, 0, 0, 0, -1000, 0, 0, 0, 0};
	vx_resc_t *verts = vx_resc_copyf(axes, 12);
	vx_buffer_add_back(gridBuff, vxo_lines(verts, 4, GL_LINES, vxo_points_style(vx_red, 2.0f)));

	float posAxes[6] = {0, 0, 0, 0, 1000, 0};
	verts = vx_resc_copyf(posAxes, 6);
	vx_buffer_add_back(gridBuff, vxo_lines(verts, 2, GL_LINES, vxo_points_style(vx_green, 2.0f)));

	//Draw Arms
	vx_buffer_t *armBuff = vx_world_get_buffer(layerData->world, "arm");
	pthread_mutex_lock(&state->servo_angles_mutex);
	draw_arm(armBuff, state->target_servo_angles, vx_red);
	pthread_mutex_unlock(&state->servo_angles_mutex);
	draw_arm(armBuff, state->current_servo_angles, vx_blue);
	
	//Swap buffers
	vx_buffer_swap(gridBuff);
	vx_buffer_swap(armBuff);
	return 1;
}

int destroyArmsLayer(state_t *state, layer_data_t *layerData) {
	vx_world_destroy(layerData->world);
	return 1;
}

int initDebugLayer(state_t *state, layer_data_t *layerData) {
	layerData->world = vx_world_create();
	return 1;
}

int displayInitDebugLayer(state_t *state, layer_data_t *layerData) {
	float black[4] = {0.0f, 0.0f, 0.0f, 1.0f};
	vx_layer_set_background_color(layerData->layer, black);
	vx_layer_set_viewport_rel(layerData->layer, layerData->position);
	//vx_layer_add_event_handler(layerData->layer, &state->veh);
	return 1;
}

int renderDebugLayer(state_t *state, layer_data_t *layerData) {
	/*vx_buffer_t *textBuff = vx_world_get_buffer(layerData->world, "text");

	char debugText[700];
	const char* formatting = "<<left,#ffffff,serif>>X: %f\nY: %f\nTheta: %f\nGyro[0]: %d\nDiff_x: %f\nPID_OUT: %f\nDIAMOND: %d\nDOING_PID: %d\nCOV_X: %f, COV_Y: %f\n, FSM Time(t): %f\n, FSM Time(c): %f\n";
	sprintf(debugText, formatting,
			state->pos_x, state->pos_y,
			state->pos_theta, state->gyro[0],
			state->diff_x, state->green_pid_out,
			state->diamond_seen, state->doing_pid,
			matd_get(state->var_matrix,0,0),
			matd_get(state->var_matrix,1,1),
			state->fsm_time_elapsed,
			state->fsmTimeElapsed
		   );
	vx_object_t *vo = vxo_text_create(VXO_TEXT_ANCHOR_TOP_LEFT, debugText);
	vx_buffer_add_back(textBuff, vxo_pix_coords(VX_ORIGIN_TOP_LEFT, vo));
	vx_buffer_swap(textBuff);*/
	//printf("endRender DEBUG\n");
	return 1;
}

int destroyDebugLayer(state_t *state, layer_data_t *layerData) {
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

	/*state->layers[1].enable = 1;
	state->layers[1].name = "Debug";
	state->layers[1].position[0] = 0;
	state->layers[1].position[1] = 0;
	state->layers[1].position[2] = 0.666f;
	state->layers[1].position[3] = 0.333f;
	state->layers[1].init = initDebugLayer;
	state->layers[1].displayInit = displayInitDebugLayer;
	state->layers[1].render = renderDebugLayer;
	state->layers[1].destroy = destroyDebugLayer;*/

	vx_remote_display_source_t * remote = vx_remote_display_source_create_attr(&state->app, &remote_attr);

	// Handles layer init, rendering, and destruction
	parameter_gui_t *pg = pg_create();
    pg_add_double_slider(pg, "s0", "S0 (Shoulder Rotation)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s1", "S1 (Shoulder Bend)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s2", "S2 (Elbow)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s3", "S3 (Wrist Bend)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s4", "S4 (Wrist Rotation)", -M_PI, M_PI, 0);
    pg_add_double_slider(pg, "s5", "S5 (Gripper)", -M_PI, M_PI, 0);
    pg_add_check_boxes(pg, "cb1", "Update Arm Continuously", state->update_arm_cont, NULL);
    pg_add_buttons(pg, "but1", "Update Arm", "but2", "Go To Home", NULL);

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
