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
#include "gui.h"
#include "state.h"
#include "eecs467_util.h"

int displayCount;

void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    if (!strcmp("sl1", name)) {
        printf("sl1 = %f\n", pg_gd(pg, name));
    } else if (!strcmp("sl2", name)) {
        printf("sl2 = %d\n", pg_gi(pg, name));
    } else if (!strcmp("cb1", name) | !strcmp("cb2", name)) {
        printf("%s = %d\n", name, pg_gb(pg, name));
    } else {
        printf("%s changed\n", name);
    }
}


void display_finished(vx_application_t * app, vx_display_t * disp)
{
	printf("Top of display finished\n");
	uint64_t i;
	vx_layer_t *value;

	state_t * state = app->impl;

	//printf("disp end: %d\n", disp);

	pthread_mutex_lock(&state->layer_mutex);

	for (i = 0; i < NUM_LAYERS; i++) {
		layer_data_t *layerData = &(state->layers[i]);
		if (layerData->enable == 1) {
			uint64_t key = ((uint64_t) disp) * (i + 1);
			printf("disp: %zu, key %zu\n", ((uint64_t) disp), key);
			zhash_remove(state->layer_map, &key, NULL, &value);
			printf("layer being destroyed!\n");
			vx_layer_destroy(value);
		}
	}

	pthread_mutex_unlock(&state->layer_mutex);

	displayCount--;
	if (displayCount <= 0) {
		pthread_mutex_lock(&state->running_mutex);
		printf("Last display quit, end program\n");
		state->running = 0;
		pthread_mutex_unlock(&state->running_mutex);
	}

	printf("hash table size after remove: %d\n", zhash_size(state->layer_map));
}

void display_started(vx_application_t * app, vx_display_t * disp)
{
	uint64_t i;

	state_t * state = app->impl;

	//printf("disp start: %d\n", disp);

	for (i = 0; i < NUM_LAYERS; i++) {
		layer_data_t *layerData = &(state->layers[i]);
		if (layerData->enable == 1) {
			uint64_t key = ((uint64_t) disp) * (i + 1);
			printf("Layer being created\n");
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

int initCameraPOVLayer(state_t *state, layer_data_t *layerData) {
	layerData->world = vx_world_create();
	return 1;
}

int displayInitCameraPOVLayer(state_t *state, layer_data_t *layerData) {
	/*image_source_format_t isrc_format;
	double decimate = state->getopt_options.decimate;
	state->isrc->get_format(state->isrc, 0, &isrc_format);

	float lowLeft[2] = {0, 0};
	float upRight[2] = {isrc_format.width / decimate, isrc_format.height / decimate};

	vx_layer_camera_fit2D(layerData->layer, lowLeft, upRight, 1);
	vx_layer_set_viewport_rel(layerData->layer, layerData->position);*/
	//vx_layer_add_event_handler(layerData->layer, &state->veh);
	return 1;
}

int renderCameraPOVLayer(state_t *state, layer_data_t *layerData) {
	/*if (state->imageValid == 0) {
		return 1;
	}
	pthread_mutex_lock(&state->image_mutex);
	image_u32_t *im = state->im;
	if (im != NULL) {

		double decimate = state->getopt_options.decimate;

		if (decimate != 1.0) {
			im = image_util_u32_decimate(im, decimate);
		}

		vx_object_t * vo = vxo_image_from_u32(im, VXO_IMAGE_FLIPY,
				VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

		// show downsampled image, but scale it so it appears the
		// same size as the original
		vx_buffer_t *vb = vx_world_get_buffer(layerData->world, "image");

		vx_buffer_add_back(vb, vo);
		vx_buffer_swap(vb);

		if (decimate != 1.0) {
			image_u32_destroy(im);
		}
	}
	pthread_mutex_unlock(&state->image_mutex);*/
	//printf("endRender cameraPOV\n");
	return 1;
}

int destroyCameraPOVLayer(state_t *state, layer_data_t *layerData) {
	vx_world_destroy(layerData->world);
	return 1;
}

int initWorldTopDownLayer(state_t *state, layer_data_t *layerData) {
	layerData->world = vx_world_create();
	return 1;
}

int displayInitWorldTopDownLayer(state_t *state, layer_data_t *layerData) {
	/*const float eye[3] = {0, 0, 20};
	const float lookat[3] = {0, 0, 0};
	const float up[3] = {0, 1, 0};

	vx_layer_set_viewport_rel(layerData->layer, layerData->position);
	vx_layer_add_event_handler(layerData->layer, &state->veh);
	vx_layer_camera_lookat(layerData->layer, eye, lookat, up, 1);*/
	return 1;
}

int renderWorldTopDownLayer(state_t *state, layer_data_t *layerData) {
	//Draw Grid
	/*vx_buffer_t *gridBuff = vx_world_get_buffer(layerData->world, "grid");
	vx_buffer_add_back(gridBuff, vxo_grid());
	//printf("stride %d\n", state->gridMap.image->stride);
	pthread_mutex_lock(&state->map_mutex);
	vx_object_t *vo = vxo_chain(
			vxo_mat_scale3(MAP_RES, MAP_RES, MAP_RES),
			vxo_mat_scale3(CM_TO_VX, CM_TO_VX, CM_TO_VX),
			vxo_mat_translate3(-state->map.width / 2 - 0.5, -state->map.height / 2 - 0.5, -0.05),
			vxo_image_from_u32(state->map.image, 0, 0)
			);
	pthread_mutex_unlock(&state->map_mutex);

	vx_buffer_add_back(gridBuff, vo);
	//Draw Axes
	float axes[12] = {-1000, 0, 0, 1000, 0, 0, 0, -1000, 0, 0, 1000, 0};
	vx_resc_t *verts = vx_resc_copyf(axes, 12);
	vx_buffer_add_back(gridBuff, vxo_lines(verts, 4, GL_LINES, vxo_points_style(vx_red, 2.0f)));
	//Draw Bruce
	vx_buffer_t *bruceBuff = vx_world_get_buffer(layerData->world, "bruce");

	vo = vxo_chain(
			vxo_mat_scale3(CM_TO_VX, CM_TO_VX, CM_TO_VX),
			vxo_mat_translate3(state->pos_x, state->pos_y, state->pos_z + BRUCE_HEIGHT / 2),
			vxo_mat_scale3(BRUCE_DIAMETER, BRUCE_DIAMETER, BRUCE_HEIGHT),
			vxo_cylinder(vxo_mesh_style(vx_blue),
				vxo_lines_style(vx_cyan, 2.0f))
			);

	vx_buffer_add_back(bruceBuff, vo);

	vo = vxo_chain(
			vxo_mat_scale3(CM_TO_VX, CM_TO_VX, CM_TO_VX),
			vxo_mat_translate3(state->pos_x, state->pos_y ,state->pos_z + BRUCE_HEIGHT + 0.1),
			vxo_mat_rotate_z(-state->pos_theta),
			vxo_mat_translate3(0, -BRUCE_DIAMETER / 2, 0),
			vxo_mat_rotate_x(-M_PI/2),
			vxo_mat_scale3(BRUCE_DIAMETER / 2, 1, BRUCE_DIAMETER),
			vxo_square_pyramid(vxo_mesh_style(vx_red))
			);

	vx_buffer_add_back(bruceBuff, vo);

	//Draw Actual Trajectory
	vx_buffer_t *tTrajBuff = vx_world_get_buffer(layerData->world, "target-trajectory");
	if (state->targetPathValid == 1) {
		draw_path(tTrajBuff, state->targetPath, vx_orange);
	}

	//Draw Actual Trajectory
	vx_buffer_t *aTrajBuff = vx_world_get_buffer(layerData->world, "actual-trajectory");
	if (state->pathTakenValid == 1) {
		draw_path(aTrajBuff, state->pathTaken, vx_green);
	}

	//Draw Gaussian Ellipse
	vx_buffer_t *ellipseBuff = vx_world_get_buffer(layerData->world, "ellipse-buff");
	for(int i = 0; i < state->stored_mat_num; ++i) {
		drawEllipse(ellipseBuff, state->stored_matrices[i],
					state->stored_pos[i], state);
	}
	odometry_t tp;
	tp.x = state->pos_x;
	tp.y = state->pos_y;
	tp.theta = state->pos_theta;
	drawEllipse(ellipseBuff, state->var_matrix, tp, state);

	//Swap buffers
	vx_buffer_swap(gridBuff);
	vx_buffer_swap(bruceBuff);
	vx_buffer_swap(ellipseBuff);
	vx_buffer_swap(tTrajBuff);
	vx_buffer_swap(aTrajBuff);*/
	//printf("endRender TOPDOWN\n");
	return 1;
}



int destroyWorldTopDownLayer(state_t *state, layer_data_t *layerData) {
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
	state_t * state = data;
	int i;

	// Init
	printf("Entering layer init\n");
	for (i = 0; i < NUM_LAYERS; i++) {
		layer_data_t *layer = &(state->layers[i]);
		if (layer->enable == 1) {
			if (layer->init(state, layer) == 0) {
				printf("Failed to init layer: %s\n", layer->name);
				return NULL;
			}
		}
	}
	printf("Entering render loop\n");
	// Render Loop
	while(state->running) {
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
			} else {\
				break;
			}
			pthread_mutex_unlock(&state->running_mutex);
		}
		// 30 FPS
		usleep(33333);
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
void* gui_create(void *data) {
	state_t * state = data;
	pthread_t render_thread;

	vx_remote_display_source_attr_t remote_attr;
	vx_remote_display_source_attr_init(&remote_attr);
	remote_attr.advertise_name = "Kinect Arm";


	// Init layer data structs
	/*state->layers[0].enable = 1;
	state->layers[0].name = "WorldTopDown";
	state->layers[0].position[0] = 0;
	state->layers[0].position[1] = 0.333f;
	state->layers[0].position[2] = 0.666f;
	state->layers[0].position[3] = 0.666f;
	state->layers[0].init = initWorldTopDownLayer;
	state->layers[0].displayInit = displayInitWorldTopDownLayer;
	state->layers[0].render = renderWorldTopDownLayer;
	state->layers[0].destroy = destroyWorldTopDownLayer;*/

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
    pg_add_double_slider(pg, "sl1", "Slider 1", 0, 100, 50);
    pg_add_int_slider(pg, "sl2", "Slider 2", 0, 100, 25);
    pg_add_check_boxes(pg,
                       "cb1", "Check Box 1", 0,
                       "cb2", "Check Box 2", 1,
                       NULL);
    pg_add_buttons(pg,
                   "but1", "Button 1",
                   "but2", "Button 2",
                   "but3", "Button 3",
                   NULL);

    parameter_listener_t *my_listener = calloc(1,sizeof(parameter_listener_t*));
    my_listener->impl = NULL;
    my_listener->param_changed = my_param_changed;
    pg_add_listener(pg, my_listener);

	pthread_create(&render_thread, NULL, renderLayers, state);
	eecs467_gui_run(&state->app, pg, 800, 600);
	pthread_join(render_thread, NULL);
	pthread_mutex_lock(&state->running_mutex);
	state->running = 0;
	pthread_mutex_unlock(&state->running_mutex);
	vx_remote_display_source_destroy(remote);

	printf("end of gui thread (after destroy)\n");
	return NULL;
}
