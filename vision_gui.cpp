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
#include "vision_gui.h"
#include "vision_state.h"
#include "eecs467_util.h"

int displayCount;

double arm_segment_length[5] = {11.5, 10, 10, 9.5, 8};
double arm_segment_width = 5;
double arm_segment_depth = 2.5;

void my_param_changed(parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
	state_t *state = (state_t*) pl->impl;
    if (!strcmp("but1", name)) {
		state->set_hand_dist = 1;
    } else if (!strcmp("but2", name)) {
    	state->set_open_hand = 1;
    } else if (!strcmp("but3", name)) {
    	state->set_closed_hand = 1;
    } else if (!strcmp("cb1", name)) {
    	state->send_data = pg_gb(pg, name);
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

int initKinectImageLayer(state_t *state, layer_data_t *layerData) {
	layerData->world = vx_world_create();
	return 1;
}

int initKinectDepthLayer(state_t *state, layer_data_t *layerData) {
	layerData->world = vx_world_create();
	return 1;
}

int displayInitKinectImageLayer(state_t *state, layer_data_t *layerData) {
	float lowLeft[2] = {0, 0};
	float upRight[2] = {640, 480};

	vx_layer_camera_fit2D(layerData->layer, lowLeft, upRight, 1);
	vx_layer_set_viewport_rel(layerData->layer, layerData->position);
	return 1;
}

int displayInitKinectDepthLayer(state_t *state, layer_data_t *layerData) {
	float lowLeft[2] = {0, 0};
	float upRight[2] = {640, 480};

	vx_layer_camera_fit2D(layerData->layer, lowLeft, upRight, 1);
	vx_layer_add_event_handler(layerData->layer, &state->veh);
	vx_layer_set_viewport_rel(layerData->layer, layerData->position);
	return 1;
}

int renderKinectImageLayer(state_t *state, layer_data_t *layerData) {
	pthread_mutex_lock(&state->kinect_mutex);
	{
		//Visual map
		vx_object_t * vo = vxo_image_from_u32(
				state->im.getImage(videoToIm, state->getopt_options.use_markers), 
				VXO_IMAGE_FLIPY,
				VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
		vx_buffer_t *vb = vx_world_get_buffer(layerData->world, "viz-image");
		vx_buffer_add_back(vb, vo);
		for(size_t i = 0; i < state->im_lines.size(); ++i) {
			add_line_to_buffer(vb,state->im_lines[i]);
		}
		vx_buffer_swap(vb);
	}
	pthread_mutex_unlock(&state->kinect_mutex);
	return 1;
}
int normalize_y(int y) {
	return 480-y;
	//return y < 480/2 ? 480 - y : y - 480/2;
}

line_t normalize_line(line_t line) {
	line.ll.y = normalize_y(line.ll.y);
	line.ru.y = normalize_y(line.ru.y);
	return line;
}

void add_line_to_buffer(vx_buffer_t *vb, line_t line) {
	int npoints = 2;
	float points[npoints*3];
	line = normalize_line(line);
	
	points[3*0 + 0] = line.ll.x;
	points[3*0 + 1] = line.ll.y;
	points[3*0 + 2] = 0;
	points[3*1 + 0] = line.ru.x;
	points[3*1 + 1] = line.ru.y;
	points[3*1 + 2] = 0;
	vx_object_t * vo = vxo_chain(
			vxo_mat_scale3(1,1,1),
			vxo_mat_translate3(0,0,0),
			vxo_lines(
				vx_resc_copyf(points, npoints*3),
				npoints,
				GL_LINE_LOOP,
				vxo_lines_style(vx_red, 1.0f)
				)
			);
	vx_buffer_add_back(vb,vo);
}

int renderKinectDepthLayer(state_t *state, layer_data_t *layerData) {
	pthread_mutex_lock(&state->kinect_mutex);
	{
		//Depth map
		vx_object_t * vo = vxo_image_from_u32(
				state->depth.getImage(depthToIm, state->getopt_options.use_markers),
			   	VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
		vx_buffer_t *vb = vx_world_get_buffer(layerData->world, "depth-image");
		vx_buffer_add_back(vb, vo);
		for(size_t i = 0; i < state->depth_lines.size(); ++i) {
			add_line_to_buffer(vb,state->depth_lines[i]);
		}
		if (state->mouseDownSet) {
			line_t line;
			line.ll.x = line.ru.x = state->mouseDownX;
			line.ll.y = 0;
			line.ru.y = 680;
			add_line_to_buffer(vb,line);
		}
		vx_buffer_swap(vb);
	}
	pthread_mutex_unlock(&state->kinect_mutex);
	return 1;
}

int destroyKinectImageLayer(state_t *state, layer_data_t *layerData) {
	vx_world_destroy(layerData->world);
	return 1;
}

int destroyKinectDepthLayer(state_t *state, layer_data_t *layerData) {
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
	remote_attr.advertise_port++;
	remote_attr.connection_port++;

	// Init layer data structs
	state->layers[0].enable = 1;
	state->layers[0].name = "KinectImage";
	state->layers[0].position[0] = 0.0f;
	state->layers[0].position[1] = 0.0f;
	state->layers[0].position[2] = 0.5f;
	state->layers[0].position[3] = 1.0f;
	state->layers[0].init = initKinectImageLayer;
	state->layers[0].displayInit = displayInitKinectImageLayer;
	state->layers[0].render = renderKinectImageLayer;
	state->layers[0].destroy = destroyKinectImageLayer;
	// Init layer data structs
	state->layers[1].enable = 1;
	state->layers[1].name = "KinectDepth";
	state->layers[1].position[0] = 0.5f;
	state->layers[1].position[1] = 0.0f;
	state->layers[1].position[2] = 0.5f;
	state->layers[1].position[3] = 1.0f;
	state->layers[1].init = initKinectDepthLayer;
	state->layers[1].displayInit = displayInitKinectDepthLayer;
	state->layers[1].render = renderKinectDepthLayer;
	state->layers[1].destroy = destroyKinectDepthLayer;

	vx_remote_display_source_t * remote = vx_remote_display_source_create_attr(&state->app, &remote_attr);

	// Handles layer init, rendering, and destruction
	parameter_gui_t *pg = pg_create();
    pg_add_check_boxes(pg, "cb1", "Send Data", state->send_data, NULL);
    pg_add_buttons(pg, "but1", "Set Hand Distance",
    					"but2", "Set Open Hand",
    					"but3", "Set Closed Hand",
    					 NULL);

    parameter_listener_t *my_listener = (parameter_listener_t*) calloc(1,sizeof(parameter_listener_t*));
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener(pg, my_listener);

	pthread_create(&render_thread, NULL, renderLayers, state);
	eecs467_gui_run(&state->app, pg, 800, 600);
	pthread_mutex_lock(&state->running_mutex);
	state->running = 0;
	//pthread_join(&render_thread, NULL);
	//pthread_mutex_unlock(&state->running_mutex);
	vx_remote_display_source_destroy(remote);

	printf("end of gui thread (after destroy)\n");
}
