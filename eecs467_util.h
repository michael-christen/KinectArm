#ifndef EECS467_UTIL_H
#define EECS467_UTIL_H

// XXX This is evil
#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "common/image_u32.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

typedef struct eecs467_default_implementation eecs467_default_implementation_t;

eecs467_default_implementation_t *eecs467_default_implementation_create(vx_world_t *world, zhash_t *layers);

void eecs467_default_display_started(vx_application_t *app, vx_display_t *disp);
void eecs467_default_display_finished(vx_application_t *app, vx_display_t *disp);

void eecs467_init(int argc, char **argv);
void eecs467_gui_run(vx_application_t *app, parameter_gui_t *pg, int w, int h);

#endif
