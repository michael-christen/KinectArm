#include "vision_state.h"
#include "kinect_handle.h"
#include "Line.h"
#define CM_TO_VX 1

// Functions
void gui_create(state_t *state);
void display_finished(vx_application_t * app, vx_display_t * disp);
void display_started(vx_application_t * app, vx_display_t * disp);
void add_line_to_buffer(vx_buffer_t *vb, line_t line);
