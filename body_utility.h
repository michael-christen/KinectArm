#ifndef BODY_UTILITY_H
#define BODY_UTILITY_H

#include "body.h"
#include "vx/vx.h"
#include "skeleton_joint_list_t.h"
#include "data_smoother.h"

void body_processMsg(Body *body, const skeleton_joint_list_t *msg, DataSmoother *ds);
void body_getServoAngles(Body *body, double servoAngles[], bool right_side);
void body_draw(Body *body, vx_buffer_t *buf);

#endif