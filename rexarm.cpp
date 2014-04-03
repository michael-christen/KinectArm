#include "rexarm.h"
#include "arm_gui.h"
#include <math.h>
#include <stdio.h>

#include "vx/vxo_drawables.h"

const int RexArm::numServos = NUM_SERVOS;
const double RexArm::segmentLength[5] = {11.5, 10, 10, 9.5, 8};
const double RexArm::segmentWidth[5] = {5, 5, 5, 5, 5};
const double RexArm::segmentDepth[5] = {2.5, 2.5, 2.5, 2.5, 2.5};

RexArm::RexArm() {
	pthread_mutex_init(&this->curAnglesMutex, NULL);
	pthread_mutex_init(&this->targetAnglesMutex, NULL);
}

void RexArm::setTargetAngles(double angles[]) {
	pthread_mutex_lock(&this->targetAnglesMutex);
	for (int i = 0; i < this->numServos; i++) {
		this->targetAngles[i] = angles[i];
	}
	pthread_mutex_unlock(&this->targetAnglesMutex);
}

void RexArm::setCurAngles(double angles[]) {
	pthread_mutex_lock(&this->curAnglesMutex);
	for (int i = 0; i < this->numServos; i++) {
		this->curAngles[i] = angles[i];
	}
	pthread_mutex_unlock(&this->curAnglesMutex);
}

void RexArm::getTargetAngles(double arr[]) {
	pthread_mutex_lock(&this->targetAnglesMutex);
	for (int i = 0; i < this->numServos; i++) {
		arr[i] = this->targetAngles[i];
	}
	pthread_mutex_unlock(&this->targetAnglesMutex);
}

void RexArm::getCurAngles(double arr[]) {
	pthread_mutex_lock(&this->curAnglesMutex);
	for (int i = 0; i < this->numServos; i++) {
		arr[i] = this->curAngles[i];
	}
	pthread_mutex_unlock(&this->curAnglesMutex);
}

void RexArm::drawCurState(vx_buffer_t *buf, const float color[]) {
	pthread_mutex_lock(&this->curAnglesMutex);
	this->drawState(buf, color, this->curAngles);
	pthread_mutex_unlock(&this->curAnglesMutex);
}

void RexArm::drawTargetState(vx_buffer_t *buf, const float color[]) {
	pthread_mutex_lock(&this->targetAnglesMutex);
	this->drawState(buf, color, this->targetAngles);
	pthread_mutex_unlock(&this->targetAnglesMutex);
}

void RexArm::drawState(vx_buffer_t *buf, const float color[], double angles[]) {
	vx_object_t *segment = vxo_chain(
		// Base
		vxo_mat_rotate_z(angles[0]),
		vxo_mat_scale3(CM_TO_VX, CM_TO_VX, CM_TO_VX),
		vxo_mat_translate3(0, 0, this->segmentLength[0]/2),
		vxo_mat_scale3(this->segmentWidth[0], this->segmentDepth[0], this->segmentLength[0]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[0], 1/this->segmentDepth[0], 1/this->segmentLength[0]),
		vxo_mat_translate3(0, 0, this->segmentLength[0]/2),
		// Upper Arm
		vxo_mat_rotate_x(angles[1]),
		vxo_mat_translate3(0, 0, this->segmentLength[1]/2),
		vxo_mat_scale3(this->segmentWidth[1], this->segmentDepth[1], this->segmentLength[1]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[1], 1/this->segmentDepth[1], 1/this->segmentLength[1]),
		vxo_mat_translate3(0, 0, this->segmentLength[1]/2),
		// Lower Arm
		vxo_mat_rotate_x(angles[2]),
		vxo_mat_translate3(0, 0, this->segmentLength[2]/2),
		vxo_mat_scale3(this->segmentWidth[2], this->segmentDepth[2], this->segmentLength[2]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[2], 1/this->segmentDepth[2], 1/this->segmentLength[2]),
		vxo_mat_translate3(0, 0, this->segmentLength[2]/2),
		// Wrist
		vxo_mat_rotate_x(angles[3]),
		vxo_mat_translate3(0, 0, this->segmentLength[3]/2),
		vxo_mat_scale3(this->segmentWidth[3], this->segmentDepth[3], this->segmentLength[3]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[3], 1/this->segmentDepth[3], 1/this->segmentLength[3]),
		vxo_mat_translate3(0, 0, this->segmentLength[3]/2),
		// Gripper
		vxo_mat_rotate_z(angles[4] + M_PI/2),
		// Static Gripper
		vxo_mat_translate3(0, -this->segmentDepth[4]/2, this->segmentLength[4]/2),
		vxo_mat_scale3(this->segmentWidth[3], this->segmentDepth[4]/2, this->segmentLength[4]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[3], 2/this->segmentDepth[4], 1/this->segmentLength[4]),
		vxo_mat_translate3(0, this->segmentDepth[4], -this->segmentLength[4]/2),
		// Dynamic Gripper
		vxo_mat_rotate_x(angles[5] - M_PI/2),
		vxo_mat_translate3(0, 0, this->segmentLength[4]/2),
		vxo_mat_scale3(this->segmentWidth[4], this->segmentDepth[4]/2, this->segmentLength[4]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_yellow, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[4], 2/this->segmentDepth[4], 1/this->segmentLength[4])
	);

	vx_buffer_add_back(buf, segment);
}