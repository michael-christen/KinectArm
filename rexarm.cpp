#include "rexarm.h"
#include "arm_gui.h"
#include <math.h>
#include <stdio.h>
#include "arm_gui.h"
#include "vx/vxo_drawables.h"

int colCount = 0;

const int RexArm::numServos = NUM_SERVOS;
const double RexArm::segmentWidth[4] = {5, 5, 5, 5};
const double RexArm::segmentHeight[4] = {4, 4, 4, 2.5};
const double RexArm::segmentDepth[4] = {10, 10, 9.5, 8};
const double RexArm::segmentZOffset = 11.5;

RexArm::RexArm() {
	printf("REXARM CTOR\n");
	pthread_mutex_init(&this->curAnglesMutex, NULL);
	pthread_mutex_init(&this->targetAnglesMutex, NULL);
	this->dsf = 0.15;
	this->tsf = 0.2;
	for (int i = 0; i < NUM_SERVOS; i++) {
		this->targetAngles[i] = this->prevAngles[i] = this->curAngles[i] = 0;
		this->prevTrendFactors[i] = 0;
	}
}

void RexArm::setDSF(double dsf) {
	this->dsf = dsf;
}

double RexArm::getDSF() {
	return this->dsf;
}

void RexArm::setTSF(double tsf) {
	this->tsf = tsf;
}

double RexArm::getTSF() {
	return this->tsf;
}

void RexArm::setBoundingBoxes(BoundingBox boxes[]) {
	for (int i = 0; i < NUM_SEGMENTS - 1; i++) {
		boxes[i].setPosition(0, 0, 0);
		boxes[i].setRotations(0, 0, 0);
		boxes[i].setDimensions(this->segmentWidth[i], this->segmentHeight[i], this->segmentDepth[i]);
	}

	boxes[4].setPosition(0, 0, 0);
	boxes[4].setRotations(0, 0, 0);
	boxes[4].setDimensions(this->segmentWidth[3], this->segmentHeight[3], this->segmentDepth[3]);
}

void RexArm::setTargetAngles(double newAngles[], ConfigSpace &cfs) {
	matd_t *pos, *curPos, *temp;
	matd_t *totRot, *totTransform;
	matd_t *ux, *uy, *uz;
	matd_t *tux, *tuy, *tuz;
	matd_t *tsux, *tsuy, *tsuz;
	BoundingBox tempSegments[NUM_SEGMENTS];
	this->setBoundingBoxes(tempSegments);

	double angles[NUM_SERVOS];
	double b[NUM_SERVOS];
	for (int i = 0; i < NUM_SERVOS; i++) {
		angles[i] = this->dsf*newAngles[i] + (1 - this->dsf) * (this->targetAngles[i] + this->prevTrendFactors[i]);
		b[i] = this->tsf * (angles[i] - this->targetAngles[i]) + (1 - this->tsf) * this->prevTrendFactors[i];
	}

	double transSeg0_data[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, RexArm::segmentZOffset, 0, 0, 0, 1};
	double transSeg1_data[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, this->segmentDepth[0]/2, 0, 0, 0, 1};
	double transSeg2_data[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, this->segmentDepth[1]/2, 0, 0, 0, 1};
	double transSeg3_data[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, this->segmentDepth[2]/2, 0, 0, 0, 1};
	double rotBase_data[] = {cos(angles[0]), -sin(angles[0]), 0, 0, sin(angles[0]), cos(angles[0]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	double rotShoulder_data[] = {1, 0, 0, 0, 0, cos(angles[1]), -sin(angles[1]), 0, 0, sin(angles[1]), cos(angles[1]), 0, 0, 0, 0, 1};
	double rotElbow_data[] = {1, 0, 0, 0, 0, cos(angles[2]), -sin(angles[2]), 0, 0, sin(angles[2]), cos(angles[2]), 0, 0, 0, 0, 1};
	double rotWrist_data[] = {1, 0, 0, 0, 0, cos(angles[3]), -sin(angles[3]), 0, 0, sin(angles[3]), cos(angles[3]), 0, 0, 0, 0, 1};

	matd_t *trans0 = matd_create_data(4, 4, transSeg0_data);
	matd_t *trans1 = matd_create_data(4, 4, transSeg1_data);
	matd_t *trans2 = matd_create_data(4, 4, transSeg2_data);
	matd_t *trans3 = matd_create_data(4, 4, transSeg3_data);

	matd_t *rotBase = matd_create_data(4, 4, rotBase_data);
	matd_t *rotShoulder = matd_create_data(4, 4, rotShoulder_data);
	matd_t *rotElbow = matd_create_data(4, 4, rotElbow_data);
	matd_t *rotWrist = matd_create_data(4, 4, rotWrist_data);

	double vec[4] = {0, 0, 0, 1};
	pos = matd_create_data(4, 1, vec);
	vec[0] = 1;
	ux = matd_create_data(4, 1, vec);
	vec[0] = 0;
	vec[1] = 1;
	uy = matd_create_data(4, 1, vec);
	vec[1] = 0;
	vec[2] = 1;
	uz = matd_create_data(4, 1, vec);

	totTransform = matd_op("M*M*M*M", trans0, rotBase, rotShoulder, trans1);
	curPos = matd_multiply(totTransform, pos);
	matd_destroy(totTransform);
	temp = curPos;
	curPos = matd_select(temp, 0, 2, 0, 0);
	tempSegments[0].setPosition(curPos);
	matd_destroy(temp);
	matd_destroy(curPos);
	totRot = matd_op("M*M", rotBase, rotShoulder);
	tux = matd_multiply(totRot, ux);
	tuy = matd_multiply(totRot, uy);
	tuz = matd_multiply(totRot, uz);
	tsux = matd_select(tux, 0, 2, 0, 0);
	tsuy = matd_select(tuy, 0, 2, 0, 0);
	tsuz = matd_select(tuz, 0, 2, 0, 0);
	tempSegments[0].setUnitVectors(tsux, tsuy, tsuz);
	matd_destroy(tux);
	matd_destroy(tuy);
	matd_destroy(tuz);
	matd_destroy(tsux);
	matd_destroy(tsuy);
	matd_destroy(tsuz);

	totTransform = matd_op("M*M*M*M*M*M*M", trans0, rotBase, rotShoulder, trans1, trans1, rotElbow, trans2);
	curPos = matd_multiply(totTransform, pos);
	matd_destroy(totTransform);
	temp = curPos;
	curPos = matd_select(temp, 0, 2, 0, 0);
	tempSegments[1].setPosition(curPos);
	matd_destroy(temp);
	matd_destroy(curPos);
	totRot = matd_op("M*M*M", rotBase, rotShoulder, rotElbow);
	tux = matd_multiply(totRot, ux);
	tuy = matd_multiply(totRot, uy);
	tuz = matd_multiply(totRot, uz);
	tsux = matd_select(tux, 0, 2, 0, 0);
	tsuy = matd_select(tuy, 0, 2, 0, 0);
	tsuz = matd_select(tuz, 0, 2, 0, 0);
	tempSegments[1].setUnitVectors(tsux, tsuy, tsuz);
	matd_destroy(tux);
	matd_destroy(tuy);
	matd_destroy(tuz);
	matd_destroy(tsux);
	matd_destroy(tsuy);
	matd_destroy(tsuz);

	totTransform = matd_op("M*M*M*M*M*M*M*M*M*M", trans0, rotBase, rotShoulder, trans1, trans1, rotElbow, trans2, trans2, rotWrist, trans3);
	curPos = matd_multiply(totTransform, pos);
	matd_destroy(totTransform);
	temp = curPos;
	curPos = matd_select(temp, 0, 2, 0, 0);
	tempSegments[2].setPosition(curPos);
	//matd_print(curPos, "%f");
	matd_destroy(temp);
	matd_destroy(curPos);
	totRot = matd_op("M*M*M*M", rotBase, rotShoulder, rotElbow, rotWrist);
	tux = matd_multiply(totRot, ux);
	tuy = matd_multiply(totRot, uy);
	tuz = matd_multiply(totRot, uz);
	tsux = matd_select(tux, 0, 2, 0, 0);
	tsuy = matd_select(tuy, 0, 2, 0, 0);
	tsuz = matd_select(tuz, 0, 2, 0, 0);
	tempSegments[2].setUnitVectors(tsux, tsuy, tsuz);
	matd_destroy(tux);
	matd_destroy(tuy);
	matd_destroy(tuz);
	matd_destroy(tsux);
	matd_destroy(tsuy);
	matd_destroy(tsuz);

	matd_destroy(trans0);
	matd_destroy(trans1);
	matd_destroy(trans2);
	matd_destroy(trans3);

	matd_destroy(rotBase);
	matd_destroy(rotShoulder);
	matd_destroy(rotElbow);
	matd_destroy(rotWrist);

	matd_destroy(ux);
	matd_destroy(uy);
	matd_destroy(uz);


	//printf("---------\n");

	if (cfs.testCollisions(tempSegments, 3)) {
		printf("Collision detected! - %d\n", colCount++);
	} else {
		pthread_mutex_lock(&this->targetAnglesMutex);
		for (int i = 0; i < this->numServos; i++) {
			this->prevTrendFactors[i] = b[i];
			this->targetAngles[i] = angles[i];
		}
		pthread_mutex_unlock(&this->targetAnglesMutex);
	}

	
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
		vxo_mat_translate3(0, 0, RexArm::segmentZOffset),
		// Upper Arm
		vxo_mat_rotate_x(angles[1]),
		vxo_mat_translate3(0, 0, this->segmentDepth[0]/2),
		vxo_mat_scale3(this->segmentWidth[0], this->segmentHeight[0], this->segmentDepth[0]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_black, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[0], 1/this->segmentHeight[0], 1/this->segmentDepth[0]),
		vxo_mat_translate3(0, 0, this->segmentDepth[0]/2),
		// Lower Arm
		vxo_mat_rotate_x(angles[2]),
		vxo_mat_translate3(0, 0, this->segmentDepth[1]/2),
		vxo_mat_scale3(this->segmentWidth[1], this->segmentHeight[1], this->segmentDepth[1]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_black, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[1], 1/this->segmentHeight[1], 1/this->segmentDepth[1]),
		vxo_mat_translate3(0, 0, this->segmentDepth[1]/2),
		// Wrist
		vxo_mat_rotate_x(angles[3]),
		vxo_mat_translate3(0, 0, this->segmentDepth[2]/2),
		vxo_mat_scale3(this->segmentWidth[2], this->segmentHeight[2], this->segmentDepth[2]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_black, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[2], 1/this->segmentHeight[2], 1/this->segmentDepth[2]),
		vxo_mat_translate3(0, 0, this->segmentDepth[2]/2),
		// Gripper
		vxo_mat_rotate_z(angles[4] + M_PI/2),
		// Static Gripper
		vxo_mat_translate3(0, -this->segmentHeight[3]/2, this->segmentDepth[3]/2),
		vxo_mat_scale3(this->segmentWidth[2], this->segmentHeight[3]/2, this->segmentDepth[3]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_black, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[2], 2/this->segmentHeight[3], 1/this->segmentDepth[3]),
		vxo_mat_translate3(0, this->segmentHeight[3], -this->segmentDepth[3]/2),
		// Dynamic Gripper
		vxo_mat_rotate_x(angles[5] - M_PI/2),
		vxo_mat_translate3(0, 0, this->segmentDepth[3]/2),
		vxo_mat_scale3(this->segmentWidth[3], this->segmentHeight[3]/2, this->segmentDepth[3]),
		vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_black, 2.0f)),
		vxo_mat_scale3(1/this->segmentWidth[3], 2/this->segmentHeight[3], 1/this->segmentDepth[3])
	);

	vx_buffer_add_back(buf, segment);
}