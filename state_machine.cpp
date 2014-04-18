#include "arm_state.h"
#include "rexarm.h"
#include "body_utility.h"
#include "math.h"
#include "../common/math_util.h"
#include <unistd.h>
#include <pthread.h>

void stopArm(state_t* state){
	double angles[NUM_SERVOS], curAngles[NUM_SERVOS];
	state->arm->getTargetAngles(angles);
	state->arm->getCurAngles(curAngles);
	angles[0] = curAngles[0];
	state->arm->setTargetAngles(angles, state->cfs);
	return;
}

void rotateArm(state_t* state){
	double angleSpeedThres = 0.4;
	double angles[NUM_SERVOS], curAngles[NUM_SERVOS];
	body_getServoAngles(state->body, angles, 1);
	state->arm->getTargetAngles(curAngles);
	curAngles[0] = (M_PI-0.2)*(angles[2] > 0 ? 1 : -1);

	double speed = fabs(angles[2]);

	if (speed > angleSpeedThres) {
		speed = (fabs(angles[2]) - angleSpeedThres) / (2*M_PI);
	} else {
		speed = 0;
	}

	state->arm->setTargetSpeed(speed);
	state->arm->setTargetAngles(curAngles, state->cfs);
	return;
}

void commandShoulderWrist(state_t* state, bool shoulder){
	double angles[NUM_SERVOS], curAngles[NUM_SERVOS];
	body_getServoAngles(state->body, angles, 1);
	double shoulderAngle = angles[1];
	double elbowAngle = angles[2];

	state->arm->getTargetAngles(curAngles);
	if(shoulder){
		curAngles[1] = shoulderAngle;
		curAngles[2] = elbowAngle;
	}else{
		curAngles[3] = elbowAngle;
	}
	state->arm->setTargetSpeed(0.5);
	state->arm->setTargetAngles(curAngles, state->cfs);
	return;
}

void openCloseGripper(state_t* state){
	double angles[NUM_SERVOS], curAngles[NUM_SERVOS];
	double maxAngle = 2.355;
	double threshold = 0.2;
	state->arm->getCurAngles(curAngles);
	state->arm->getTargetAngles(angles);
	double last_gripper_angle = curAngles[5];

	if(state->close_gripper){
		if (fabs(curAngles[5] - maxAngle) > threshold || fabs(curAngles[5] - state->last_gripper_angle) > threshold) {
			// Not completely closed or stopped closing
			angles[5] = curAngles[5] + 0.1;
			state->arm->setTargetSpeed(0.3);
		}
	}else{
		angles[5] = 0;
		state->arm->setTargetSpeed(0.5);
	}
	state->last_gripper_angle = last_gripper_angle;
	state->arm->setTargetAngles(angles, state->cfs);
	return;
}

void state_machine_run(state_t* state){
	int hz = 30;
	while(state->running){
		pthread_mutex_lock(&state->fsm_mutex);
		if(state->FSM_state != state->FSM_next_state){
			stopArm(state);
		}
		state->FSM_state = state->FSM_next_state;
		switch(state->FSM_state){
			case FSM_ARM:{
				//right hand controls shoulder and elbow
				commandShoulderWrist(state, true);
				break;}
			case FSM_WRIST:{
				//right hand controls wrist
				commandShoulderWrist(state, false);
				break;}
			case FSM_GRIP:{
				//left hand controls grip
				openCloseGripper(state);
				break;}
			case FSM_ROTATE:{
				rotateArm(state);
				break;}
			case FSM_NONE:{
				//freeze the arm
				break;}
		}
		pthread_mutex_unlock(&state->fsm_mutex);
		usleep(1000000/hz);
	}
}
