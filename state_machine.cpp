#include "arm_state.h"
#include "rexarm.h"
#include "body.h"
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

void rotateArm(state_t* state, bool left){
	double angles[NUM_SERVOS];
	state->arm->getTargetAngles(angles);
	angles[0] = (M_PI-0.2)*(left ? 1 : -1);
	state->arm->setTargetSpeed(0.05);
	state->arm->setTargetAngles(angles, state->cfs);
	return;
}

void commandShoulderWrist(state_t* state, bool shoulder){
	double angles[NUM_SERVOS], curAngles[NUM_SERVOS];
	state->body->getServoAngles(angles, 1);
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
	double angles[NUM_SERVOS];
	state->arm->getTargetAngles(angles);
	double last_gripper_angle = angles[5];
	if(state->close_gripper){
		double threshold = 0.2;
		if(fabs(angles[5] - state->last_gripper_angle) < threshold){//slowing down
			angles[5] = angles[5] + 0.1;
			state->arm->setTargetSpeed(0.2);
		}else{
			angles[5] = 2.0*M_PI/3.0;
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
			case FSM_ROT_LEFT:{
				//rotate arm left at fixed speed
				rotateArm(state, 1);
				break;}
			case FSM_ROT_RIGHT:{
				//rotate arm right at fixed speed
				rotateArm(state, 0);
				break;}
			case FSM_NONE:{
				//freeze the arm
				break;}
		}
		pthread_mutex_unlock(&state->fsm_mutex);
		usleep(1000000/hz);
	}
}
