#include "arm_state.h"
#include "rexarm.h"
#include "body.h"
#include "math.h"
#include "../common/math_util.h"

void stopArm(state_t* state){
	double angles[NUM_SERVOS];
	state->arm->getCurAngles(angles);
	state->arm->setTargetAngles(angles, state->cfs);
	return;
}

void rotateArm(state_t* state, bool left){
	double angles[NUM_SERVOS];
	state->arm->getCurAngles(angles);
	angles[0] = M_PI*(left ? 1 : -1);
	state->arm->setTargetSpeed(0.2);
	state->arm->setTargetAngles(angles, state->cfs);
	return;
}

void commandShoulderWrist(state_t* state, bool shoulder){
	double angles[NUM_SERVOS];
	state->body->getServoAngles(angles, 1);
	double shoulderAngle = angles[1];
	double elbowAngle = angles[2];

	state->arm->getCurAngles(angles);
	if(shoulder){
		angles[1] = shoulderAngle;
		angles[2] = elbowAngle;
	}else{
		angles[3] = elbowAngle;
	}
	state->arm->setTargetSpeed(0.5);
	state->arm->setTargetAngles(angles, state->cfs);
	return;
}

void openCloseGripper(state_t* state){
	double angles[NUM_SERVOS];
	state->arm->getCurAngles(angles);
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
	state->arm->setTargetAngles(angles, state->cfs);
	return;
}

void state_machine_run(state_t* state){
	while(state->running){
		switch(state->FSM_state){
			case FSM_ARM:{
				//right hand controls shoulder and elbow
				commandShoulderWrist(state, 1);
				break;}
			case FSM_WRIST:{
				//right hand controls wrist
				commandShoulderWrist(state, 0);
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
		if(state->FSM_state != state->FSM_next_state){
			stopArm(state);
		}
		state->FSM_state = state->FSM_next_state;
	}
}
