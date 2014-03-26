#include "body.h"
#include "../common/matd.h"
#include "math.h"

void getServoAnglesFromBody(body_t* body, double servoAngles[NUM_SERVOS]){
	matd_t* floor_shoulder;
	matd_t* shoulder_elbow;
	matd_t* shoulder_elbow0;
	matd_t* shoulder_elbow1;
	matd_t* elbow_wrist;
	joint_t shoulder, elbow, wrist;

	if(body->use_right){
		//use right side of body
		shoulder = body->right_shoulder;
		elbow = body->right_elbow;
		wrist = body->right_wrist;
	}else{
		//use left side of body
		shoulder = body->left_shoulder;
		elbow = body->left_elbow;
		wrist = body->left_wrist;
	}

	double floor_shoulder_data[3] = {
		0, 0, shoulder.z};

	double shoulder_elbow_data[3] = {
		elbow.x - shoulder.x,
		elbow.y - shoulder.y,
		elbow.z - shoulder.z};

	double shoulder_elbow_data0[3] = {
		0,
		elbow.y - shoulder.y,
		elbow.z - shoulder.z}; 

	double shoulder_elbow_data1[3] = {
		elbow.x - shoulder.x,
		0,
		elbow.z - shoulder.z};
		
	double elbow_wrist_data[3] = {
		wrist.x - elbow.x,
		wrist.y - elbow.y,
		wrist.z - elbow.z};

	floor_shoulder = matd_create_data(3, 1, floor_shoulder_data);
	shoulder_elbow = matd_create_data(3, 1, shoulder_elbow_data);
	shoulder_elbow0 = matd_create_data(3, 1, shoulder_elbow_data0);
	shoulder_elbow1 = matd_create_data(3, 1, shoulder_elbow_data1);
	elbow_wrist = matd_create_data(3, 1, elbow_wrist_data);

	double magfs = matd_vec_mag(floor_shoulder);
	double magse = matd_vec_mag(shoulder_elbow);
	double magse0 = matd_vec_mag(shoulder_elbow0);
	double magse1 = matd_vec_mag(shoulder_elbow1);
	double magew = matd_vec_mag(elbow_wrist);

	double shoulderAngle0 = acos(matd_vec_dot_product(floor_shoulder, shoulder_elbow0) / (magfs * magse0));
	double shoulderAngle1 = acos(matd_vec_dot_product(floor_shoulder, shoulder_elbow1) / (magfs * magse1));
	double elbowAngle = acos(matd_vec_dot_product(shoulder_elbow, elbow_wrist) / (magse * magew));

	servoAngles[0] = shoulderAngle0;
	servoAngles[1] = shoulderAngle1;
	servoAngles[2] = elbowAngle;

	matd_destroy(floor_shoulder);
	matd_destroy(shoulder_elbow);
	matd_destroy(shoulder_elbow0);
	matd_destroy(shoulder_elbow1);
	matd_destroy(elbow_wrist);
}
