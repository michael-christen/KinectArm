#include "body.h"
#include "body_utility.h"
#include "../common/matd.h"
#include "../common/math_util.h"
#include "math.h"
#include "skeleton_joint_list_t.h"
#include "skeleton_joint_t.h"
#include "data_smoother.h"
#include "vx/vxo_drawables.h"
#include <stdio.h>
#include "data_smoother.h"

void body_processMsg(Body *body, const skeleton_joint_list_t *msg, DataSmoother *ds) {
	/*double distThresh = 1000;
	double dist = 0;
	for (int i = 0; i < msg->len; i++) {
		
		if (msg->joints[i].valid) {
			dist += fabs(msg->joints[i].x - this->joints[i].x);
			dist += fabs(msg->joints[i].y - this->joints[i].y);
			dist += fabs(msg->joints[i].z - this->joints[i].z);
		}		
	}

	printf("dist - %f\n", dist);*/

	//if (dist < distThresh) {
	body->setWristRotation(msg->wrist_rotation);

	for (int i = 0; i < msg->len; i++) {
		if (msg->joints[i].valid) {
			joint_t joint;
			joint.x = ds->getNewVal(i * 5, (double) msg->joints[i].x);
			joint.y = ds->getNewVal(i * 5 + 1, (double) msg->joints[i].y);
			joint.z = ds->getNewVal(i * 5 + 2, (double) msg->joints[i].z);
			joint.screen_x = ds->getNewVal(i * 5 + 3, (double) msg->joints[i].screen_x);
			joint.screen_y = ds->getNewVal(i * 5 + 4, (double) msg->joints[i].screen_y);
			body->setJoint((Joints) i, joint);
		}
	}
	//}
}

void body_getServoAngles(Body *body, double servoAngles[], bool right_side) {
	matd_t* floor_shoulder;
	matd_t* shoulder_elbow;
	matd_t* shoulder_elbow0;
	matd_t* shoulder_elbow1;
	matd_t* elbow_wrist;
	joint_t shoulder, elbow, wrist;

	if(right_side){
		//use right side of body
		shoulder = body->getJoint(RSHOULDER);
		elbow = body->getJoint(RELBOW);
		wrist = body->getJoint(RWRIST);
	}else{
		//use left side of body
		shoulder = body->getJoint(LSHOULDER);
		elbow = body->getJoint(LELBOW);
		wrist = body->getJoint(LWRIST);
	}

	double floor_shoulder_data[3] = {
		0, shoulder.y, 0};

	double shoulder_elbow_data[3] = {
		elbow.x - shoulder.x,
		elbow.y - shoulder.y,
		elbow.z - shoulder.z};

	//Shoulder rotation in the yz plane (forward/backward)
	double shoulder_elbow_data0[3] = {
		0,
		elbow.y - shoulder.y,
		elbow.z - shoulder.z}; 

	//Shoulder rotation in the xy plane (left/right)
	double shoulder_elbow_data1[3] = {
		elbow.x - shoulder.x,
		elbow.y - shoulder.y,
		0};

	double elbow_wrist_data[3] = {
		wrist.x - elbow.x,
		wrist.y - elbow.y,
		wrist.z - elbow.z};

	floor_shoulder = matd_create_data(3, 1, floor_shoulder_data);
	shoulder_elbow = matd_create_data(3, 1, shoulder_elbow_data);
	//shoulder_elbow0 = matd_create_data(3, 1, shoulder_elbow_data0);
	shoulder_elbow1 = matd_create_data(3, 1, shoulder_elbow_data1);
	elbow_wrist = matd_create_data(3, 1, elbow_wrist_data);

	double magfs = matd_vec_mag(floor_shoulder);
	double magse = matd_vec_mag(shoulder_elbow);
	//double magse0 = matd_vec_mag(shoulder_elbow0);
	double magse1 = matd_vec_mag(shoulder_elbow1);
	double magew = matd_vec_mag(elbow_wrist);
	
	//double shoulderValue0 = matd_vec_dot_product(floor_shoulder, shoulder_elbow0) / (magfs * magse0);
	double shoulderValue1 = matd_vec_dot_product(floor_shoulder, shoulder_elbow1) / (magfs * magse1);
	double elbowValue = matd_vec_dot_product(shoulder_elbow, elbow_wrist) / (magse * magew);

	//printf("Elbow: %g\n", elbowValue);

	//double shoulderAngle0 = (acos(shoulderValue0));
	double shoulderAngle1 = sgn(elbow.y - shoulder.y)*(acos(shoulderValue1) - M_PI/2);
	double elbowAngle = acos(elbowValue);

	/*if (elbow.z - shoulder.z < 0) {
		shoulderAngle0 = M_PI - shoulderAngle0;
	} else {
		shoulderAngle0 = shoulderAngle0 - M_PI;
	}*/

	if (elbow.y < shoulder.y) {
		shoulderAngle1 = -shoulderAngle1;
	}

	double unitZ_data[3] = {0, 0, 1};
	matd_t *unitZ = matd_create_data(3, 1, unitZ_data);
	matd_t *elbowCheck = matd_crossproduct(elbow_wrist, shoulder_elbow);
	double elbowSign = sgn(matd_vec_dot_product(elbowCheck, unitZ));

	matd_destroy(unitZ);
	matd_destroy(elbowCheck);

	servoAngles[1] = shoulderAngle1;
	servoAngles[2] = elbowSign*elbowAngle;

	//printf("shoulder - %f, elbow - %f\n", servoAngles[1], servoAngles[2]);

	matd_destroy(floor_shoulder);
	matd_destroy(shoulder_elbow);
	//matd_destroy(shoulder_elbow0);
	matd_destroy(shoulder_elbow1);
	matd_destroy(elbow_wrist);
}

void body_draw(Body *body, vx_buffer_t *buf) {
	vx_object_t *vo;
	float scale = 1/20.0;
	float zoffset = 0;

	joint_t rShoulder = body->getJoint(RSHOULDER);
	joint_t rElbow = body->getJoint(RELBOW);
	joint_t rWrist = body->getJoint(RWRIST);
	joint_t lWrist = body->getJoint(LWRIST);

	double rShoulderX = 0;
	double rShoulderY = 0;
	double rShoulderZ = scale*(-2 * rShoulder.y);
	double rElbowX = scale*(rElbow.x - rShoulder.x);
	double rElbowY = scale*(rElbow.z - rShoulder.z);
	double rElbowZ = scale*(-rElbow.y - rShoulder.y);
	double rWristX = scale*(rWrist.x - rShoulder.x);
	double rWristY = scale*(rWrist.z - rShoulder.z);
	double rWristZ = scale*(-rWrist.y - rShoulder.y);
	double lWristX = scale*(lWrist.x - rShoulder.x);
	double lWristY = scale*(lWrist.z - rShoulder.z);
	double lWristZ = scale*(-lWrist.y - rShoulder.y);
	//Draw Axes
	float axes[12] = {(float) rShoulderX, (float) rShoulderY, (float) rShoulderZ,
						(float) rElbowX, (float) rElbowY, (float) rElbowZ, 
						(float) rElbowX, (float) rElbowY, (float) rElbowZ,
						(float) rWristX, (float) rWristY, (float) rWristZ};

	vx_resc_t *verts = vx_resc_copyf(axes, 12);
	vo = vxo_chain(
		vxo_lines(verts, 4, GL_LINES, vxo_points_style(vx_blue, 2.0f))
	);

	vx_buffer_add_back(buf, vo);	

	//Draw Joints
	vo = vxo_chain(
		vxo_mat_translate3(rShoulderX, rShoulderY, rShoulderZ),
		vxo_mat_scale3(1.5, 1.5, 1.5),
		vxo_sphere(vxo_mesh_style(vx_yellow))
	);

	vx_buffer_add_back(buf, vo);

	vo = vxo_chain(
		vxo_mat_translate3(rElbowX, rElbowY, rElbowZ),
		vxo_mat_scale3(1.5, 1.5, 1.5),
		vxo_sphere(vxo_mesh_style(vx_blue))
	);

	vx_buffer_add_back(buf, vo);

	vo = vxo_chain(
		vxo_mat_translate3(rWristX, rWristY, rWristZ),
		vxo_mat_scale3(1.5, 1.5, 1.5),
		vxo_sphere(vxo_mesh_style(vx_green))
	);

	vx_buffer_add_back(buf, vo);

	vo = vxo_chain(
		vxo_mat_translate3(lWristX, lWristY, lWristZ),
		vxo_mat_scale3(3, 3, 3),
		vxo_sphere(vxo_mesh_style(vx_black))
	);

	vx_buffer_add_back(buf, vo);
}