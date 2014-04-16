#include "common/matd.h"
#include <math.h>
#include "bounding_box.h"
#include <stdio.h>
#include "arm_gui.h"
#include "vx/vx.h"
#include "vx/vxo_drawables.h"

BoundingBox::BoundingBox() {
	double vec[3] = {0, 0, 0};
	this->pos = matd_create_data(3, 1, vec);
	vec[0] = 1;
	this->ux = matd_create_data(3, 1, vec);
	vec[0] = 0;
	vec[1] = 1;
	this->uy = matd_create_data(3, 1, vec);
	vec[1] = 0;
	vec[2] = 1;
	this->uz = matd_create_data(3, 1, vec);

	this->tx = 0;
	this->ty = 0;
	this->tz = 0;
}

BoundingBox::~BoundingBox() {
	matd_destroy(this->pos);
	matd_destroy(this->ux);
	matd_destroy(this->uy);
	matd_destroy(this->uz);
}

void BoundingBox::reset() {
	this->setPosition(0, 0, 0);
	this->setRotations(0, 0, 0);
}

void BoundingBox::setPosition(double x, double y, double z) {
	matd_put(this->pos, 0, 0, x);
	matd_put(this->pos, 1, 0, y);
	matd_put(this->pos, 2, 0, z);
}

void BoundingBox::setPosition(matd_t *pos) {
	matd_put(this->pos, 0, 0, matd_get(pos, 0, 0));
	matd_put(this->pos, 1, 0, matd_get(pos, 1, 0));
	matd_put(this->pos, 2, 0, matd_get(pos, 2, 0));
}

void BoundingBox::setRotations(double tx, double ty, double tz) {
	this->tx = tx;
	this->ty = ty;
	this->tz = tz;
}

void BoundingBox::addRotations(double tx, double ty, double tz) {
	this->tx += tx;
	this->ty += ty;
	this->tz += tz;
}

void BoundingBox::setDimensions(double w, double h, double d) {
	this->hW = w / 2;
	this->hH = h / 2;
	this->hD = d / 2;
	this->w = w;
	this->h = h;
	this->d = d;
}

void BoundingBox::setUnitVectors(matd_t *ux, matd_t *uy, matd_t *uz) {
	matd_put(this->ux, 0, 0, matd_get(ux, 0, 0));
	matd_put(this->ux, 1, 0, matd_get(ux, 1, 0));
	matd_put(this->ux, 2, 0, matd_get(ux, 2, 0));
	matd_put(this->uy, 0, 0, matd_get(uy, 0, 0));
	matd_put(this->uy, 1, 0, matd_get(uy, 1, 0));
	matd_put(this->uy, 2, 0, matd_get(uy, 2, 0));
	matd_put(this->uz, 0, 0, matd_get(uz, 0, 0));
	matd_put(this->uz, 1, 0, matd_get(uz, 1, 0));
	matd_put(this->uz, 2, 0, matd_get(uz, 2, 0));
}

/*void BoundingBox::rotateUnitVectors(double tx, double ty, double tz) {	
	matd_t *temp;
	
	double rotation_dataz[9] = {cos(tz), -sin(tz), 0, sin(tz), cos(tz), 0, 0, 0, 1};
	matd_t *rot = matd_create_data(3, 3, rotation_dataz);
	// Rotate X	about Z
	temp = this->ux;
	this->ux = matd_multiply(rot, temp);
	matd_destroy(temp);
	// Rotate Y about Z
	temp = this->uy;
	this->uy = matd_multiply(rot, temp);
	matd_destroy(temp);
	//Rotate Z aboutZ
	temp = this->uz;
	this->uz = matd_multiply(rot, temp);
	matd_destroy(temp);
	matd_destroy(rot);
	
	double rotation_datax[9] = {1, 0, 0, 0, cos(tx), -sin(tx), 0, sin(tx), cos(tx)};
	rot = matd_create_data(3, 3, rotation_datax);
	// Rotate X	about X
	temp = this->ux;
	this->ux = matd_multiply(rot, temp);
	matd_destroy(temp);
	// Rotate Y about X
	temp = this->uy;
	this->uy = matd_multiply(rot, temp);
	matd_destroy(temp);
	//Rotate Z about X
	temp = this->uz;
	this->uz = matd_multiply(rot, temp);
	matd_destroy(temp);
	matd_destroy(rot);
	
	double rotation_datay[9] = {1, 0, 0, 0, cos(tx), -sin(tx), 0, sin(tx), cos(tx)};
	rot = matd_create_data(3, 3, rotation_datay);
	// Rotate X	about Y
	temp = this->ux;
	this->ux = matd_multiply(rot, temp);
	matd_destroy(temp);
	// Rotate Y about Y
	temp = this->uy;
	this->uy = matd_multiply(rot, temp);
	matd_destroy(temp);
	//Rotate Z about Y
	temp = this->uz;
	this->uz = matd_multiply(rot, temp);
	matd_destroy(temp);
	matd_destroy(rot);
}*/

bool BoundingBox::intersect(BoundingBox *b) {
	double checkL, sum;
	BoundingBox *a = this;
	matd_t *cross;
	matd_t *T = matd_subtract(b->pos, a->pos);

	//printf("\nT\n");
	//matd_print(T, "%f");

	/*printf("\na pos\n");
	matd_print(a->pos, "%f");

	printf("\nb pos\n");
	matd_print(b->pos, "%f");*/
	/*printf("\nb ux\n");
	matd_print(b->ux, "%f");
	printf("\nb uy\n");
	matd_print(b->uy, "%f");
	printf("\nb uz\n");
	matd_print(b->uz, "%f");*/

	// Case 1
	checkL = fabs(matd_vec_dot_product(T, a->ux));
	sum = a->hW;
	sum += fabs(b->hW*matd_vec_dot_product(a->ux, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(b->hD*matd_vec_dot_product(a->ux, b->uz));

	if (checkL > sum) {
	    //printf("Axis found, case 1\n");
		return false;
	}
	//printf("1 - checkL %f, sum %f\n", checkL, sum);

	// Case 2
	checkL = fabs(matd_vec_dot_product(T, a->uy));
	sum = a->hH;
	sum += fabs(b->hW*matd_vec_dot_product(a->uy, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(b->hD*matd_vec_dot_product(a->uy, b->uz));

	if (checkL > sum) {
	    //printf("Axis found, case 2\n");
		return false;
	}
	//printf("2 - checkL %f, sum %f\n", checkL, sum);

	// Case 3
	checkL = fabs(matd_vec_dot_product(T, a->uz));
	sum = a->hD;
	sum += fabs(b->hW*matd_vec_dot_product(a->uz, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->uz, b->uy));
	sum += fabs(b->hD*matd_vec_dot_product(a->uz, b->uz));

	if (checkL > sum) {
    	//printf("Axis found, case 3\n");
		return false;
	}
	//printf("3 - checkL %f, sum %f\n", checkL, sum);

	// Case 4
	checkL = fabs(matd_vec_dot_product(T, b->ux));
	sum = fabs(a->hW*matd_vec_dot_product(a->ux, b->ux));
	sum += fabs(a->hH*matd_vec_dot_product(a->uy, b->ux));
	sum += fabs(a->hD*matd_vec_dot_product(a->uz, b->ux));
	sum += b->hW;

	if (checkL > sum) {
	    //printf("Axis found, case 4\n");
		return false;
	}
	//printf("4 - checkL %f, sum %f\n", checkL, sum);

	// Case 5
	checkL = fabs(matd_vec_dot_product(T, b->uy));
	sum = fabs(a->hW*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(a->hH*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(a->hD*matd_vec_dot_product(a->uz, b->uy));
	sum += b->hH;

	if (checkL > sum) {
	    //printf("Axis found, case 5\n");
		return false;
	}
	//printf("5 - checkL %f, sum %f\n", checkL, sum);

	// Case 6
	checkL = fabs(matd_vec_dot_product(T, b->uz));
	sum = fabs(a->hW*matd_vec_dot_product(a->ux, b->uz));
	sum += fabs(a->hH*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(a->hD*matd_vec_dot_product(a->uz, b->uz));
	sum += b->hD;
	
	if (checkL > sum) {
	    //printf("Axis found, case 6\n");
		return false;
	}
	//printf("6 - checkL %f, sum %f\n", checkL, sum);

	// Case 7
	cross = matd_crossproduct(a->ux, b->ux);
	checkL = fabs(matd_vec_dot_product(T, cross));
	sum = fabs(a->hH*matd_vec_dot_product(a->uz, b->ux));
	sum += fabs(a->hD*matd_vec_dot_product(a->uy, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->ux, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->ux, b->uy));
	matd_destroy(cross);

	if (checkL > sum) {
	    //printf("Axis found, case 7\n");
		return false;
	}
	//printf("7 - checkL %f, sum %f\n", checkL, sum);

	// Case 8
	cross = matd_crossproduct(a->ux, b->uy);
	checkL = fabs(matd_vec_dot_product(T, cross));
	sum = fabs(a->hH*matd_vec_dot_product(a->uz, b->uy));
	sum += fabs(a->hD*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(b->hW*matd_vec_dot_product(a->ux, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->ux, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
	    //printf("Axis found, case 8\n");
		return false;
	}
	//printf("8 - checkL %f, sum %f\n", checkL, sum);

	// Case 9
	cross = matd_crossproduct(a->ux, b->uz);
	checkL = fabs(matd_vec_dot_product(T, cross));
	sum = fabs(a->hH*matd_vec_dot_product(a->uz, b->uz));
	sum += fabs(a->hD*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(b->hW*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(b->hH*matd_vec_dot_product(a->ux, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
	    //printf("Axis found, case 9\n");
		return false;
	}
	//printf("9 - checkL %f, sum %f\n", checkL, sum);

	// Case 10
	cross = matd_crossproduct(a->uy, b->ux);
	checkL = fabs(matd_vec_dot_product(T, cross));
	sum = fabs(a->hW*matd_vec_dot_product(a->uz, b->ux));
	sum += fabs(a->hD*matd_vec_dot_product(a->ux, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->uy, b->uy));
	matd_destroy(cross);

	if (checkL > sum) {
	    //printf("Axis found, case 10\n");
		return false;
	}
	//printf("10 - checkL %f, sum %f\n", checkL, sum);

	// Case 11
	cross = matd_crossproduct(a->uy, b->uy);
	checkL = fabs(matd_vec_dot_product(T, cross));
	sum = fabs(a->hW*matd_vec_dot_product(a->uz, b->uy));
	sum += fabs(a->hD*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(b->hW*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->uy, b->ux));
	matd_destroy(cross);
	
	if (checkL > sum) {
	    //printf("Axis found, case 11\n");
		return false;
	}
	//printf("11 - checkL %f, sum %f\n", checkL, sum);

	// Case 12
	cross = matd_crossproduct(a->uy, b->uz);
	checkL = fabs(matd_vec_dot_product(T, cross));
	sum = fabs(a->hW*matd_vec_dot_product(a->uz, b->uz));
	sum += fabs(a->hD*matd_vec_dot_product(a->ux, b->uz));
	sum += fabs(b->hW*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(b->hH*matd_vec_dot_product(a->uy, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
	    //printf("Axis found, case 12\n");
		return false;
	}
	//printf("12 - checkL %f, sum %f\n", checkL, sum);

	// Case 13
	cross = matd_crossproduct(a->uz, b->ux);
	checkL = fabs(matd_vec_dot_product(T, cross));
	sum = fabs(a->hW*matd_vec_dot_product(a->uy, b->ux));
	sum += fabs(a->hH*matd_vec_dot_product(a->ux, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->uz, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->uz, b->uy));
	matd_destroy(cross);

	if (checkL > sum) {
	    //printf("Axis found, case 13\n");
		return false;
	}
	//printf("13 - checkL %f, sum %f\n", checkL, sum);

	// Case 14
	cross = matd_crossproduct(a->uz, b->uy);
	checkL = fabs(matd_vec_dot_product(T, cross));
	sum = fabs(a->hW*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(a->hH*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(b->hW*matd_vec_dot_product(a->uz, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->uz, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
	    //printf("Axis found, case 14\n");
		return false;
	}
	//printf("14 - checkL %f, sum %f\n", checkL, sum);

	// Case 15
	cross = matd_crossproduct(a->uz, b->uz);
	checkL = fabs(matd_vec_dot_product(T, cross));
	sum = fabs(a->hW*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(a->hH*matd_vec_dot_product(a->ux, b->uz));
	sum += fabs(b->hW*matd_vec_dot_product(a->uz, b->uy));
	sum += fabs(b->hH*matd_vec_dot_product(a->uz, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
	    //printf("Axis found, case 15\n");
		return false;
	}
	//printf("15 - checkL %f, sum %f\n", checkL, sum);

	matd_destroy(T);
	return true;
}

bool BoundingBox::pointWithinBox(double x, double y, double z) {
	double bx = matd_get(this->pos, 0, 0);
	double by = matd_get(this->pos, 1, 0);
	double bz = matd_get(this->pos, 2, 0);

	return (x >= bx - this->hW && x <= bx + this->hW
		&& y >= by - this->hH && y <= by + this->hH
		&& z >= bz - this->hD && z <= bz + this->hD);
}

void BoundingBox::draw(vx_buffer *buf, const float color[]) {
    vx_object_t *box = vxo_chain(
	    // Base
	    vxo_mat_scale3(CM_TO_VX, CM_TO_VX, CM_TO_VX),
	    vxo_mat_translate3(this->getX(), this->getY(), this->getZ()),
	    vxo_mat_scale3(this->getW(), this->getH(), this->getD()),
	    vxo_box(vxo_mesh_style(color), vxo_lines_style(vx_black, 2.0f))
	);
	
	vx_buffer_add_back(buf, box);
}

double BoundingBox::getX() {
    return matd_get(this->pos, 0, 0);
}

double BoundingBox::getY() {
    return matd_get(this->pos, 1, 0);
}

double BoundingBox::getZ() {
    return matd_get(this->pos, 2, 0);
}

double BoundingBox::getW() {
    return this->w;
}

double BoundingBox::getH() {
    return this->h;
}

double BoundingBox::getD() {
    return this->d;
}

double BoundingBox::getHw() {
    return this->hW;
}

double BoundingBox::getHh() {
    return this->hH;
}

double BoundingBox::getHd() {
    return this->hD;
}
