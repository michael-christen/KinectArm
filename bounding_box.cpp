#include "common/matd.h"
#include <math.h>
#include "bounding_box.h"

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
}

BoundingBox::~BoundingBox() {
	matd_destroy(this->pos);
	matd_destroy(this->ux);
	matd_destroy(this->uy);
	matd_destroy(this->uz);
}

void BoundingBox::setPosition(double x, double y, double z) {
	matd_put(this->pos, 0, 0, x);
	matd_put(this->pos, 0, 1, y);
	matd_put(this->pos, 0, 2, z);
}

void BoundingBox::setDimensions(double w, double h, double d) {
	this->hW = w / 2;
	this->hH = h / 2;
	this->hD = d / 2;
}

void BoundingBox::setRotation(double rX, double rY, double rZ) {
	matd_destroy(this->ux);
}

bool BoundingBox::intersect(BoundingBox *b) {
	double checkL, sum;
	BoundingBox *a = this;
	matd_t *cross;
	matd_t *T = matd_subtract(b->pos, a->pos);

	// Case 1
	checkL = matd_vec_dot_product(T, a->ux);
	sum = a->hW;
	sum += fabs(b->hW*matd_vec_dot_product(a->ux, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(b->hD*matd_vec_dot_product(a->ux, b->uz));

	if (checkL > sum) {
		return true;
	}

	// Case 2
	checkL = matd_vec_dot_product(T, a->uy);
	sum = a->hH;
	sum += fabs(b->hW*matd_vec_dot_product(a->uy, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(b->hD*matd_vec_dot_product(a->uy, b->uz));

	if (checkL > sum) {
		return true;
	}

	// Case 3
	checkL = matd_vec_dot_product(T, a->uz);
	sum = a->hD;
	sum += fabs(b->hW*matd_vec_dot_product(a->uz, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->uz, b->uy));
	sum += fabs(b->hD*matd_vec_dot_product(a->uz, b->uz));

	if (checkL > sum) {
		return true;
	}

	// Case 4
	checkL = matd_vec_dot_product(T, b->ux);
	sum = fabs(a->hW*matd_vec_dot_product(a->ux, b->ux));
	sum += fabs(a->hH*matd_vec_dot_product(a->uy, b->ux));
	sum += fabs(a->hD*matd_vec_dot_product(a->uz, b->ux));
	sum += b->hW;

	if (checkL > sum) {
		return true;
	}

	// Case 5
	checkL = matd_vec_dot_product(T, b->uy);
	sum = fabs(a->hW*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(a->hH*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(a->hD*matd_vec_dot_product(a->uz, b->uy));
	sum += b->hH;

	if (checkL > sum) {
		return true;
	}

	// Case 6
	checkL = matd_vec_dot_product(T, b->uz);
	sum = fabs(a->hW*matd_vec_dot_product(a->ux, b->uz));
	sum += fabs(a->hH*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(a->hD*matd_vec_dot_product(a->uz, b->uz));
	sum += b->hD;

	// Case 7
	cross = matd_crossproduct(a->ux, b->ux);
	checkL = matd_vec_dot_product(T, cross);
	sum = fabs(a->hH*matd_vec_dot_product(a->uz, b->ux));
	sum += fabs(a->hD*matd_vec_dot_product(a->uy, a->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->ux, b->uz));
	sum += fabs(b->hH*matd_vec_dot_product(a->ux, b->uy));
	matd_destroy(cross);

	if (checkL > sum) {
		return true;
	}

	// Case 8
	cross = matd_crossproduct(a->ux, b->uy);
	checkL = matd_vec_dot_product(T, cross);
	sum = fabs(a->hH*matd_vec_dot_product(a->uz, b->uy));
	sum += fabs(a->hD*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(b->hW*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(b->hD*matd_vec_dot_product(a->ux, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
		return true;
	}

	// Case 9
	cross = matd_crossproduct(a->ux, b->uz);
	checkL = matd_vec_dot_product(T, cross);
	sum = fabs(a->hH*matd_vec_dot_product(a->uz, b->uz));
	sum += fabs(a->hD*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(b->hW*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(b->hH*matd_vec_dot_product(a->ux, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
		return true;
	}

	// Case 10
	cross = matd_crossproduct(a->ux, b->uz);
	checkL = matd_vec_dot_product(T, cross);
	sum = fabs(a->hW*matd_vec_dot_product(a->uz, b->ux));
	sum += fabs(a->hD*matd_vec_dot_product(a->ux, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->uy, b->uy));
	matd_destroy(cross);

	if (checkL > sum) {
		return true;
	}

	// Case 11
	cross = matd_crossproduct(a->uy, b->uy);
	checkL = matd_vec_dot_product(T, cross);
	sum = fabs(a->hW*matd_vec_dot_product(a->uz, b->uy));
	sum += fabs(a->hD*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(b->hW*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->uy, b->ux));
	matd_destroy(cross);

	// Case 12
	cross = matd_crossproduct(a->uy, b->uz);
	checkL = matd_vec_dot_product(T, cross);
	sum = fabs(a->hW*matd_vec_dot_product(a->uz, b->uz));
	sum += fabs(a->hD*matd_vec_dot_product(a->ux, b->uz));
	sum += fabs(b->hW*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(b->hH*matd_vec_dot_product(a->uy, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
		return true;
	}

	// Case 13
	cross = matd_crossproduct(a->uz, b->ux);
	checkL = matd_vec_dot_product(T, cross);
	sum = fabs(a->hW*matd_vec_dot_product(a->uy, b->ux));
	sum += fabs(a->hH*matd_vec_dot_product(a->ux, b->ux));
	sum += fabs(b->hH*matd_vec_dot_product(a->uz, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->uz, b->uy));
	matd_destroy(cross);

	if (checkL > sum) {
		return true;
	}

	// Case 14
	cross = matd_crossproduct(a->uz, b->uy);
	checkL = matd_vec_dot_product(T, cross);
	sum = fabs(a->hW*matd_vec_dot_product(a->uy, b->uy));
	sum += fabs(a->hH*matd_vec_dot_product(a->ux, b->uy));
	sum += fabs(b->hW*matd_vec_dot_product(a->uz, b->uz));
	sum += fabs(b->hD*matd_vec_dot_product(a->uz, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
		return true;
	}

	// Case 15
	cross = matd_crossproduct(a->uz, b->uz);
	checkL = matd_vec_dot_product(T, cross);
	sum = fabs(a->hW*matd_vec_dot_product(a->uy, b->uz));
	sum += fabs(a->hH*matd_vec_dot_product(a->ux, b->uz));
	sum += fabs(b->hW*matd_vec_dot_product(a->uz, b->uy));
	sum += fabs(b->hH*matd_vec_dot_product(a->uz, b->ux));
	matd_destroy(cross);

	if (checkL > sum) {
		return true;
	}

	matd_destroy(T);
	return false;
}
