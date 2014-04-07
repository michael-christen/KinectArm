#ifndef __LINE__H__
#define __LINE__H__
#include "Blob.h"
#include "Gradient.h"
#include<vector>
#include<cmath>

struct line_t {
	double m, b;
	//ll is lower-left, if leftmost is upper terminal point
	//ll is still that point, think of it as 1st priority to
	//left and second to lower, similar with ru
	pos_t ll, ru;
	double variance;
	double num_pts;
};

line_t linear_regression(Blob<Gradient> &blob);

#endif
