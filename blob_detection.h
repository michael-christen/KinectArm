#ifndef __BLOB_DETECTION__H__
#define __BLOB_DETECTION__H__
#include "disjoint.h"
//#include "eecs467_util.h"
#include "common/image_util.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include "image_helper.h"
#include "pixel.h"
#include "Image.h"

//ABGR
//#define TEMPLATE_PX 0xff1e2a25
//#define TEMPLATE_PX 0xff2c3f34
//Green
//#define TEMPLATE_PX 0xff394d2c
//Blue
//#define TEMPLATE_PX 0xff4f4029
//#define TEMPLATE_PX 0xff514430
//#define SHOW_PX 0xffe127ff
#define MIN_PXS 500
#define MAX_PXS 40000000
#define MAX_NUM_NEIGHBORS 8
#define MAX_NUM_BALLS 15000

//#define COLOR_THRESHOLD 37.0
//GLOBALS

typedef struct ball ball_t;
struct ball {
    double x, y;
    pixel_t t, b, l, r;
    int valid;
    int num_px;
};

typedef struct blob blob_t;
struct blob {
	double x, y;
	pixel_t t, b, l, r;
	int valid;
	int num_px;
};

bool color_fit(double color_threshold, double template_hue, 
		uint32_t px);

bool px_match(uint32_t base, uint32_t test);


void getNLabels(int n_labels[], int labels[], int neighbors[], int
	len_neighbors);

//Requires len_labels >= 1
int minLabel(int n_labels[], int len_labels);


void unionLabels(std::vector<Set> links, std::vector<int> n_labels);

std::vector<blob_t> blob_detection(Image<uint32_t> &im, 
		double template_hue, uint32_t show_px,
		double color_threshold, int min_pxs);


#endif
