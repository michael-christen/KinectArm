#ifndef __BLOB_DETECTION__H__
#define __BLOB_DETECTION__H__
#include "disjoint.h"
//#include "eecs467_util.h"
#include "common/image_util.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "image_helper.h"
#include "pixel.h"

#define COLOR_THRESHOLD 37.0
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

typedef struct ball ball_t;
struct ball {
    double x, y;
    pixel_t t, b, l, r;
    int valid;
    int num_px;
};

unsigned int is_ball(double color_threshold,
                    double template_hue, uint32_t px);


//Returns number of neighbors, modifies neighbors array to
//contain their position in the buf
unsigned int getNeighbors(image_u32_t *im, int x, int y,
	int neighbors[MAX_NUM_NEIGHBORS],
    double template_hue, double color_threshold);

void getNLabels(int n_labels[], int labels[], int neighbors[], int
	len_neighbors);

//Requires len_labels >= 1
int minLabel(int n_labels[], int len_labels);


void unionLabels(std::vector<Set> links, std::vector<int> n_labels);

int blob_detection(image_u32_t *im, ball_t *final_balls,
                   double template_hue, uint32_t show_px,
                   double color_threshold, int min_pxs);


#endif
