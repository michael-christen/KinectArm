#ifndef GRIPPERC_H
#define GRIPPERC_H

#include <glib-object.h>

#define DEPTH_THRESHOLD 20 //assuming mm, but not sure
#define XY_THRESHOLD 50 //defines bounding box of (XY_THRESHOLD*2)^2
#define DELTA_THRESHOLD 400 //used to classify open/closed
#define CHANGE_SAMPLES 10

typedef struct Pixel Pixel;

struct Pixel {
	int x, y, z;
	int i, k;
	int visited;
};

/*gipperClosed
x and y: position of the hand
reduced_buffer: depth image
reduced_width and reduced_height: dimensions of depth image
*/
int handPixels(int x, int y, guint16 *reduced_buffer,
	 int reduced_width, int reduced_height);

#endif
