#ifndef GRIPPER_H
#define GRIPPER_H

#define DEPTH_THRESHOLD 20 //assuming mm, but not sure
#define XY_THRESHOLD 50 //defines bounding box of (XY_THRESHOLD*2)^2
#define PIXEL_THRESHOLD 500 //used to classify open/closed

struct Pixel {
	int x, y, z;
	bool visited;
};

/*gipperClosed
x and y: position of the hand
reduced_buffer: depth image
reduced_width and reduced_height: dimensions of depth image
*/
bool gripperClosed(int x, int y, int *reduced_buffer,
	 int reduced_width, int reduced_height);

#endif