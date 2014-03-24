#ifndef __PIXEL__H__
#define __PIXEL__H__
#include <math.h>

typedef struct pixel pixel_t;
struct pixel{
    int x, y;
};

int pixel_width(pixel_t a, pixel_t b);
int pixel_height(pixel_t a, pixel_t b);
#endif
