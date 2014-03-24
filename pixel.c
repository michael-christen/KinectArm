#include "pixel.h"
#include <math.h>
#include <stdlib.h>

int pixel_width(pixel_t a, pixel_t b) {
    return abs(a.x - b.x);
}

int pixel_height(pixel_t a, pixel_t b) {
    return abs(a.y - b.y);
}
