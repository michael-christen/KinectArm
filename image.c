#include "image.h"

uint8_t get_red(uint32_t px) {
    return px & 0xFF;
}
uint8_t get_green(uint32_t px) {
    return (px >> 8) & 0xFF;
}
uint8_t get_blue(uint32_t px) {
    return (px >> 16) & 0XFF;
}

double MIN(double a, double b, double c) {
    double min = a;
    if(b < min) min = b;
    if(c < min) min = c;
    return min;
}

double MAX(double a, double b, double c) {
    double max = a;
    if(b > max) max = b;
    if(c > max) max = c;
    return max;
}


void RGBtoHSV( uint32_t int_r, uint32_t int_g, uint32_t int_b, double *h, double *s, double *v)
{
	double r,g,b;
	r = (int_r + 0.0)/255.0;
	g = (int_g + 0.0)/255.0;
	b = (int_b + 0.0)/255.0;

	double min, max, delta;
	min = MIN( r, g, b ) + 0.0;
	max = MAX( r, g, b ) + 0.0;
	*v = max;
	// v
	delta = max - min;
	if( max != 0 ) {
		*s = delta / max;
	}
	// s
	else {
		// r = g = b = 0,
		// s = 0,
		// v is undefined
		*s = 0;
		*h = -1;
		return;
	}
	if( r == max) {
		*h = ( g - b + 0.0) / delta;
	}
	// between yellow & magenta
	else if( g == max) {
		*h = 2 + ( b - r + 0.0) / delta;
	}
	// between cyan & yellow
	else {
		*h = 4 + ( r - g + 0.0) / delta;
	}
	// between magenta & cyan
	*h *= 60;
	// degrees
	if( *h < 0)
	{
		*h += 360;
	}
}


//Returns distance from test_px to match_px
double color_dist(uint32_t p1, uint32_t p2) {
    //Only first 8 bits are used until computation
    uint32_t r1, g1, b1, r2, g2, b2;
    double   h1, s1, v1, h2, s2, v2;
	h1 = s1 = v1 = h2 = s2 = v2 = 2;
    //uint32_t a1, a2;
    r1 = p1 & 0xFF;
    r2 = p2 & 0XFF;
    g1 = (p1 >> 8) & 0xFF;
    g2 = (p2 >> 8) & 0xFF;
    b1 = (p1 >> 16) & 0XFF;
    b2 = (p2 >> 16) & 0XFF;
    //Can ignore alpha values
//    a1 = (p1 >> 24) & 0xFF;
 //   a2 = (p2 >> 24) & 0xFF;
    //return abs(r1 - r2) + abs(g1 - g2) + abs(b1 - b2);
    RGBtoHSV(r1,g1,b1,&h1,&s1,&v1);
    RGBtoHSV(r2,g2,b2,&h2,&s2,&v2);
	//printf("h1: %f, h2: %f, r2: %d, b2: %d, g2: %d\n",h1,h2,r2,b2,g2);
    return sqrt(pow((r1 - r2),2) + pow((g1 - g2),2) + pow((b1 - b2),2));
    //return sqrt(pow((h1 - h2),2) + pow((s1 - s2),2) + pow((v1 - v2),2));
//    return abs(h1 - h2) + abs(s1 - s2) + abs(v1 - v2);
}

double hue_dist(double hue, uint32_t p2) {
    //Only first 8 bits are used until computation
    uint32_t  r2, g2, b2;
    double    h2, s2, v2;
	 h2 = s2 = v2 = 2;
    //uint32_t a1, a2;
    r2 = p2 & 0XFF;
    g2 = (p2 >> 8) & 0xFF;
    b2 = (p2 >> 16) & 0XFF;
    //Can ignore alpha values
//    a1 = (p1 >> 24) & 0xFF;
 //   a2 = (p2 >> 24) & 0xFF;
    //return abs(r1 - r2) + abs(g1 - g2) + abs(b1 - b2);
    RGBtoHSV(r2,g2,b2,&h2,&s2,&v2);
	//printf("h1: %f, h2: %f, r2: %d, b2: %d, g2: %d\n",h1,h2,r2,b2,g2);
	return fabs(hue - h2);
}

uint32_t avg_px(uint32_t *pxs, int n) {
    uint32_t reds, greens, blues;
    reds = greens = blues = 0;
    for(int i = 0; i < n; ++i) {
	reds   += get_red(pxs[i]);
	greens += get_green(pxs[i]);
	blues  += get_blue(pxs[i]);
    }
    reds   = (uint32_t) (reds + 0.0)/(n+0.0);
    greens = (uint32_t) (greens + 0.0)/(n+0.0);
    blues  = (uint32_t) (blues + 0.0)/(n+0.0);
    return (0xff << 24) | (blues << 16) | (greens << 8) | reds;
}

uint32_t dist_to_grey(double dist) {
    //360 is largest distance
    uint32_t grey_val = (dist/360.0) * 256;
    grey_val &= 0xff;
    uint32_t result = 0xff000000;
    result |= grey_val;
    result |= grey_val << 8;
    result |= grey_val << 16;
    int valid = get_red(result) == get_blue(result) &&
	   get_blue(result) == get_green(result);
    if(!valid) {
	printf("grey_val: %x\n",grey_val);
	printf("r: %x\ng: %x\nb: %x\n\n",get_red(result),
	       get_green(result),get_blue(result));
	assert(0);
    }
    return result;
}

void fill_color(double hue, double thresh, image_u32_t *im) {
//	int num_points = 0;
	int y, x, id;
	uint32_t px;

	//1st pass
	for(x = 0; x < im->width; ++x) {
		//look bottom to top
		for(y = im->height-1; y >= 0; --y) {
			id = im->stride*y + x;
			px = im->buf[id];
			double dist = hue_dist(hue, px);
			im->buf[id] = dist_to_grey(dist);
			if(dist < thresh) {
				//Make sure above is blue too
				im->buf[id] = 0xff039dfd;
			}
		}
	}
}
