#ifndef __IMAGE_HELPER__H__
#define __IMAGE_HELPER__H__
#include "common/image_util.h"
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
uint8_t get_red(uint32_t px);
uint8_t get_green(uint32_t px);
uint8_t get_blue(uint32_t px);
uint32_t get_px(uint8_t r, uint8_t g, uint8_t b, uint8_t a);

double color_dist(uint32_t p1, uint32_t p2);

double hue_dist(double hue, uint32_t p2);

uint32_t avg_px(uint32_t *pxs, int n);

uint32_t dist_to_grey(double dist);

void RGBtoHSV( uint32_t rgb,
	double *h, double *s, double *v);

void RGBtoHSV( uint32_t r, uint32_t g, uint32_t b,
	double *h, double *s, double *v);

void HSVtoRGB(double h, double s, double v,
		uint8_t *r, uint8_t *g, uint8_t *b);

uint32_t HSVtoRGB(double h, double s, double v);

void fill_color(double hue, double thresh, image_u32_t *im);

#endif
