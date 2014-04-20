#include "gripperc.h"
#include "../common/math_util.h"
#include <glib-object.h>
#include <stdio.h>

int qpush(Pixel* q[], Pixel *p, int back){
	q[back] = p;
	return back + 1;
}

Hand_t handPixels(int x, int y, guint16 *reduced_buffer,
	int reduced_width, int reduced_height){

	int cornerx = x - XY_THRESHOLD;
	int cornery = y - XY_THRESHOLD;
	int boxsize = XY_THRESHOLD * 2;
	int tempx, tempy;
	int centerx, centery;
	if(cornerx < 0){
		cornerx = 0;
	}
	if(cornery < 0){
		cornery = 0;
	}
	if(cornerx + boxsize >= reduced_width && cornery + boxsize >= reduced_height){
		boxsize = imin(reduced_width - cornerx, reduced_height - cornery);
	}else if(cornerx + boxsize >= reduced_width){
		boxsize = reduced_width - cornerx;
	}else if(cornery + boxsize >= reduced_height){
		boxsize = reduced_height - cornery;
	}
	centerx = centery = boxsize / 2;
		
	//Create a map of the pixels around the hand
	Pixel map[4 * XY_THRESHOLD * XY_THRESHOLD];

	for(int i = 0; i < boxsize; i++){
		for(int k = 0; k < boxsize; k++){
			//O((XY_THRESHOLD*2)^2)
			tempx = cornerx + i;
			tempy = cornery + k;
			map[k * boxsize + i].x = tempx;
			map[k * boxsize + i].y = tempy;
			map[k * boxsize + i].i = i;
			map[k * boxsize + i].k = k;
			if(tempx < reduced_width && tempy < reduced_height){
				map[k * boxsize + i].z = reduced_buffer[tempy * reduced_width + tempx];
			}else{
				map[k * boxsize + i].z = -100;
			}
			map[k * boxsize + i].visited = 0;
		}
	}
	map[centery * boxsize + centerx].visited = 1;
	
	int hand_pixels = 1;
	int averagex = 0;
	int averagey = 0;

	Pixel* q[4 * XY_THRESHOLD * XY_THRESHOLD];
	int index = 0;
	int back = 0;
	back = qpush(q, &map[centery * boxsize + centerx], back);

	Pixel *next;
	while(index != back){
		Pixel cur = *q[index];	//q.front()
		index++; //q.pop()
		
		for(int i = 0; i < 4; i++){
			switch(i){
				case 0: //LEFT
						tempx = cur.i - 1;
						tempy = cur.k;
						break;
				case 1:	//DOWN
						tempx = cur.i;
						tempy = cur.k -1;
						break;
				case 2:	//RIGHT
						tempx = cur.i + 1;
						tempy = cur.k;
						break;
				case 3:	//UP
						tempx = cur.i;
						tempy = cur.k + 1;
						break;
				default: tempx = cur.i;
						 tempy = cur.k;
			}

			if(tempx >= 0 && tempx < boxsize && 
				tempy >= 0 && tempy < boxsize){
				next = &map[tempy * boxsize + tempx];
				//problem in next line
				if(!next->visited && abs(cur.z - next->z) < DEPTH_THRESHOLD){
					hand_pixels++;
					averagex += next->x;
					averagey += next->y;
					back = qpush(q, next, back);
				}
				next->visited = 1;
			}
		}
	}
	averagex /= hand_pixels;
	averagey /= hand_pixels;

	int xdif = x - averagex;
	int ydif = y - averagey;

	double theta = atan2(ydif, xdif);

	Hand_t hand = {hand_pixels, theta};
	return hand;
}
