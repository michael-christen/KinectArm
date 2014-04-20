#include "gripper.h"
#include "../common/math_util.h"
#include "math.h"
#include <queue>
#include <vector>
#include <cstdio>

Hand_t handPixels(int x, int y, int *reduced_buffer,
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
		boxsize = std::min(reduced_width - cornerx, reduced_height - cornery);
	}else if(cornerx + boxsize >= reduced_width){
		boxsize = reduced_width - cornerx;
	}else if(cornery + boxsize >= reduced_height){
		boxsize = reduced_height - cornery;
	}
    centerx = centery = boxsize / 2;
		
	//Create a map of the pixels around the hand
	std::vector<std::vector<Pixel>> map;

	for(int i = 0; i < boxsize; i++){
		for(int k = 0; k < boxsize; k++){
			//O((XY_THRESHOLD*2)^2)
			tempx = cornerx + i;
			tempy = cornery + k;
			map.at(i).at(k).x = tempx;
			map.at(i).at(k).y = tempy;
			map.at(i).at(k).i = i;
			map.at(i).at(k).k = k;
			if(tempx < reduced_width && tempy < reduced_height){
				map.at(i).at(k).z = reduced_buffer[tempy * reduced_width + tempx];
			}else{
				map.at(i).at(k).z = -100;
			}
			map.at(i).at(k).visited = false;
		}
	}
	map.at(centerx).at(centery).visited = true;
	printf("Thinks hand is: x: %d, y: %d\n", centerx, centery);
	
	int hand_pixels = 1;
	int averagex = 0;
	int averagey = 0;

	std::queue<Pixel> q;
	q.push(map.at(centerx).at(centery));

	Pixel *next;
	while(!q.empty()){
		Pixel cur = q.front();
		q.pop();
		for(int i = 0; i < 4; i++){
			switch(i){
				case 0: //LEFT
						tempx = cur.x - 1;
						tempy = cur.y;
						break;
				case 1:	//DOWN
						tempx = cur.x;
						tempy = cur.y -1;
						break;
				case 2:	//RIGHT
						tempx = cur.x + 1;
						tempy = cur.y;
						break;
				case 3:	//UP
						tempx = cur.x;
						tempy = cur.y + 1;
						break;
				default: tempx = cur.x;
						 tempy = cur.y;
			}

			if(tempx >= 0 && tempx < boxsize && 
				tempy >= 0 && tempy < boxsize){
				next = &map.at(tempx).at(tempy);
				if(!next->visited && abs(cur.z - next->z) < DEPTH_THRESHOLD){
					hand_pixels++;
					averagex += next->x;
					averagey += next->y;
					next->visited = true; //because we're pushing by value
					q.push(*next);
				}
				next->visited = true;
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

Hand_t altHandPx(int start, Image<uint16_t> dp) {
	std::vector<int> passed = 
		blob_merging(dp,start);
	int num_px = passed.size();
	int avgX = 0;
	int avgY = 0;
	for(int i = 0; i < passed.size(); ++i) {
		int id = passed[i];
		int x  = dp.getX(id);
		int y  = dp.getY(id);
		avgX += x; 
		avgY += y;
	}
	avgX /= num_px;
	avgY /= num_px;
	int origX = dp.getX(start);
	int origY = dp.getY(start);
	double theta = 
		atan2(origY-avgY, origX-avgX);
	Hand_t hand = {num_px, theta};
	return hand;
}
