#include "gripper.h"
#include "../common/math_util.h"
#include "math.h"
#include <queue>
#include <vector>

bool gripperClosed(int x, int y, int *reduced_buffer,
	int reduced_width, int reduced_height){

	int cornerx = x - XY_THRESHOLD;
	int cornery = y - XY_THRESHOLD;
	int boxsize = XY_THRESHOLD * 2;
	int tempx, tempy;
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
		
	//Create a map of the pixels around the hand
	std::vector<std::vector<Pixel>> map;

	for(int i = 0; i < boxsize; i++){
		for(int k = 0; k < boxsize; k++){
			//O((XY_THRESHOLD*2)^2)
			tempx = cornerx + i;
			tempy = cornery + k;
			map.at(i).at(k).x = tempx;
			map.at(i).at(k).y = tempy;
			if(tempx < reduced_width && tempy < reduced_height){
				map.at(i).at(k).z = reduced_buffer[tempy * reduced_width + tempx];
			}else{
				map.at(i).at(k).z = -100;
			}
			map.at(i).at(k).visited = false;
		}
	}
	map.at(x-cornerx).at(y-cornery).visited = true;
	
	int hand_pixels = 1;

	std::queue<Pixel> q;
	q.push(map.at(x-cornerx).at(y-cornery));

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

			if(tempx >= cornerx && tempx < (cornerx + boxsize) && 
				tempy >= cornery && tempy < (cornery + boxsize)){
				next = &map.at(tempx).at(tempy);
				if(!next->visited && abs(cur.z - next->z) < DEPTH_THRESHOLD){
					hand_pixels++;
					next->visited = true; //because we're pushing by value
					q.push(*next);
				}
				next->visited = true;
			}
		}
	}

	return (hand_pixels < PIXEL_THRESHOLD);
}
