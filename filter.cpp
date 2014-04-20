/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

	* File Name : filter.cpp
	* Purpose :
	* Creation Date : 29-03-2014
	* Last Modified : Sun 20 Apr 2014 10:28:30 AM EDT
	* Created By : Michael Christen

_._._._._._._._._._._._._._._._._._._._._.*/

#include<iostream>
#include<cmath>
#include<string>
#include<vector>
#include<queue>
#include<algorithm>
#include<cassert>

#include "filter.h"


void filter_front(Image<uint16_t> & im) {
	//Find closest pixel that isn't 0
	uint16_t min_depth = 0xffff;
	//For some reason these values showed up at areas where they
	//shouldn't, filtering them out seemed to make things work much
	//better
	std::vector<uint16_t> NOISE_DEPTHS; 
	NOISE_DEPTHS.push_back(0x203);
	NOISE_DEPTHS.push_back(0x208);
	NOISE_DEPTHS.push_back(0x20f);
	NOISE_DEPTHS.push_back(0x2c0);
	NOISE_DEPTHS.push_back(0x2d8);
	NOISE_DEPTHS.push_back(0x472);
	NOISE_DEPTHS.push_back(0x4b2);
	NOISE_DEPTHS.push_back(0x6e0);
	NOISE_DEPTHS.push_back(0x780);
	NOISE_DEPTHS.push_back(0x9f8);
	NOISE_DEPTHS.push_back(513);
	NOISE_DEPTHS.push_back(517);
	int      min_id    = 0;
	for(size_t i = 0; i < im.size(); ++i) {
		uint16_t depth = im.get(i);
		if(depth >= MIN_ALLOWED_DEPTH &&
				depth < min_depth) {
			//If it's a noise value, ignore it
			if(std::find(NOISE_DEPTHS.begin(), NOISE_DEPTHS.end(),
						depth) != NOISE_DEPTHS.end()) {
				continue;
			}
			min_id = i;
			min_depth = depth;
		}
	}
	//min_id = im.id(320,240);
	//printf("Min depth: %d\n", min_depth);
	blob_merging(im, min_id);
}

std::vector<int> blob_merging(Image<uint16_t> &im, int start) {
	int id;
	uint32_t px;
	std::queue<int> search;
	std::vector<int> passed;
	std::vector<bool> visited   =
		std::vector<bool>(im.h()*im.w(), false);
	std::vector<bool> pass_vect =
		std::vector<bool>(im.h()*im.w(), false);
	static std::vector<bool> neighbor_search =
		std::vector<bool>(8,true);
	search.push(start);
	while(!search.empty()) {
		//printf("passed_size: %d\n",passed.size());
		size_t cur_id = search.front();
		search.pop();
		if(cur_id < 0 || cur_id >= im.size()) {
			assert(0);
			assert(cur_id >= 0 && cur_id < im.size());
		}
		if( visited[cur_id]) {
			continue;
		}
		passed.push_back(cur_id);
		pass_vect[cur_id] = true;
		visited[cur_id] = true;
		int x = cur_id % im.w();
		int y = cur_id / im.w();
		std::vector<int> neighbors = im.getNeighborIds(x, y,
				neighbor_search, px_close_enough);
		for(size_t i = 0; i < neighbors.size(); ++i) {
			id = neighbors[i];
			px = im.get(id);
			if(!visited[id] && 
					px_close_enough(px, im.get(cur_id)) &&
					px >= MIN_ALLOWED_DEPTH) {
				search.push(id);
			}
		}
	}
	//Mark
	//for(size_t i = 0; i < passed.size(); ++i) {
	//	id = passed[i];
	//	im->buf[id] = 0xff00ff00;
	//}
	//Filter out everything else
	for(size_t i = 0; i < im.size(); ++i) {
		if(!pass_vect[i]) {
			im.invalidate(i);
		}
	}
	return passed;
}


std::vector<int> getNeighbors(image_u32_t *im, int x, int y) {
	int id;
	uint32_t px;
	uint16_t depth, cur_depth;
	std::vector<int> neighbors;
	cur_depth  = im->buf[im->stride*y + x];
	//Searching *'s
	//***
	//*.*
	//***
	for(int j = -1; j <= 1; ++j) {
		//edge
		if(j + y < 0 || j + y > im->height) {
			continue;
		}
		for(int i = -1; i <= 1; ++i) {
			//edge
			if(i + x < 0 || i + x > im->width) {
				continue;
			}
			//only search *'s
			if(j == 0 && i == 0) {
				continue;
			}
			id = im->stride*(y+j) + (x+i);

			if(id < 0 || id >= im->height*im->stride) {
				continue;
			}
			px = im->buf[id];
			depth = get_px_depth(px);
			if(px_close_enough(cur_depth,depth)) {
				neighbors.push_back(id);
			}
		}
	}
	return neighbors;
}

bool px_close_enough(uint16_t depth_0, uint16_t depth_1) {
	return abs(depth_0 - depth_1) < 700;
}

bool grad_close_enough(Gradient cur, Gradient other) {
	return (cur.mag() > 70 && other.mag() > 70) && 
		//0.7 works well for image, but depth is a little too
		//jittery
		fabs(getThetaDist(cur.angle(),other.angle())) < 0.15;
}

bool get_all(uint16_t a, uint16_t b) {
	return true;
}


void dtocs(std::vector<double> & dist, Image<uint16_t> &im) {
	//printf("in dtocs\n");
	std::vector<unsigned int> dist_map = 
		std::vector<unsigned int>(im.size(), 0);
	std::vector<bool> search1 = {
		1, 1, 1,
		1,    0,
		0, 0, 0
	};
	std::vector<bool> search2 = {
		0, 0, 0,
		0,    1,
		1, 1, 1
	};

	double prev_time = utime_now()/1000000.0;
	//Setup the F image, non-bg is max, bg is 0
	for(int i = 0; i < im.size(); ++i) {
		if(im.isValid(i)) {
			dist_map[i] = UINT_MAX;
		}
	}
	std::vector<int> neighborIds;
	std::vector<unsigned int> calcs 
		= std::vector<unsigned int>(4,UINT_MAX);
	const int num_iters = 1;
	for(int n = 0; n < num_iters; ++n) {
		//First real pass
		for(int y = 0; y < im.h(); ++y) {
			for(int x = 0; x < im.w(); ++x) {
				neighborIds = im.getNeighborIds(
						x,y,search1,get_all);
				if(!neighborIds.empty()) {
					const unsigned int dist_to_i = 1;
					for(size_t i = 0; i < neighborIds.size(); ++i) {
						calcs[i] = 1 + dist_to_i + dist_map[i];
					}
					unsigned int min_val = 
						*std::min_element(calcs.begin(), calcs.begin() +
								neighborIds.size());
					dist_map[im.id(x,y)] = std::min(
							dist_map[im.id(x,y)],
							min_val);
				}
			}
		}
		//Second pass
		for(int y = im.h()-1; y >= 0; --y) {
			for(int x = im.w()-1; x >= 0; --x) {
				neighborIds = im.getNeighborIds(
						x,y,search2,get_all);
				if(!neighborIds.empty()) {
					const unsigned int dist_to_i = 1;
					for(size_t i = 0; i < neighborIds.size(); ++i) {
						calcs[i] = 1 + dist_to_i + dist_map[i];
					}
					unsigned int min_val = 
						*std::min_element(calcs.begin(), calcs.begin() +
								neighborIds.size());
					dist_map[im.id(x,y)] = std::min(
							dist_map[im.id(x,y)],
							min_val);
				}
			}
		}
	}
	//Set double values
	dist.clear();
	for(int i = 0; i < dist_map.size(); ++i) {
		dist.push_back( (double) dist_map[i]);
	}
	double cur_time = utime_now()/1000000.0;
	//printf("dtocs took: %f(s)\n",cur_time-prev_time);
	//printf("done hagar\n");
}

#define INF 1E20

//Adapted from Felzenswalb
std::vector<double> dTrans1(std::vector<double> & row) {
	std::vector<double> d = std::vector<double>(row.size(),0);
	std::vector<int>    v = std::vector<int>(row.size(), 0);
	std::vector<double> z = std::vector<double>(row.size()+1,0);
	int k = 0;
	v[0] = 0;
	z[0] = -INF;
	z[1] = +INF;
	for(int q = 1; q < row.size(); ++q) {
		double s = 
			((row[q] + pow(q,2)) -
			(row[v[k]] + pow(v[k],2))) / 
			(2*q - 2*v[k]);
		while(s <= z[k]) {
			k --;
			s = 
				((row[q] + pow(q,2)) -
				(row[v[k]] + pow(v[k],2))) / 
				(2*q - 2*v[k]);
		}
		k++;
		v[k] = q;
		z[k] = s;
		z[k+1] = +INF;
	}
	k = 0;
	for(int q = 0; q < row.size(); ++q) {
		while(z[k+1] < q) {
			k++;
		}
		d[q] = pow(q-v[k],2) + row[v[k]];
	}
	return d;
}

void dTransImage(Image<double> & im) {
	//Transform along columns
	double val;
	for(int x = 0; x < im.w(); ++x) {
		std::vector<double> col;
		for(int y = 0; y < im.h(); ++y) {
			val = im.isValid(x,y) ? +INF : 0;
			col.push_back(val);
		} 
		col = dTrans1(col);
		for(int y = 0; y < im.h(); ++y) {
			im.set(x,y,col[y]);
		}
	}

	//Transform along rows
	for(int y = 0; y < im.h(); ++y) {
		std::vector<double> row;
		for(int x = 0; x < im.w(); ++x) {
			row.push_back(im.get(x,y));
		} 
		row = dTrans1(row);
		for(int x = 0; x < im.w(); ++x) {
			im.set(x,y,row[x]);
		}
	}
}

void get_dist_transform(Image<double> & dist, Image<uint16_t> & im) {
	im.copyValid(dist.valid);
	dTransImage(dist);
	return;
	dist.data = std::vector<double>(640*480,0);
	//dist   = std::vector<double>(im.size(),0);
	std::vector<bool> visited  = std::vector<bool>(im.size(), false);
	std::vector<bool> new_visited = std::vector<bool>(im.size(), false);
	std::vector<bool> neighbor = std::vector<bool>(8, true);
	//List to visit
	std::vector<int> toVisit;
	std::vector<int> newToVisit;
	//Initialize large gradients to visited
	for(int i = 0; i < im.size(); ++i) {
		if(im.gradient[i].mag() > 50) {
			visited[i] = true;
		}
		if(im.isValid(i)) {
			toVisit.push_back(i);
		}
	}
	int num_times = 0;
	new_visited = visited;
	double cur_time = utime_now()/1000000.0;
	while(!toVisit.empty()) {
		newToVisit.clear();
		for(size_t i = 0; i < toVisit.size(); ++i) {
			num_times ++;
			int cur_id = toVisit[i];
			if(!visited[cur_id]) {
				std::vector<int> neighborIds = 
					im.getNeighborIds(im.getX(cur_id),im.getY(cur_id),
							neighbor,get_all);
				//Get min dist
				double min_d = 999999999;
				bool found_min = false;
				for(size_t j = 0; j < neighborIds.size(); ++j) {
					int id = neighborIds[j];
					if(visited[id] && dist.get(id) < min_d) {
						found_min = true;
						min_d = dist.get(id);
					}
				}
				if(found_min) {
					new_visited[cur_id] = true;
					dist.set(cur_id,min_d+1);
				} else {
					newToVisit.push_back(cur_id);
				}
			}
		}
		visited = new_visited;
		toVisit = newToVisit;
	}
	double new_time = utime_now()/1000000.0;
	//printf("time = %f ",new_time-cur_time);
	//printf("num_times = %d\n",num_times);
}

std::vector<pixel> minc_local_threshold(
		Image<double> & transf) {
	const double c = -9;
	const int num_wide = 1;
	std::vector<pixel> skeleton_pts;
	double mean;
	int num_valid;
	std::vector<int> neighbors;
	//std::vector<double> new_transf = transf.data;
	//Array that holds sum of neighbors value
	std::vector<double> neighbor_total = 
		std::vector<double>(num_wide*2+1, 0);
	std::vector<int> neighbor_num_valid = 
		std::vector<int>(num_wide*2+1, 0);

	/*
	for(int i = 0; i < transf.size(); ++i) {
		if(transf.isValid(i)) {
			neighbors = transf.getRColNeighborIds(i,num_wide);
			for(int j = 0; j < neighbors.size(); ++j) {
				if(transf.isValid(neighbors[j])) {
					mean += transf.gradient[neighbors[j]].mag();
				}
			}
		}
	}
	*/
	for(int i = 0; i < transf.size(); ++i) {
		mean = 0;
		num_valid = 0;
		if(transf.isValid(i)) {
			neighbors = transf.getBlockNeighborIds(i,num_wide);
			mean += transf.gradient[i].mag();
			num_valid ++;
			for(int j = 0; j < neighbors.size(); ++j) {
				if(transf.isValid(neighbors[j])) {
					//mean += transf.get(neighbors[j]);
					mean += transf.gradient[neighbors[j]].mag();
					num_valid ++;
				}
			}
			mean /= num_valid + 0.0;
			//printf("mean: %f\n",mean);
			if(transf.gradient[i].mag() < mean + c) {
				//new_transf[i] = 100;
				pixel_t px = {transf.getX(i), transf.getY(i)};
				skeleton_pts.push_back(px);
				transf.validate(i);
			} else {
				//new_transf[i] =  0;
				transf.invalidate(i);
			}
		}
	}
	//transf.data = new_transf;
	return skeleton_pts;
}

const int HOUGH_N = std::max(640,480);
const int HOUGH_THRESHOLD = 35;
const double theta_max = +M_PI/2;
const double theta_min = -theta_max;
const double theta_range = theta_max - theta_min;
const int    theta_num = 180*4; 

const double r_max     = +HOUGH_N*sqrt(2); 
const double r_min     = -r_max; 
const int    r_num     = HOUGH_N*4; 

std::vector<line_t> hough_transform(Image<double> & im) {
		//Array is row = r(adius), col = theta
	//Index = r_num*theta + r(adius)
	std::vector<int> hough_array = std::vector<int>(theta_num*r_num, 0);
	//Keep track of endpoints placed in bin
	std::vector<int> min_x       = std::vector<int>(theta_num*r_num, 640);
	std::vector<int> max_x       = std::vector<int>(theta_num*r_num, 0);

	std::vector<line_t> lines;
	
	//Array of ids that fit
	std::vector<int> points;
	for(int i = 0; i < im.size(); ++i) {
		if(im.isValid(i)) {
			points.push_back(i);
		}
	}

	//Run over each id and increment where it hits hough array
	for(int i = 0; i < points.size(); ++i) {
		int cur_id = points[i];
		double theta = im.gradient[cur_id].angle();
		
		if(theta > theta_max || theta < theta_min) {
			theta += M_PI;
			if(theta > theta_max) {
				theta = theta - 2*M_PI;
			}
			//printf("grad_ang:%f,min:%f,max:%f\n",
			//	theta, theta_min, theta_max);
		}
		for(theta = theta_min; theta <= theta_max; 
				theta += theta_range / theta_num) {
			int x = im.getX(cur_id);
			double radius = get_radius_from_xyt(
				   x, im.getY(cur_id), theta);	
			//Increment
			int id = getHoughId(theta,radius);
			hough_array[id] ++;
			//Update x ranges
			if(x < min_x[id]) {
				min_x[id] = x;
			}
			if(x > max_x[id]) {
				max_x[id] = x;
			}
		}
	}

	//Select those indices which have a large value
	for(int i = 0; i < hough_array.size(); ++i) {
		if(hough_array[i] > HOUGH_THRESHOLD) {
			double theta = getThetaFromHoughId(i);
			double radius= getRadFromHoughId(i);
			line_t tmp_line = getLineFrom_TR(theta,radius);
			tmp_line.ll.x = min_x[i];
			tmp_line.ll.y = tmp_line.ll.x*tmp_line.m + tmp_line.b;
			tmp_line.ru.x = max_x[i];
			tmp_line.ru.y = tmp_line.ru.x*tmp_line.m + tmp_line.b;
			lines.push_back(tmp_line);
			/*printf("line %d, m = %f, b = %f, <x=%d, >x=%d, theta:%f, r:%f, houghId:%d\n",
					lines.size()-1,tmp_line.m,tmp_line.b,
					tmp_line.ll.x, tmp_line.ru.x,
					theta, radius, i);*/
		}
	}
	
	return lines;
}

double get_radius_from_xyt(int x, int y, double theta) {
	return x*cos(theta) + y*sin(theta);
}

int getBox(double min, double max, int num, double val) {
	double iter_val = (max - min) / (num+0.0);
	int box =  (int) ((val - min) / iter_val);
	if(box >= num) {
		box = num - 1;
	} else if(box < 0) {
		box = 0;
	}
	assert(box >= 0 && box < num);
	return box;
}

double getValFromBoxNum(double min, double max, int num, int box) {
	double range = max - min;
	double iter_val = range / (num+0.0);
	//Returns half way
	return box * iter_val + min + iter_val/2.0;
}

int getHoughId(double theta, double radius) {
	assert(theta >= theta_min && theta <= theta_max);
	assert(radius >= r_min && radius <= r_max);
	int radius_box = getBox(r_min, r_max, r_num, radius);
	int theta_box  = getBox(theta_min, theta_max, theta_num,
			theta);
	int id = theta_box*r_num + radius_box; 
	if(id >= theta_num*r_num || id < 0) {
		/*printf("sucks to suck: t:%f,r:%f,id:%d,t_box:%d,r_box:%d,t_num:%d,r_num:%d",
				theta, radius, id, theta_box, radius_box,theta_num,r_num);*/
	}
	assert(radius_box >= 0 && radius_box < r_num);
	assert(theta_box  >= 0 &&  theta_box < theta_num);
	
	assert(id < theta_num*r_num && id >= 0);
	return id;
}

double getThetaFromHoughId(int i) {
	return getValFromBoxNum(theta_min, theta_max, theta_num, i/r_num);
}

double getRadFromHoughId(int i) {
	return getValFromBoxNum(r_min, r_max, r_num, i%r_num);
}

line_t getLineFrom_TR(double theta, double radius) {
	double tempX = 0.0, tempX2 = 5.0;
	double tempY, tempY2;
	tempY = (radius - tempX * cos(theta))/sin(theta);
	tempY2= (radius - tempX2* cos(theta))/sin(theta);
	double m = (tempY2 - tempY) / (tempX2 - tempX);
	double b = tempY;
	line_t line;
	line.m = m;
	line.b = b;
	return line;
}

double getThetaDist(double from, double to) {

	from = fmod(from, 2*M_PI);
	to   = fmod(to, 2*M_PI);
	double difference = fmod(to - from, 2*M_PI);
	double sn = sign(difference);
	if(fabs(difference) > M_PI) {
		difference = -sn*M_PI + fmod(difference, M_PI);
	}
	return difference;
}
double sign(double val) {
	return val < 0  ? -1.0 : 1.0;
}

