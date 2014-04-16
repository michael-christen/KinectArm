#ifndef __IMAGE__H__
#define __IMAGE__H__
#include "common/image_util.h"
#include "Gradient.h"
#include <cassert>
#include <vector>

double GetRealWorldXFromDepth(uint16_t depth, int x); //returns real world X in mm, with reference to the center of the image as 0mm
double GetRealWorldYFromDepth(uint16_t depth, int y); //returns real world Y in mm, with reference to the center of the image as 0mm

template <typename T>
class Image {
	public:
		Image(int w, int h);
		~Image();

		T get(int x, int y);
		T get(int i);
		int getX(int i);
		int getY(int i);
		void set(int x, int y, T t);
		void set(int i, T t);
		//Update data starting at index=0 -> index=ts.size()-1
		void update(const std::vector<T> & ts);
		//Returns im, pointer, not really safe, but nice for display
		//Sets the image and returns it
		//Comparator takes T and valid bit, if add other things like gradient, this would take those too
		image_u32_t * getImage(uint32_t(*tToPX)(T, bool, Gradient,int));

		//Computes the gradient for the image
		void computeGradient(double(*tVal)(T,bool));

		void invalidate(int x, int y);
		void invalidate(int i);
		void validate(int x, int y);
		void validate(int i);
		void setValid(int x, int y, bool v);
		void setValid(int i, bool v);
		bool isValid(int x, int y);
		bool isValid(int i);
		void copyValid(std::vector<bool> & otherValid);
		//Search
		//***  012
		//*.*  3.4
		//***  567
		//Vector of neighbors
		//Pass comparator to tell wheter neighbor
		//And bit vector corresponding to above diagram
		std::vector<T> getNeighbors(int x, int y,
				const std::vector<bool> & u_valid,
				bool(*is_neighbor)(T, T)
		);
		std::vector<int> getNeighborIds(int x, int y,
				const std::vector<bool> & u_valid,
				bool(*is_neighbor)(T, T)
		);
		std::vector<int> getNeighborIds(int x, int y, 
				const std::vector<bool> & u_valid,
				bool(*is_neighbor)(Gradient, Gradient));
		//Get nxn grid of id's centered at i
		std::vector<int> getBlockNeighborIds(int i, int n);

		size_t size();
		bool   empty();
		int w();
		int h();
		void printGradient();
		void printGradient(int i);
		int id(int x, int y);
		std::vector<bool> valid;
		//Gradient information
		std::vector<Gradient> gradient;
		std::vector<T> data;
	private:
		int width;
		int height;
		//Width*height vector containing our data
		//Access to (x,y) -> x + width*y
		//Valid bits to keep track of validness 
		image_u32_t * im;
};

template <typename T>
Image<T>::Image(int w, int h) 
:width(w), height(h) {
	data.resize(w*h);
	valid = std::vector<bool>(w*h, true);
	gradient = std::vector<Gradient>(w*h, Gradient());
	im = image_u32_create(w, h);
	/*
	printf("Im w: %d, h: %d",
			im->width,
			im->height);
			*/
}

template <typename T>
Image<T>::~Image() {
	//image_u32_destroy(im);	
}

template <typename T>
T Image<T>::get(int x, int y) {
	return data[id(x,y)];
}

template <typename T>
int Image<T>::getY(int i) {
	assert(i >= 0 && i < (int)data.size());
	return i/width;
}

template <typename T>
int Image<T>::getX(int i) {
	assert(i >= 0 && i < (int)data.size());
	return i%width;
}

template <typename T>
T Image<T>::get(int i) {
	assert(i >= 0 && i < (int)data.size());
	return data[i];
}

template <typename T>
int Image<T>::w() {
	return width;
}

template <typename T>
int Image<T>::h() {
	return height;
}

template <typename T>
void Image<T>::set(int x, int y, T t) {
	data[id(x,y)] = t;
}

template <typename T>
void Image<T>::set(int i, T t) {
	assert(i >= 0 && i < (int)data.size());
	data[i] = t;
}

template <typename T>
size_t Image<T>::size() {
	return width*height;
}

template <typename T>
bool Image<T>::empty() {
	return size() == 0;
}

template <typename T>
void Image<T>::update(const std::vector<T> & ts) {
	assert(ts.size() == data.size());
	data = ts;
	valid.assign(size(),true);
	//gradient.assign(size(),Gradient());
}

template <typename T>
image_u32_t * Image<T>::getImage(uint32_t(*tToPX)(T, bool, Gradient,
			int)) {
	for(int x = 0; x < im->width; ++x) {
		for(int y = 0; y < im->height; ++y) {
			/*
			if(gradient[id(x,y)].x() || 
					gradient[id(x,y)].y()) {
				printf("gradient @ (%d,%d)\n",x,y);
				gradient[id(x,y)].print();
			}
			*/
			im->buf[x+y*im->stride] = tToPX(
					get(x,y),
					valid[id(x,y)],
					gradient[id(x,y)],
					id(x,y)
			);
		}
	}
	//printGradient();
	return im;
}

template <typename T>
void Image<T>::computeGradient(double(*tVal)(T,bool)) {
	//Leave far edges of image with gradient = 0, to avoid bounds
	//checking
	for(int x = 1; x < width-1; ++x) {
		for(int y = 1; y < height-1; ++y) {
			//printf("(x+1,y):%d\n", tVal(get(x+1,y)));
			//printf("(x+1,y):%f\n",tVal(get(x+1,y)) - tVal(get(x-1,y)));
			gradient[id(x,y)].x( 
					tVal(get(x+1,y),valid[id(x+1,y)]) - 
					tVal(get(x-1,y),valid[id(x-1,y)]) 
			);
			//printf("(x+1,y):%f\n",gradient[id(x,y)].x());
			gradient[id(x,y)].y( 
					tVal(get(x,y+1),valid[id(x,y+1)]) - 
					tVal(get(x,y-1),valid[id(x,y-1)]) 
			);
			//gradient[id(x,y)].print();
		}
	}
	//printGradient();
}


template <typename T>
void Image<T>::invalidate(int i) {
	setValid(i,false);
}

template <typename T>
void Image<T>::invalidate(int x, int y) {
	setValid(x,y,false);
}

template <typename T>
void Image<T>::validate(int x, int y) {
	setValid(x,y,true);
}

template <typename T>
void Image<T>::validate(int i) {
	setValid(i,true);
}

template <typename T>
void Image<T>::setValid(int x, int y, bool v) {
	valid[id(x,y)] = v;
}

template <typename T>
void Image<T>::setValid(int i, bool v) {
	assert(i >= 0 && i < (int)valid.size());
	valid[i] = v;
}

template <typename T>
bool Image<T>::isValid(int x, int y) {
	return valid[id(x,y)];
}

template <typename T>
bool Image<T>::isValid(int i) {
	assert(i >= 0 && i < (int)valid.size());
	return valid[i];
}

template <typename T>
void Image<T>::copyValid(std::vector<bool> &otherValid) {
	otherValid = valid;
}

template <typename T>
int Image<T>::id(int x, int y) {
	assert(x >= 0 && x < width);
	assert(y >= 0 && y < height);
	return y*width + x;
}

template <typename T>
void Image<T>::printGradient(int i) {
	gradient[i].print();
}

template <typename T>
void Image<T>::printGradient() {
	for(size_t i = 0; i < size(); ++i) {
		gradient[i].print();
	}
}

template <typename T>
std::vector<int> Image<T>::getNeighborIds(int x, int y, 
		const std::vector<bool> & u_valid,
		bool(*is_neighbor)(T, T)
		) {

	assert(u_valid.size() == 8);

	std::vector<int> neighbors;
	T current = data[id(x,y)];
	int index = 0;
	for(int j = -1; j <= 1; ++j) {
		for(int i = -1; i <= 1; ++i) {
			//edge
			if(j + y < 0 || j + y >= height) {
				index ++;
				continue;
			}
			//edge
			if(i + x < 0 || i + x >= width) {
				index ++;
				continue;
			}
			//Don't return middle
			if(j == 0 && i == 0) {
				//Don't increment index
				continue;
			}
			size_t n = id(x+i,y+j);
			T   t = data[n];
			if(valid[n] && u_valid[index]
				&& is_neighbor(current,t)) {
				assert(n >= 0 && n < size());
				neighbors.push_back(n);
			}
			index ++;
		}
	}
	return neighbors;
}

template <typename T>
std::vector<int> Image<T>::getNeighborIds(int x, int y, 
		const std::vector<bool> & u_valid,
		bool(*is_neighbor)(Gradient, Gradient)
		) {

	assert(u_valid.size() == 8);

	std::vector<int> neighbors;
	Gradient current = gradient[id(x,y)];
	int index = 0;
	for(int j = -1; j <= 1; ++j) {
		for(int i = -1; i <= 1; ++i) {
			//edge
			if(j + y < 0 || j + y >= height) {
				index ++;
				continue;
			}
			//edge
			if(i + x < 0 || i + x >= width) {
				index ++;
				continue;
			}
			//Don't return middle
			if(j == 0 && i == 0) {
				//Don't increment index
				continue;
			}
			size_t n = id(x+i,y+j);
			Gradient   t = gradient[n];
			if(valid[n] && u_valid[index]
				&& is_neighbor(current,t)) {
				assert(n >= 0 && n < size());
				neighbors.push_back(n);
			}
			index ++;
		}
	}
	return neighbors;
}

template <typename T>
std::vector<T> Image<T>::getNeighbors(int x, int y, 
		const std::vector<bool> & u_valid,
		bool(*is_neighbor)(T, T)
		) {
	std::vector<int> neighbors =
		getNeighborIds(x,y,u_valid,is_neighbor);
	std::vector<T> v_neighbors;
	for(size_t i = 0; i < neighbors.size(); ++i) {
		v_neighbors.push_back(get(neighbors[i]));
	}
	return v_neighbors;
}

template <typename T>
std::vector<int> Image<T>::getBlockNeighborIds(int i, int n) {
	std::vector<int> neighbors;
	int cur_x = getX(i);
	int cur_y = getY(i);

	for(int y = -n/2; y <= n/2; ++y) { 
		if(cur_y + y < 0 || cur_y + y >= height) {
			continue;
		}
		for(int x = -n/2; x <= n/2; ++x) { 
			//edge
			if(cur_x + x < 0 || cur_x + x >= width) {
				continue;
			}
			//Don't return middle
			if(cur_y == 0 && cur_x == 0) {
				//Don't increment index
				continue;
			}
			neighbors.push_back(id(cur_x + x, cur_y + y));
		}
	}
	return neighbors;
}
#endif
