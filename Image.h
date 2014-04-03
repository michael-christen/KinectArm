#ifndef __IMAGE__H__
#define __IMAGE__H__
#include "common/image_util.h"
#include <cassert>
#include <vector>

template <typename T>
class Image {
	public:
		Image(int w, int h);
		~Image();

		T get(int x, int y);
		T get(int i);
		void set(int x, int y, T t);
		//Update data starting at index=0 -> index=ts.size()-1
		void update(const std::vector<T> & ts);
		//Returns im, pointer, not really safe, but nice for display
		//Sets the image and returns it
		//Comparator takes T and valid bit, if add other things like gradient, this would take those too
		image_u32_t * getImage(uint32_t(*tToPX)(T, bool));

		void invalidate(int x, int y);
		void invalidate(int i);
		void validate(int x, int y);
		void setValid(int x, int y, bool v);
		void setValid(int i, bool v);
		bool isValid(int x, int y);
		void copyValid(Image & otherIm);
		//Search
		//***  012
		//*.*  3.4
		//***  567
		//Vector of neighbors
		//Pass comparator to tell wheter neighbor
		//And bit vector corresponding to above diagram
		std::vector<T> getNeighbors(int x, int y,
				const std::vector<bool> & u_valid,
				bool(*is_neighbor)(uint16_t, uint16_t)
		);
		std::vector<int> getNeighborIds(int x, int y,
				const std::vector<bool> & u_valid,
				bool(*is_neighbor)(uint16_t, uint16_t)
		);

		size_t size();
		int w();
		int h();
	private:
		int width;
		int height;
		int id(int x, int y);
		//Width*height vector containing our data
		//Access to (x,y) -> x + width*y
		std::vector<T> data;
		//Valid bits to keep track of validness 
		std::vector<bool> valid;
		image_u32_t * im;
};

template <typename T>
Image<T>::Image(int w, int h) 
:width(w), height(h) {
	data.resize(w*h);
	valid = std::vector<bool>(w*h, true);
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
size_t Image<T>::size() {
	return width*height;
}

template <typename T>
void Image<T>::update(const std::vector<T> & ts) {
	assert(ts.size() == data.size());
	data = ts;
	valid.assign(size(),true);
}

template <typename T>
image_u32_t * Image<T>::getImage(uint32_t(*tToPX)(T, bool)) {
	for(int x = 0; x < im->width; ++x) {
		for(int y = 0; y < im->height; ++y) {
			im->buf[x+y*im->stride] = tToPX(get(x,y),valid[id(x,y)]);
		}
	}
	return im;
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
void Image<T>::copyValid(Image & otherIm) {
	for(size_t i = 0; i < valid.size(); ++i) {
		otherIm.setValid(i,valid[i]);
	}
}

template <typename T>
int Image<T>::id(int x, int y) {
	assert(x >= 0 && x < width);
	assert(y >= 0 && y < height);
	return y*width + x;
}

template <typename T>
std::vector<int> Image<T>::getNeighborIds(int x, int y, 
		const std::vector<bool> & u_valid,
		bool(*is_neighbor)(uint16_t, uint16_t)
		) {

	assert(u_valid.size() == 8);

	std::vector<int> neighbors;
	T current = data[id(x,y)];
	int index = 0;
	for(int j = -1; j <= 1; ++j) {
		//edge
		if(j + y < 0 || j + y >= height) {
			index ++;
			continue;
		}
		for(int i = -1; i <= 1; ++i) {
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
std::vector<T> Image<T>::getNeighbors(int x, int y, 
		const std::vector<bool> & u_valid,
		bool(*is_neighbor)(uint16_t, uint16_t)
		) {
	std::vector<int> neighbors =
		getNeighborIds(x,y,u_valid,is_neighbor);
	std::vector<T> v_neighbors;
	for(size_t i = 0; i < neighbors.size(); ++i) {
		v_neighbors.push_back(get(neighbors[i]));
	}
	return v_neighbors;
}
#endif
