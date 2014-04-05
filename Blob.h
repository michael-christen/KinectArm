#ifndef __BLOB__H__
#define __BLOB__H__
#include<iostream>
#include<vector>
struct pos_t {
	int x, y;
};

template <typename T>
class Blob {
	public:
		void push(int x, int y, T data);
		size_t size();
		T getData(int i);
		pos_t getPos(int i);
	private:
		std::vector<T> blob_data;
		std::vector<pos_t> blob_pos;
};

template <typename T>
void Blob<T>::push(int x, int y, T data) {
	blob_data.push_back(data);
	pos_t tmp = {x,y};
	blob_pos.push_back(tmp);
}

template <typename T>
size_t Blob<T>::size() {
	return blob_data.size();
}

template <typename T>
T Blob<T>::getData(int i) {
	assert(i < blob_data.size() && i >= 0);
	return blob_data[i];
}

template <typename T>
pos_t Blob<T>::getPos(int i) {
	assert(i < blob_data.size() && i >= 0);
	return blob_pos[i];
}
#endif
