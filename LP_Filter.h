#ifndef __LP_Filter__H__
#define __LP_Filter__H__
#include<queue>

class LP_Filter {
	public:
		LP_Filter(int s);
		int getVal(int inVal);
		int get();
	private:
		std::queue<int> data;
		int size;
		int total;
};

#endif
