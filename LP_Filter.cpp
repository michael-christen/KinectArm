/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

      * File Name : LP_Filter.cpp

         * Purpose :

	    * Creation Date : 20-04-2014

	       * Last Modified : Sun 20 Apr 2014 04:42:59 PM EDT

	          * Created By : Michael Christen

		     _._._._._._._._._._._._._._._._._._._._._.*/
#include "LP_Filter.h"

LP_Filter::LP_Filter(int s) {
	size = s;
	total = 0;
}

int LP_Filter::getVal(int inVal) {
	//going to be larger, so remove
	if((int)data.size() > size) {
		int tail = data.front();
		data.pop();
		total -= tail;
	}
	data.push(inVal);
	total += inVal;
	return get();
}

int LP_Filter::get() {
	return (total+0.0) / (data.size()+0.0);
}
