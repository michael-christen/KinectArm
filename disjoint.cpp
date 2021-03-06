/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

   * File Name : disjoint.cpp

   * Purpose :

   * Creation Date : 23-04-2013

   * Last Modified : Thu 03 Apr 2014 12:50:18 AM EDT

   * Created By : Michael Christen

   _._._._._._._._._._._._._._._._._._._._._.*/
#include<iostream>
#include "disjoint.h"
using namespace std;

Set Set::unionS(Set *y)
{
    linkS(y->findS());
    return *this;
}

Set* Set::findS()
{
	if(parent != this) {
		parent = parent->findS();
	}
    return parent;
}

void Set::printF()
{
    if(parent != this) {
		parent = findS();
	}
    cout << parent->val << endl;
}

int Set::get() {
	return val;
}

void Set::linkS(Set* y)
{
    if(y == findS()) {
		return;
	}
    if(findS()->rank > y->rank) {
		y->parent = findS();
	}
	else {
		findS()->parent = y;
		if(findS()->rank == y->rank) {
			y->rank += 1;
		}
	}
    return;
}

