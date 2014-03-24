#ifndef __DISJOINT__H__
#define __DISJOINT__H__
#include<stdlib.h>
#include<assert.h>

typedef struct Set Set;
struct Set
{
    	Set *parent;
	//distinguishes size
	int rank;
	//label
	int val;
};

Set * set_init (int x);
void  set_destroy(Set *set);
//Sets both a and b -> a | b
void set_union(Set *a, Set *b);
//Returns the topmost parent of this set
Set * set_find(Set *a);
//links 2 sets together
void set_link(Set *a, Set *b);






#endif
