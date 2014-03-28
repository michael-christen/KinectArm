/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

   * File Name : disjoint.cpp

   * Purpose :

   * Creation Date : 23-04-2013

   * Last Modified : Fri 28 Mar 2014 05:45:06 PM EDT

   * Created By : Michael Christen

   _._._._._._._._._._._._._._._._._._._._._.*/
#include "disjoint.h"

void set_union(Set *a, Set *b)
{
    assert(a && b);
    set_link(set_find(a), set_find(b));
    return;
}

Set * set_init(int x) {
    Set * newSet = (Set *) malloc(sizeof(Set));
    newSet->parent = newSet;
    newSet->val = x;
    newSet->rank = 0;
    return newSet;
}

Set* set_find(Set *a)
{
    assert(a);
    if(a->parent  && a->parent != a) {
	assert(a->parent);
	a->parent = set_find(a->parent);
    }
    return a->parent;
}

void set_link(Set* a, Set *b)
{
    assert(a && b);
    if(a == set_find(b))
	return;
    if(set_find(b)->rank > a->rank)
	a->parent = set_find(b);
    else
    {
	set_find(b)->parent = a;
	if(set_find(b)->rank == a->rank)
	    a->rank += 1;
    }
    return;
}

void set_destroy(Set *set) {
	if(set) {
		free(set);
	}
}

