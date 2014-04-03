#ifndef __DISJOINT__H__
#define __DISJOINT__H__

class Set
{
    public:
	//Make sure don't initialize by pushing back onto vector
	//Could cause '*this' to be invalid
	Set(int x)
	    :parent(this),rank(0),val(x) {}
	Set unionS(Set *y);
	Set* findS();
	void printF();
	int get();

    private:
	void linkS(Set* y);

	Set *parent;
	int rank;
	int val;
};


	


#endif
