/* -.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.-.

      * File Name : Line.cpp

         * Purpose :

	    * Creation Date : 05-04-2014

	       * Last Modified : Sat 05 Apr 2014 07:13:56 PM EDT

	          * Created By : Michael Christen

		     _._._._._._._._._._._._._._._._._._._._._.*/
#include "Line.h"
#include "common/matd.h"

line_t linear_regression(Blob<Gradient> &blob) {
	line_t line;
	//Perform linear regression
	//Ax = B
	//A = [1 X_0],[1 X_1]
	//B = [Y_0]  ,[Y_1]
	//x = [ b, m]
	matd_t *A = matd_create(blob.size(),2);
	matd_t *B = matd_create(blob.size(),1);
	int leftmost, rightmost;
	leftmost = rightmost = blob.getPos(0).x;
	//Convert to Matd
	for(size_t i = 0; i < blob.size(); ++i) {
		pos_t tmp = blob.getPos(i);
		if(tmp.x < leftmost) {
			leftmost = tmp.x;
		}
		if(tmp.x > rightmost) {
			rightmost = tmp.x;
		}

		MATD_EL(A, i, 0) = 1;
		MATD_EL(A, i, 1) = tmp.x;
		MATD_EL(B, i, 0) = tmp.y;
	}
	matd_t *x = matd_op("(M' * M)^-1 * M' * M",
			A,A,A,B);
	line.b = MATD_EL(x,0,0);
	line.m = MATD_EL(x,1,0);

	line.ll.x = leftmost;
	line.ll.y = line.m*leftmost + line.b;

	line.ru.x = rightmost;
	line.ru.y = line.m*rightmost + line.b;

	line.num_pts = blob.size();
	//Compute difference of hypothetical vs. real
	matd_t *diff = matd_op("(M*M) - M",
			A,x,B);
	//Sum of squares
	matd_t *var  = matd_op("M' * M",diff,diff);
	line.variance = MATD_EL(var,0,0);

	//Clean up
	matd_destroy(A);
	matd_destroy(B);
	matd_destroy(x);
	matd_destroy(diff);
	matd_destroy(var);
	return line;
}

