#include "pid_ctrl.h"

double sign(double val) {
	if(val < 0) return -1;
	if(val > 0) return 1;
	return 0;
}

void pid_init(pid_ctrl_t *pid, double P, double I, double D, double goal, double min_output, double max_val) {
	pid->min_output = min_output;
	pid->max_val = max_val;
    pid_update_pid(pid, P, I, D);
    pid_update_goal(pid, goal);
}

void pid_update_pid(pid_ctrl_t *pid, double P, double I, double D) {
    pid->P = P;
    pid->I = I;
    pid->D = D;
}

void pid_update_goal(pid_ctrl_t *pid, double goal) {
    pid->integral   = 0;
    pid->first_meas = 1;
    pid->goal       = goal;
    //Shouldn't be used first time
    pid->prev_clk   = clock();
    pid->prev_err   = 0;
}

double pid_get_output(pid_ctrl_t *pid, double meas) {
    clock_t cur_clock   = clock();
	double  cur_time    = utime_now()/1000000.0;
	//printf("clock: %d, prev_clock: %d, utime: %f\n",cur_clock, pid->prev_clk, utime_now()/1000000.0);
    //double dt           = (cur_clock - pid->prev_clk + 0.0)/CLOCKS_PER_SEC;
	double dt = cur_time - pid->prev_time;
    double err          = pid->goal - meas;
	//printf("%f, %f\n",dt,err);
	/*
	if(err < 0) {
		printf("ERROR is < 0\n");
	}
	else if(err > 0) {
		printf("ERROR is > 0\n");
	}
	*/
    double derivative   = 0;

    if(pid->first_meas) {
		pid->first_meas = 0;
    } else {
		//Don't want any nasty nan's
		pid->integral   += err*dt;
		if(dt == 0) {
			derivative = 0;
		} else {
			derivative = (err - pid->prev_err)/dt;
		}
		//printf("err: %f, prev_err: %f\n",err, pid->prev_err);
		//printf("derivative: %f, dt: %f\n",derivative, dt);
    }

	//If passes, get rid of integral
	if(sign(pid->prev_err) != sign(err)) {
		//printf("SWITCH\n");
		pid->integral = 0;
	}

	double proportion   = pid->P * err;
	double integral     = pid->I * pid->integral;
	double deriv_out    = pid->D * derivative;
	/*
	   printf("proportion: %f\n",proportion);
	   printf("integral  : %f\n",integral);
	   printf("derivative: %f\n",deriv_out);
	*/
    double output       = proportion +
		                  integral   +
						  deriv_out;

	if(fabs(output) > pid->max_val) {
		output = sign(output) * pid->max_val;
	}
	//STOP when within error bounds
    if(fabs(err) < pid->min_output) {
		pid->integral   = 0;
		output          = 0;
	}

    pid->prev_err       = err;
    pid->prev_clk       = cur_clock;
    pid->prev_time      = cur_time;
    return output;
}

//Map (-max_val, +max_val) -> (-1,1), ~actually want (-.14,.14)
//what to do when < min_movable, I will catch up, but will it
//take too long?
double pid_to_rot(pid_ctrl_t *pid, double pid_out) {
    //There
    if(fabs(pid_out) < pid->min_output) {
		//printf("there\n");
		return 0;
    }
	//Scale to (-1,1)
	pid_out /= pid->max_val;

	//Scale to factor
	pid_out *= -MAPPING_FACTOR;

    //printf("speed: %f\n",pid_out);
    return pid_out;
}

