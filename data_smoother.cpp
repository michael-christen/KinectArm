#include "data_smoother.h"

DataSmoother::DataSmoother(double dsf, double tsf, double initialVal, double initialTrend) {
	this->dsf = dsf;
	this->tsf = tsf;
	for (int i = 0; i < DS_NUMVALS; i++) {
		this->prevVal[i] = initialVal;
		this->prevTrend[i] = initialTrend;
	}
}

double DataSmoother::getNewVal(int id, double newVal) {
	double ret = this->dsf*newVal + (1 - this->dsf) * (this->prevVal[id] + this->prevTrend[id]);
	this->prevTrend[id] = this->tsf * (ret - this->prevVal[id]) + (1 - this->tsf) * this->prevTrend[id];
	this->prevVal[id] = ret;
	return ret;
}

void DataSmoother::setDSF(double dsf) {
	this->dsf = dsf;
}

double DataSmoother::getDSF() {
	return this->dsf;
}

void DataSmoother::setTSF(double tsf) {
	this->tsf = tsf;
}

double DataSmoother::getTSF() {
	return this->tsf;
}