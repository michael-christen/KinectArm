#include "data_smoother.h"

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