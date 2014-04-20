#ifndef DATA_SMOOTHER_H
#define DATA_SMOOTHER_H

#define DS_NUMVALS 50

class DataSmoother {
	public:
		DataSmoother(double dsf, double tsf, double initialVal, double initialTrend);
		double getNewVal(int id, double newVal);
		void setDSF(double dsf);
		double getDSF();
		void setTSF(double tsf);
		double getTSF();

	private:
		double dsf, tsf;
		double prevVal[DS_NUMVALS];
		double prevTrend[DS_NUMVALS];
};

#endif