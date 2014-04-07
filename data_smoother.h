
class DataSmoother {
	public:
		DataSmoother(double dsf, double tsf, double initialVal, double initialTrend);
		double getNewVal(double newVal);
		void setDSF(double dsf);
		double getDSF();
		void setTSF(double tsf);
		double getTSF();

	private:
		double dsf, tsf;
		double prevVal;
		double prevTrend;
}