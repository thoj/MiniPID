#ifndef MINIPID_H
#define MINIPID_H

class MiniPID{
public:
	MiniPID(double, double, double);
	MiniPID(double, double, double, double);
	void setP(double);
	void setI(double);
	void setD(double);
	void setF(double);
	void setPID(double, double, double);
	void setPID(double, double, double, double);
	void setMaxIOutput(double);
	void setOutputLimits(double);
	void setOutputLimits(double,double);
	void setDirection(bool);
	void setSetpoint(double);
	void reset();
	void setOutputRampRate(double);
	void setSetpointRange(double);
	void setOutputFilter(double);
	void setFeedForwardValue(double);
	void setFeedForwardValue(bool);
	double getOutput();
	double getOutput(double);
	double getOutput(double, double);

private:
	double clamp(double, double, double);
	bool bounded(double, double, double);
	void checkSigns();
	void init();
	double P;
	double I;
	double D;
	double F;

	double maxIOutput;
	double maxError;
	double errorSum;

	double maxOutput; 
	double minOutput;

	double setpoint;

	double lastActual;
	
	double feedForwardValue;

	bool firstRun;
	bool reversed;
	bool useFeedForwardValue;

	double outputRampRate;
	double lastOutput;

	double outputFilter;

	double setpointRange;
};
#endif
