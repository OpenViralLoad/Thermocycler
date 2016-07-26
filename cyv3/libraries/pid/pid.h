#ifndef PID_h
#define PID_h

class pid
{
    public:
        pid();
		void update(double sample, short target, double current_time);
		void setBiases(double p, double i, double d);
		double getM();
};

#endif
