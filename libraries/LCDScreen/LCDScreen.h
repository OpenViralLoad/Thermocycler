#ifndef LCDSCREEN_h
#define LCDSCREEN_h

class LCDScreen
{
    public:
        LCDScreen();
		void init();
        void setup();
		void displayInitialStatus( double temp, int current_cycle  );
		void dispDenatStatus(long int currentTime, long int stopTime);
		void updateArrow( double temp );
		void finalMess();	
		short retrieveTF();
		short retrieveTC();
		int retrieveMC();
		bool retrieveDN();
		void printError( int mess );

    private:
		void waitForSelect();
		void setUserInputs();
		void selectHighLowPush();
		void selectCycles();
		void dispUserPrefHighLow();
		void dispUserPrefCycles();
		void denaturization();
};

#endif
