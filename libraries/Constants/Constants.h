#ifndef CONSTANTS_h
#define CONSTANTS_h

// CYCLES
extern int CYCLES				  = 20;

// TEMPERATURES
extern short TEMP_CEIL			  = 95;
extern short TEMP_FLOOR			  = 60;
extern short TEMP_CRIT            = TEMP_CEIL + 20;
extern const short TEMP_FINAL	  = 50;

// DURATIONS
extern const int MAINT_DENAT      = 600000;
extern const int MAINT_CEIL       = 30000;
extern const int MAINT_FLOOR      = 60000;
extern const int PRINT_DUR		  = 1000; 
extern const int ERROR_DUR_MAX	  = 5000; // 5 second stall indicates problem

// PID Biases
extern double PID_P				  = 1;
extern double PID_I				  = 1;
extern double PID_D				  = 1;
extern double PID_M_MAX			  = 255;
extern double PID_M_MIN           = 0;

// Toggles
extern const bool DEBUG           = false;
extern const bool ERROR_CHECK     = false;
extern bool DENAT 		  		  = true;

// PINS
extern const short pin_therm      = 0;
extern const short pin_heater     = 4;
extern const short pin_fan        = 5;
extern const short led_red        = 3;
extern const short led_blue       = 2;


#endif
