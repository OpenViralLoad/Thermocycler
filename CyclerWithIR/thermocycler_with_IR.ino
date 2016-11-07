/*
 * ARDUINO UNO THERMAL CYCLER WITH LCD DISPLAY
 * For use with the Polymerase Chain Reaction (PCR) process
 *
 * A Global Ties - Engineering World Health Team Project
 *
 */

#include <Adafruit_MLX90614.h>
#include <avr/sleep.h>
#include <math.h>
#include "LCDScreen.h"
#include "Adafruit_RGBLCDShield.h"
#include <Wire.h>

#define pin_thermistorA0 0
#define pin_thermistorA1 1  
#define pin_heater_PWR   4
#define pin_fan_PWR      5

#define led_red          3
#define led_blue         2

LCDScreen UI = LCDScreen(); 	// LCD display

double TEMP_CEILING;		// Upper and
double TEMP_FLOOR;		// lower temperature limits to cycle between
double FINAL_FLOOR = 50; 	// Temperature to drop to on wrap()
double CRITICAL_MAX;
double WARNING_LIMIT  = 50;

int MAX_CYCLES;                 // Cycles to perform
long int MAINTAIN_DURATION = 0;
long int MAINTAIN_DENAT = 1000*60*10; //Hold at hot temp for 10 minutes for denaturization
long int MAINTAIN_CEILING =  1000*35; // Hold at hot temperature for 35 seconds
long int MAINTAIN_FLOOR = 1000*65; // Hold at cold temperature for 65 seconds
boolean DENAT;

int PRINT_DELAY       =  1000*1;  // Print to serial monitor once every second (1000 ms)
boolean DEBUG =  false; 	// Change to true to test fan and heater before running
boolean ERRORCHECK = false;
double PCR_time_start;		// Start time of process

// State of the System
double current_temp;
double current_time;
int current_cycle;
String current_stage;
// setup
// denat
// rampup
// maintain
// rampdown
// wrapup
// sleep

double A;
double B;
double C;

Adafruit_MLX90614 infraredSensor = Adafruit_MLX90614();

void update() {
  //current_temp = Thermistor( analogRead(pin_thermistorA0), analogRead(pin_thermistorA1 ));
  current_temp = infraredSensor.readObjectTempC();
  current_time = millis();
}

void setup()
{

  Serial.begin( 9600 );      // Set the data rate (baud) with which uno comms with comp

  Serial.println(F( "Setting up ... " ));
  current_stage = "setup";

  /* Heater and Fan are both outputs */
  pinMode( pin_heater_PWR, OUTPUT );
  pinMode( pin_fan_PWR,    OUTPUT );

  /* initial status of Heater and Fan */
  digitalWrite ( pin_heater_PWR, LOW    ); // Heater is on but not running
  digitalWrite ( pin_fan_PWR,    LOW    ); // Fan is off

  current_cycle = 0;			// No cycles complete yet.
  
  //current_temp = Thermistor( analogRead( pin_thermistorA0 ), analogRead(pin_thermistorA1) );
  current_temp = infraredSensor.readObjectTempC();
  if ( DEBUG ) 				// Test if fan and heater are running
  {

    Serial.println(F( "Testing heater ... " ));
    RampUp( current_temp + 10 );

    Serial.println(F( "Testing fan ... " ));
    RampDown( current_temp - 5 );

    DEBUG = false;

    setLight( 0 );
  }

  heaterOff();				// In case of reset
  fanOff();

  UI.init();				// Boot up LCD screen
  UI.setup();  				// Allow user to select preferences

  /* Collect temperature preferences from UI */
  TEMP_FLOOR = UI.retrieveTF();
  TEMP_CEILING = UI.retrieveTC();
  CRITICAL_MAX = TEMP_CEILING + 20;
  MAX_CYCLES = UI.retrieveMC();
  DENAT = UI.retrieveDN();


  Serial.println(F( "Setup complete.\n" ));
  Serial.println(F(" Current Settings: "));
  Serial.print(F("Denaturization: " ));
  if (DENAT) {
    Serial.println(F("TRUE"));
  }
  else Serial.println(F("FALSE"));
  Serial.print(F("Ceiling Temperature: "));
  Serial.print(TEMP_CEILING);
  Serial.print(F(" C for "));
  Serial.print(MAINTAIN_CEILING/1000);
  Serial.println(F(" seconds"));
  Serial.print(F("Floor Temperature: "));
  Serial.print(TEMP_FLOOR);
  Serial.print(F(" C for "));
  Serial.print(MAINTAIN_FLOOR/1000);
  Serial.println(F(" seconds"));

  PCR_time_start = millis();		// Start the clock

}

void loop()
{
  Denaturization( TEMP_CEILING );  // State 0: Denaturize samply by heating to TEMP_CEILING and Maintaining for 10 min

  if ( current_cycle < MAX_CYCLES )	// More cycles to go
  {
    //UI.displayInitialStatus( Thermistor( analogRead( pin_thermistorA0 ), analogRead(pin_thermistorA1) ), current_cycle + 1 );
    UI.displayInitialStatus(infraredSensor.readObjectTempC(), current_cycle + 1);
    Serial.print(F( "Current Cycle: " ));
    Serial.print( ++current_cycle );
    Serial.print(F( "/" ));
    Serial.println( MAX_CYCLES );
    Serial.print(F( "( Cycling between " ));
    Serial.print( TEMP_FLOOR );
    Serial.print(F( " and " ));
    Serial.print( TEMP_CEILING );
    Serial.println(F( " )" ));

    RampUp( TEMP_CEILING ); 		// State 1: Heat up sample to TEMP_CEILING
    Maintain( TEMP_CEILING );		// State 2: Keep sample heated at TEMP_CEILING for MAINTAIN_DURATION
    RampDown( TEMP_FLOOR );		// State 3: Cool down sample to TEMP_FLOOR
    Maintain( TEMP_FLOOR );   //State 2; Keep at Temp floor for duration

  }
  else if ( current_cycle == MAX_CYCLES ) // All cycles completed
  {
    Serial.print( current_cycle++ );
    Serial.println(F(" cycles completed." ));

    wrapUp();				// State 4: Cool sample close to room temperature, turn off fan and heater

    double total_running_time = ( millis() - PCR_time_start ) / 1000; // Get final running time in seconds

    Serial.print(F( "PCR process completed in " ));
    Serial.print( total_running_time );
    Serial.println(F( " seconds." ));

    UI.finalMess();			// Print goodbye message to LCD

    sleep();				// State 5: Sleep, do nothing until reset

  }
  else
  {
    // do nothing
  }

}

/*
 * STATE 0
 * Denaturization
 *
 */
void Denaturization( int target )
{
  current_stage = "denat";
  if (DENAT)
  {
    double start = millis();
    Serial.println( F("Entering Denaturization Stage..." ));
    UI.dispDenatStatus(0, 0);
    RampUp( TEMP_CEILING ); //Ramp up and maintain
    Maintain( TEMP_CEILING ); //Hold for denaturization stage
    DENAT = 0;
    double totalTime = (millis()- start)/1000;
    Serial.print( F("Denaturization finished in " ));
    Serial.print( totalTime );
    Serial.println( F( " seconds.\n") );

  }
  else {
    //Skip this step
  }
}
/*
 * STATE 1
 * Heat up the sample to target ceiling temperature
 *
 */
void RampUp( int target )
{
  current_stage = "rampup";
  Serial.println( F("Ramping up ... ") );
  double startTime = millis();
  
  /* Get current temperature */
  //current_temp = Thermistor( analogRead( pin_thermistorA0 ), analogRead(pin_thermistorA1) );
  current_temp = infraredSensor.readObjectTempC();
  setLight( 1 ); // red
  analogWrite( pin_heater_PWR, 255 );

  int warning = 0;
  double timeToPrint = 0;
  while ( current_temp < target )
  {
    /* Refresh current temperature */
    //double newTemp = ( Thermistor( analogRead( 0 ), analogRead(1) ) );
    double newTemp = infraredSensor.readObjectTempC();

    UI.updateArrow( newTemp );
    /* Compare to previous temperature, should increase */
    if ( newTemp <= current_temp )
    {
      if ( ERRORCHECK ) {
        if ( warning > WARNING_LIMIT )
        {
          Serial.println( F("Error: Temperature not increasing on RampUp().") );
          Serial.println( F("Please make sure device is correctly wired.") );
          heaterOff();
          UI.printError( 0 );
          setLight( 3 ); // yellow
          while ( true ) {
          }
        }
        else
        {
          warning++;
        }
      }
    }
    else if ( newTemp > CRITICAL_MAX )
    {
      Serial.println( F("Error: Temperature surpassing critical maximum") );
      Serial.println( F("Please make sure heater is functioning correctly") );
      heaterOff();
      UI.printError( 1 );
      setLight( 3 ); // yellow
      while ( true ) {
      }
    }
    else {
      warning = 0;
    }

    current_temp = newTemp;

    timeToPrint = checkPrint( timeToPrint, current_temp );

  }

  heaterOff();
  double totalTime = ( millis() - startTime ) / 1000;
  Serial.print(F( "RampUp finished in " ));
  Serial.print( totalTime );
  Serial.println( F(" seconds.\n" ));

}


void PID_Redux( int setpt ) {
 static double w_error;
 static double w_temp;
 static double w_setpt;
 static double w_time;
 static double kp, ki, kd;
 
 // Check if setpt/target is the same as last check
 if (setpt != w_setpt) w_error = w_temp = w_time = 0;
 w_setpt = setpt;
 
 update();
 double n_error = current_temp - w_setpt;
 double d_error = n_error - w_error;
 double d_t     = current_time - w_time;
 double deriv   = d_error/d_t;
 double integ;

 w_time = current_time; 
 
  
}

/*
 * STATE 2
 * Keep the sample heated at target ceiling temperature for a specified duration
 *
 */
void Maintain( int target )
{
  boolean first_run = true;
  int maintain_duration = 10*1000;
  Serial.println( "Maintaining temperature ..." );
  double time = millis();
  double stopTime = time + maintain_duration;
  double previous = 0;
  double nextPrint = time + PRINT_DELAY;
  setLight( 1 ); // red
  while ( time < stopTime )
  {
    time = millis();
    if(DENAT){
      if (time - previous >= 10000){
        previous = time;
        UI.dispDenatStatus(time, stopTime);
      }
    }
    nextPrint = PID( infraredSensor.readObjectTempC(), target, first_run, time, nextPrint );
    first_run = false;
  }
  setLight( 0 ); // green
  Serial.println( "\n" );
}

/*
 * STATE 3
 * Cool down the sample to target floor temperature
 *
 */
void RampDown( int target )
{
  current_stage="rampdown";
  double startTime = millis();
  double ActualTemp = infraredSensor.readObjectTempC();

  analogWrite( pin_fan_PWR, 255 );
  analogWrite( pin_heater_PWR, 0 );
  Serial.println( F("Ramping down ... ") );

  double timeToPrint = 0;
  int warning = 0;
  while ( ActualTemp >= TEMP_FLOOR )
  {
    double newTemp = infraredSensor.readObjectTempC();
    UI.updateArrow( newTemp );
    if ( ActualTemp < newTemp )
    {
      if ( ERRORCHECK ) {

        if ( warning > WARNING_LIMIT )
        {
          Serial.println( F( "Error: Temperature not decreasing on RampDown()." ) );
          Serial.println( F( "Please make sure that device is correctly wired." ) );
          fanOff();
          UI.printError( 0 );
          while ( true ) {
          }
        }
        else {
          warning++;
        }
      }
    }
    else {
      warning = 0;
    }

    ActualTemp = newTemp;

    timeToPrint = checkPrint( timeToPrint, ActualTemp );

  }

  fanOff();

  double totalTime = ( millis() - startTime ) / 1000;
  Serial.print(F( "RampDown finished in " ));
  Serial.print( totalTime );
  Serial.println( F(" seconds.\n") );

}

/*
 * STATE 4
 * Cool the sample to close to room temperature, turn off all outputs
 *
 */
void wrapUp()
{
  current_stage="wrapup";
  analogWrite( pin_heater_PWR, 0 );

  Serial.println( F("Wrapping up...") );
  double ActualTemp = infraredSensor.readObjectTempC();

  setLight( 2 );
  analogWrite( pin_fan_PWR, 255 );

  double timeToPrint = 0;
  int warning = 0;
  while ( ActualTemp > FINAL_FLOOR )
  {
    int newTemp = infraredSensor.readObjectTempC();

    if ( ActualTemp < newTemp )
    {
      if ( ERRORCHECK ) {
        if ( warning > WARNING_LIMIT )
        {
          Serial.println( F("Error: Temperature not decreasing on WrapUp().") );
          Serial.println( F("Check log for details.") );
          fanOff();
          UI.printError( 2 );
          setLight( 3 );
          while ( true ) { }
        }
        else {
          warning++;
        }
      }
    }
    else {
      warning = 0;
    }

    ActualTemp = newTemp;

    timeToPrint = checkPrint( timeToPrint, ActualTemp );

  }

  analogWrite( pin_fan_PWR, 0 );
  setLight( 0 );
  Serial.print(F( "Wrap finished. Fan is turning off. Current temperature: ") );
  Serial.print(ActualTemp );
  Serial.println( F("\n") );
}

/*
 * STATE 5
 * Sleep until external reset
 *
 */
void sleep()
{
  current_stage="sleep";
  Serial.println( F("Turning off heater and fan ... \n") );
  analogWrite( pin_heater_PWR, 0 );         // Heater is on but not running
  analogWrite( pin_fan_PWR, 0);

  Serial.println( F("Putting device to sleep. Please restart the device.") );

  setLight( 0 );

  set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  cli();
  sei();
  sleep_cpu();

}

double PID( double Actual, double SetPt, boolean reset, double currentTime, double nextPrint )
{  
  double kP=100, kI=0, kD=10;
  
  static double Previous_Error = 0, Integral = 0, time_one = 0;

  if( reset == true )
  {
    Previous_Error = 0;
    Integral = 0;
    time_one = 0;
  }

  double time_two = millis();
  double dt = ( time_two - time_one )/1000;
  
  double Error = SetPt-Actual;
  
  double P = kP*Error;
  double I = kI*(Integral + Error*dt);
  double D = kD*((Error - Previous_Error)/dt);
  
  double heat = (P+I+D);
  heat = saturation(heat);
  
  analogWrite( pin_heater_PWR,heat );
  Previous_Error=Error;
  Integral += Error*dt;
  time_one = time_two;
  

  nextPrint = checkPrint( nextPrint, Actual ); 
  
  return nextPrint;

}

double saturation(double heat)
{
  if ( heat < 0 ) return 0;
  else if ( heat > 255 ) return 255;
  else return heat;
}

void heaterOff()
{
  analogWrite( pin_heater_PWR, 0 );
  setLight( 0 ); // green

}

void fanOff()
{
  analogWrite( pin_fan_PWR, 0 );
  setLight( 0 ); // green
}

double Thermistor( int RawADC0, int RawADC1 )			// Return temperature reading from sensor
{
  double A_Therm0 = 0.001663981162535452685;
  double B_Therm0 = 0.000143367011151991180;
  double C_Therm0 = 0.000000464117311968448;
  double A_Therm1 = 0.001557089272579081484;
  double B_Therm1 = 0.000164086901955137694;
  double C_Therm1 = 0.000000365732084202379;
  double temp = SingleThermistor(RawADC0, A_Therm0, B_Therm0, C_Therm0)/4 + 
      SingleThermistor(RawADC1, A_Therm1, B_Therm1, C_Therm1)/4;
  return temp;

}
double SingleThermistor(int RawADC, double A, double B, double C){
  double temp = log( 2000.0 * ( 1024.0 / RawADC - 1 ));
  temp = 1 / ( A + B * temp + ( C * temp * temp * temp ) ); // Kelvin
  temp = temp - 273.15;
  return temp; 
}

double checkPrint( double nextPrint, double ActualTemp )
{
  double clock = millis();
  if ( clock >= nextPrint )
  {
    nextPrint = clock + PRINT_DELAY;
    int inSeconds = clock / 1000;
    Serial.print( inSeconds );
    Serial.print(F( "s"));
    Serial.print(F("\t" ));
    Serial.print(ActualTemp);
    Serial.println(F(" C"));
  }

  return nextPrint;
}

/* color
   0 = green
   1 = red
   2 = blue
   3 = yellow
 */
double setLight( int color ) {

  int redVal, blueVal;
  if ( color < 1 ) redVal = blueVal = 0;
  else if ( color == 1 ) {
    redVal = 255;
    blueVal = 0;
  } else if ( color == 2 ) {
    redVal = 0;
    blueVal = 255;
  } else if ( color == 3 ) {
    redVal = 255;
    blueVal = 0;
  }

  analogWrite( led_blue, blueVal );
  analogWrite( led_red, redVal );
}
