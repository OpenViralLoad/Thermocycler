/*
 * ARDUINO UNO THERMAL CYCLER WITH LCD DISPLAY
 * For use with the Polymerase Chain Reaction (PCR) process
 *
 * A Global Ties - Engineering World Health Team Project
 * 
 */

#include <avr/sleep.h>
#include <math.h>
#include "LCDScreen.h"
#include "Constants.h"
#include "pid.h"
#include "Adafruit_RGBLCDShield.h"
#include <Wire.h>

LCDScreen UI = LCDScreen(); 	// LCD display

int current_cycle;		// No. of cycles passed
double current_temp;
double current_time;
double PCR_time_start;    // Start time of process
double last_print_time = 0;

void t_update() {
  current_temp = getTemp();
  current_time = millis();
}

void setup()
{
  Serial.begin(9600);
  Serial.println(F("Setting up ... "));

  /* Heater and Fan are outputs, Thermistor is input */
  pinMode(pin_heater, OUTPUT);
  pinMode(pin_fan,    OUTPUT);
  pinMode(pin_therm,  INPUT);
  digitalWrite(pin_heater, LOW);
  digitalWrite(pin_fan,    LOW);
  if (DEBUG) component_check();

  all_off();  

  /* Collect temperature preferences from UI */
  UI.init();				// Boot up LCD screen
  UI.setup();  			// Allow user to select preferences
  TEMP_FLOOR = UI.retrieveTF();
  TEMP_CEIL = UI.retrieveTC();
  TEMP_CRIT = TEMP_CEIL + 20;
  CYCLES = UI.retrieveMC();
  DENAT = UI.retrieveDN();

  t_update();
  PCR_time_start = millis();		// Start the clock
  current_cycle = 0;    
  serial_setup_results();  
}

void loop()
{
  // State 0: Denaturize sample by heating to TEMP_CEIL and Maintaining for 10 min
  if (DENAT) Denaturization(TEMP_CEIL);

  if ( current_cycle++ < CYCLES )	// More cycles to go
  {
    t_update();
    UI.displayInitialStatus( current_temp, current_cycle + 1 );
    serial_status();

    RampUp( TEMP_CEIL );
    Maintain( TEMP_CEIL, MAINT_CEIL );	
    RampDown( TEMP_FLOOR );		
    Maintain( TEMP_FLOOR, MAINT_FLOOR );
  }
  else // All cycles completed
  {
    Serial.print( current_cycle++ );
    Serial.println(F(" cycles completed."));

    t_update();

    // Get final running time in seconds
    double total_running_time = (current_time - PCR_time_start)/1000;
    Serial.print(F("PCR process completed in "));
    Serial.print( total_running_time );
    Serial.println(F(" seconds."));

    UI.finalMess();			// Print goodbye message to LCD
    sleep();				// State 5: Sleep, do nothing until reset
  }
}

/*
 * DENATURIZATION
 *
 */
void Denaturization( int target )
{
  Serial.println(F("Entering Denaturization Stage..."));
  t_update();
  double start = current_time;
  UI.dispDenatStatus(0, 0);
  
  RampUp( TEMP_CEIL ); //Ramp
  Maintain( TEMP_CEIL, MAINT_DENAT ); //Hold for denaturization stage
  
  t_update();
  double totalTime = (current_time - start)/1000;
  Serial.print(F("Denaturization finished in "));
  Serial.print( totalTime );
  Serial.println(F(" seconds.\n"));
}

/*
 * RAMP UP
 * Heat up the sample to target ceiling temperature
 *
 */
void RampUp( short target )
{
  Serial.println(F("Ramping up ... "));
  t_update();
  double startTime = current_time;
  double error_start_time = 0;

  heater_max();

  while (current_temp < target)
  {
    check_print();
    UI.updateArrow(current_temp);
    double prev_temp = current_temp;
    t_update();

    if (current_temp <= prev_temp)
    {
      if (error_start_time == -1) error_start_time = current_time;
      double error_dur = current_time - error_start_time; 
      if ( error_dur > ERROR_DUR_MAX ) {
        Serial.println(F("Error: Stall on RampUp()."));
        all_off();
        UI.printError( 0 );
        setLight( 0 );
        for(;;){}
      }
    }
    else if ( current_temp > TEMP_CRIT )
    {
      Serial.println(F("Error: Temperature surpassing critical maximum"));
      Serial.println(F("Please make sure heater is functioning correctly"));
      all_off();
      UI.printError( 1 );
      setLight( 'o' ); // yellow
      while ( true ) {
      }
    }
    else {
      error_start_time = -1;
    }
  }

  all_off();
  double totalTime = (current_time - startTime)/1000;
  Serial.print(F("RampUp finished in "));
  Serial.print( totalTime );
  Serial.println(F(" seconds.\n"));
}


/*
 * MAINTAIN
 * Keep the sample heated at target ceiling temperature for a specified duration
 *
 */
void Maintain(short target, double duration)
{
  Serial.println(F("Maintaining temperature ..."));
  t_update();
  double stopTime = current_time + duration;
  setLight( 'r' ); // red
  pid fb_ctrl = pid(); 
  fb_ctrl.setBiases(PID_P, PID_I, PID_D);
  while ( current_time < stopTime )
  {
    t_update();
    fb_ctrl.update(current_temp, target, current_time);
    double M = fb_ctrl.getM();
    if (M > PID_M_MAX) PID_M_MAX = M;
    if (M < PID_M_MIN) PID_M_MIN = M;
    double ratio = M/(PID_M_MAX-PID_M_MIN);
    double scaled = ratio*255;
    heater_val(scaled);
    Serial.print(F("Scaled Heating Value: "));
    Serial.print(scaled);
  }
  all_off();
  Serial.println(F("Maintain complete.\n"));
}

/*
 * RAMP DOWN
* Cool down the sample to target floor temperature
 *
 */
void RampDown( int target )
{
  Serial.println(F("Ramping down ... "));
  t_update();
  double startTime = current_time;
  double error_start_time = 0;

  fan_max();

  while (current_temp > target)
  {
    check_print();
    UI.updateArrow(current_temp);
    double prev_temp = current_temp;
    t_update();

    if (current_temp >= prev_temp)
    {
      if (error_start_time == -1) error_start_time = current_time;
      double error_dur = current_time - error_start_time; 
      if ( error_dur > ERROR_DUR_MAX ) {
        Serial.println(F("Error: Stall on RampDown()."));
        all_off();
        UI.printError( 0 );
        setLight( 0 );
        for(;;){}
      }
    }
    else {
      error_start_time = -1;
    }
  }

  all_off();
  double totalTime = (current_time - startTime)/1000;
  Serial.print(F("RampDown finished in "));
  Serial.print( totalTime );
  Serial.println(F(" seconds.\n"));
}

/*
 * Sleep until external reset
 *
 */
void sleep()
{
  Serial.println(F("Turning off heater and fan ... \n"));
  analogWrite( pin_heater, 0 );         // Heater is on but not running
  analogWrite( pin_fan, 0);

  Serial.println(F("Putting device to sleep. Please restart the device."));

  setLight( 0 );

  set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  cli();
  sei();
  sleep_cpu();

}

void all_off() {
  analogWrite( pin_fan, 0 ); 
  analogWrite( pin_heater, 0 );
  setLight('o');
}

void heater_max() {
  heater_val(255);
}

void heater_val(short val) {
  setLight( 'r' ); // red
  analogWrite(pin_heater, val);
}

void fan_max() {
  fan_val(255);
}

void fan_val(short val) {
  setLight( 'b' ); // blue
  analogWrite(pin_fan, val);
}

double getTemp() {
  return SH_Eq(analogRead(pin_therm));
}

double SH_Eq( int RawADC )			// Return temperature reading from sensor
{
  double A = 0.001472780436;         //Constants determined by values given from datasheet
  double B = 0.000237303927;
  double C = 0.0000001070432571;
  double temp = log( 2000.0 * ( 1024.0 / RawADC - 1 ));
  temp = 1 / ( A + B * temp + ( C * temp * temp * temp ) ); // Kelvin
  return temp - 273.15;
}

double check_print()
{
  if ((current_time - last_print_time) >= PRINT_DUR) {
    int seconds = current_time/1000;
    Serial.print( seconds );
    Serial.print(F("s\t"));
    Serial.print( current_temp );
    Serial.println(F(" C"));
    last_print_time = current_time;
  }
}

void setLight( char c ) {
  short redVal, greenVal, blueVal = 0;
  if (c == 'b') blueVal = 255;
  else if (c == 'r') redVal = 255;
  else blueVal = redVal = 0;

  analogWrite( led_blue, blueVal );
  analogWrite( led_red, redVal );
}

void component_check() {
    t_update();
    Serial.println(F( "Testing heater ... " ));
    RampUp( current_temp + 10 );
    Serial.println(F( "Testing fan ... " ));
    RampDown( current_temp - 5 );
    setLight( 0 );
}

void serial_status() {
  Serial.print(F( "Current Cycle: " ));
  Serial.print( current_cycle );
  Serial.print(F( "/" ));
  Serial.println( CYCLES );
  Serial.print(F( "( Cycling between " ));
  Serial.print( TEMP_FLOOR );
  Serial.print(F( " and " ));
  Serial.print( TEMP_CEIL );
  Serial.println(F( " )" ));
}

void serial_setup_results() {
  Serial.println(F( "Setup complete.\n" ));
  Serial.println(F(" Current Settings: "));
  Serial.print(F("Denaturization: " ));
  if (DENAT) Serial.println(F("TRUE"));
  else Serial.println(F("FALSE"));
  Serial.print(F("Ceiling Temperature: "));
  Serial.print(TEMP_CEIL);
  Serial.print(F(" C for "));
  Serial.print(MAINT_CEIL/1000);
  Serial.println(F(" seconds"));
  Serial.print(F("Floor Temperature: "));
  Serial.print(TEMP_FLOOR);
  Serial.print(F(" C for "));
  Serial.print(MAINT_FLOOR/1000);
  Serial.println(F(" seconds"));
}

