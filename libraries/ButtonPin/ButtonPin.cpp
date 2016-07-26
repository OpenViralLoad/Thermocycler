/*


*/

#include "ButtonPin.h"

ButtonPin::ButtonPin( int pin_, Adafruit_RGBLCDShield *lcd_  ) {

	lcd = lcd_;
	pin = pin_;

	BPIN_SELECT = 0; 
	BPIN_UP     = 0;
 	BPIN_DOWN   = 0;
	BPIN_LEFT   = 0;
	BPIN_RIGHT  = 0;
}

ButtonPin::~ButtonPin() {

}

float ButtonPin::voltage() {

	// Analog Read

	uint8_t result = lcd->_i2c->digitalRead(3);
	Serial.println( result );	
	return 0.;

}

int ButtonPin::readButtons() {

	float volts = voltage();

	// None pressed
	if( volts < .1 ) {
		BPIN_SELECT = 0;
		BPIN_UP     = 0;
 		BPIN_DOWN   = 0;
		BPIN_LEFT   = 0;
		BPIN_RIGHT  = 0;
		return 0;
	} 
	
	if( volts > 4 ) BPIN_SELECT = 1;
	else if( volts >  3 ) BPIN_UP = 1;
	else if( volts >  2 ) BPIN_DOWN = 1;
	else if( volts >  1 ) BPIN_LEFT = 1;
	else if( volts > .1 ) BPIN_RIGHT = 1;
		
	return 1;
}
