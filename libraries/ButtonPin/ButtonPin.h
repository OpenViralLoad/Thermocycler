/*


*/

#ifndef BUTTONPIN_H
#define BUTTONPIN_H
#include "Arduino.h"
#include "Adafruit_RGBLCDShield.h"
#include <stdlib.h>
#include <stdio.h>
#include <Wire.h>

class ButtonPin {


	private:
		int BPIN_SELECT, BPIN_UP, BPIN_DOWN;
		int BPIN_LEFT, BPIN_RIGHT;
		int pin;
		Adafruit_RGBLCDShield *lcd;

	public:
		ButtonPin( int pin_ );
		~ButtonPin();
		int readButtons();
		float voltage();

};
#endif
