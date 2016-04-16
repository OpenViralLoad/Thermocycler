#include "Arduino.h"
#include "LCDScreen.h"
#include "Adafruit_RGBLCDShield.h"
#include "Adafruit_MCP23017.h"
#include <stdbool.h>
#include <Wire.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

short tempFloor;
short tempCeil; 
int maxCycles;
bool denat;

LCDScreen::LCDScreen () {
}

void LCDScreen::init() {

  maxCycles = 40;
  tempCeil = 95;
  tempFloor = 60;
  denat = 1;
}

void LCDScreen::setup() {

  lcd.begin( 16, 2 );
  lcd.home();
  lcd.print( "PCR V.2 - O.V.L.");
  lcd.setCursor( 0, 1 );
  lcd.print( "SELECT to cont." );
  waitForSelect();  
  delay( 500 );
  setUserInputs();
  delay( 500 );

}

void LCDScreen::displayInitialStatus( double temp, int current_cycle ) {

  lcd.clear();
  lcd.home();
  
  updateArrow( temp );
  
  lcd.setCursor( 0, 1 );
  lcd.print( "Cycle no. " );
  lcd.print( current_cycle );
  lcd.print( "/" );
  lcd.print( maxCycles );


}

void LCDScreen::updateArrow( double temp )
{
  lcd.home();
  lcd.print( tempFloor );
  
  if ( temp < tempFloor ) temp = tempFloor;
  
  int n; //number of spaces to create
  if ( tempFloor > 99 && tempCeil > 99 ) n = 9;
  else if ( tempFloor > 99 && tempCeil <= 99 ) n = 10;
  else if ( tempFloor <= 99 && tempCeil > 99 ) n = 10;
  else n = 11;  
  
  int newTemp = map( temp, tempFloor, tempCeil, 0, n );
  
  for ( int o = 0; o < newTemp; o++ )
  {
    lcd.print( "-" );
  }
  lcd.print( ">" );
  for ( int o = 0; o < ( n - newTemp); o++ )
  {
    lcd.print( " " );
  }

  lcd.print( tempCeil );
}

void LCDScreen::finalMess()
{
  lcd.clear();
  lcd.home();
  lcd.print( "PCR complete." );
  lcd.setCursor( 0, 1 );
  lcd.print( "RESET to cont." );

}

short LCDScreen::retrieveTF()
{
  return tempFloor;
}

short LCDScreen::retrieveTC()
{
  return tempCeil;
}

int LCDScreen::retrieveMC()
{
  return maxCycles;
}

bool LCDScreen::retrieveDN()
{
  return denat;
}

void LCDScreen::printError( int mess )
{
  lcd.clear();
  lcd.home();
  
  String message;
  if ( mess == 1 ) { message = "ERR: Overheating"; }
  else if ( mess < 1 ) { message = "ERR: Not ramping"; }
  else { message = "ERR: wrap failed"; }

  lcd.print( message );

  lcd.setCursor( 0, 1 );
  lcd.print( "Check hardware" );
}


void LCDScreen::waitForSelect()
{
  while ( true )
  {
    uint8_t buttons = lcd.readButtons();
    
    if ( buttons )
    {
      if (buttons & BUTTON_SELECT) break;
    }
  }
}


void LCDScreen::setUserInputs()
{
  lcd.clear();
  // Check for Denaturization
  denaturization();
  delay(500);
  lcd.clear();

  // Adjust High/Low Values
  int highlow = 0;
  
  selectHighLowPush();
  delay(500);
  // Adjust No. of Cycles
  selectCycles();
  delay(500);
  // All done with configuration
}

void LCDScreen::selectHighLowPush() {

  dispUserPrefHighLow();
  
  int highlow = 0;
  uint8_t buttons; 
 
  while ( true )
  {
    
    buttons = lcd.readButtons();
    
    if (buttons & BUTTON_UP) highlow = 0;  

    if (buttons & BUTTON_DOWN) highlow = 1;

    if (buttons & BUTTON_LEFT) 
    {
      if ( highlow == 0 ) 
      {
        if ( tempCeil > ( tempFloor + 1 ) ) tempCeil--;
      }
      else
      {
        if ( tempFloor > 49 ) tempFloor--;
      }
      
      dispUserPrefHighLow();
      delay( 100 );
    }
    
    if (buttons & BUTTON_RIGHT) 
    {
      if ( highlow == 0 )
      {
        if ( tempCeil < 110 ) tempCeil++;
      }
      else 
      {
        if ( tempFloor < ( tempCeil - 1 ) ) tempFloor++;
      }
      
      dispUserPrefHighLow();
      delay( 100 );
    }

    if (buttons & BUTTON_SELECT) break;
     
  }
}


void LCDScreen::selectCycles()
{
  dispUserPrefCycles();
  
  uint8_t buttons; 
  while ( true )
  {
    buttons = lcd.readButtons();
    if (buttons & BUTTON_LEFT) 
    {
      if ( maxCycles > 0 ) {
        maxCycles--;
        dispUserPrefCycles();
        delay(100);
      }
    }
    if (buttons & BUTTON_RIGHT) 
    {
      if ( maxCycles < 100 ) {
        maxCycles ++;
        dispUserPrefCycles();
        delay(100);
      }
    }
    if (buttons & BUTTON_SELECT) break;
  }
}

void LCDScreen::dispUserPrefHighLow()
{
  lcd.clear();
  lcd.home();
  lcd.print( "High: " );
  lcd.print( tempCeil );
  lcd.setCursor( 0, 1 );
  lcd.print( "Low:  " );
  lcd.print( tempFloor );
}


void LCDScreen::dispUserPrefCycles()
{
  lcd.clear();
  lcd.setCursor( 0, 0 );
  lcd.print( "# of cycles: " );
  lcd.setCursor( 0, 15 );
  lcd.print( maxCycles );
}

void LCDScreen::denaturization()
{
  lcd.clear();
  lcd.setCursor( 0, 0 );
  lcd.print("Denaturization? ");
  lcd.setCursor( 0, 1 );
  lcd.print("Yes     No");
  lcd.setCursor( 0, 1 );
  lcd.blink();
  uint8_t buttons;
  while(true){
    buttons = lcd.readButtons();
    if (buttons & BUTTON_LEFT){
     lcd.setCursor( 0, 1 );
     denat = 1;
    }
    if (buttons & BUTTON_RIGHT){
     lcd.setCursor( 8, 1 );
     denat = 0;
    }
    if (buttons & BUTTON_SELECT){
     lcd.noBlink();
     break;
    }
  }
}

void LCDScreen::dispDenatStatus(long int currentTime, long int stopTime)
{
  int minLeft;
  lcd.clear();
  lcd.setCursor( 0, 1 );
  if(currentTime == 0 && stopTime == 0){
    lcd.print("Ramping Up");
  }
  else{
  minLeft = (stopTime - currentTime) / 60000;
  lcd.print(minLeft);
  lcd.setCursor(2,1);
  lcd.print("min left");  
  }
    
}