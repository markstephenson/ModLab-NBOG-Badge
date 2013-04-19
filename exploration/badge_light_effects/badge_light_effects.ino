/**************************************************************************************
 *
 * FileName...: 
 * Description:  
 * Target........: Arduino IDE
 * Author.......:  
 * Version.....:  
 * Date.........:  
 * Project......:  
 * Contact.....:  
 * License.....:  
 * Keywords...: 
 * History.......: 
 *
 
 Libraries:
 * LED https://code.google.com/p/led-library/
 * SoftPWM
 * PinChangeInt
 
 use buttons tro trigger:
- specific colour transitions
- pulsating
- sonar effect (pinnnnng pinnnnng)

Colour Usage:
- Red for effor
- Green for transmission complete
 
***************************************************************************************/

#include <ColorLamp.h>
#include <PinChangeInt.h>
#include <SoftPWM.h>

#define LED 4 // non PWM

#define recieveIR 2
#define transmitIR 3

#define analogPower A5
#define lightSensor A4
#define soundSensor A3

#define buttonA 6
#define buttonB 5
#define buttonBoy 9
#define buttonGirl 7
#define buttonUp 8
#define buttonDown 11
#define buttonLeft 12
#define buttonRight 10


SOFTPWM_DEFINE_CHANNEL( 0, DDRD, PORTD, PORTD4 ); 
SOFTPWM_DEFINE_CHANNEL( 1, DDRC, PORTC, PORTC0 ); // A0
SOFTPWM_DEFINE_CHANNEL( 2, DDRC, PORTC, PORTC1 ); // A1
SOFTPWM_DEFINE_CHANNEL( 3, DDRC, PORTC, PORTC2 ); // A2
SOFTPWM_DEFINE_OBJECT_WITH_BRIGHTNESS_LEVELS( 4, 255 );

//SOFTPWM_DEFINE_OBJECT_WITH_BRIGHTNESS_LEVELS( 4, 255 );

/** Create an RGB lamp on pins 9, 10 and 11. Unless specified otherwise (by adding a 'false' parameter) this will automatically write values to these arduino pins. **/
ColorLamp * lamp  =  new ColorLamp(A2,A1,A0, false); 

// Or create an Array of LEDs
// ColorLamp * lamps[10]; 

boolean isCommunicating = false;
boolean isSeeking = false;
boolean isError = false;
int heartPump = 0;


void setup()
{

  lamp->intensityTo( 255, 1000 ); // Make sure the light is turned on so we can see what is happening
  lamp->saturationTo( 255, 1000 ); // make sure we can see Hue changes
  
  // setup buttons as input and pullup
    for (int pin = 5; pin <13; pin++) 
      {
      pinMode(pin, INPUT);     
      digitalWrite(pin, HIGH);
      }

    // enable analog power
    pinMode(analogPower, OUTPUT);     
    digitalWrite(analogPower, HIGH);
    
   // Attach functions to Buttons
   PCintPort::attachInterrupt (buttonA, &buttonAfunction, FALLING);
   PCintPort::attachInterrupt (buttonB, &buttonBfunction, FALLING);
   PCintPort::attachInterrupt (buttonBoy, &buttonBoyfunction, FALLING);
   PCintPort::attachInterrupt (buttonGirl, &buttonGirlfunction, FALLING);
   PCintPort::attachInterrupt (buttonUp, &buttonGirlfunction, FALLING);

   
   Serial.begin(9600);
   Serial.println("Hello Computer");
    
   SoftPWM.begin( 60 ); // 60Hz base freq
}

void loop()
{

  if (isCommunicating) {
    if( !lamp->isAnimating()) // Returns true if the LED is in an animation
    {
      if (heartPump == 2) {
        delay(1000);
        heartPump = 0;
      }
      else if (lamp->getIntensity() == 10) // Returns the current intensity of the LED
      {
        lamp->intensityTo( 255, 500 ); // Sets the desired LED value and the time (in millis) it should take to get there
      }
      else {
        lamp->intensityTo( 10, 500 );
        heartPump++;
      }
    }
  }
  
  
  if (isSeeking) {
    if( !lamp->isAnimating()) // Returns true if the LED is in an animation
    {
      if (lamp->getIntensity() == 0) // Returns the current intensity of the LED
      {
        lamp->intensityTo( 255, 300 ); // Sets the desired LED value and the time (in millis) it should take to get there
      }
      else {
        lamp->intensityTo( 0, 2500 );
      }
    }
  }
  
  /**
  You may also use these functions
  hsbTo( 255, 255, 255, 2000 ); // byte h, byte s, byte b, long duration
  rgbTo( 255, 255, 255, 2000 ); // byte r, byte g, byte b, long duration
  **/
  
  // Always call the update function; if autoWrite is on, the Arduino will write the current intensity to the set channel 
  lamp->update();
  
  SoftPWM.set( 1, lamp->getRed() );
  SoftPWM.set( 2, lamp->getGreen() );
  SoftPWM.set( 3, lamp->getBlue() );
}

void setColor (unsigned char red, unsigned char green, unsigned char blue) {
  SoftPWM.set( 1, red);
  SoftPWM.set( 2, green); 
  SoftPWM.set( 3, blue);
}


void buttonAfunction(void)
{
   Serial.println("Button A pushed"); 
   lamp->rgbTo( 255, 0, 0, 2000 );

} 
  
  
void buttonBfunction(void)
{
  // set colour to pink
  Serial.println("Button B pushed"); 
  lamp->rgbTo( 255, 255, 0, 2000 );
  
}

void buttonBoyfunction(void)
{
  Serial.println("Button Boy pushed - Simulate Commications State - Heart"); 
  lamp->rgbTo( 252, 117, 50, 1000 );
  //lamp->hsbTo( 300, 71, 100, 1000 );
  lamp->setAnimationType( QUADRATIC, true, true);
  isCommunicating = !isCommunicating;
  isSeeking = false;
}


void buttonGirlfunction(void)
{
  Serial.println("Button Girl pushed - Simulate Seeking State - Sonar"); 
  lamp->rgbTo( 50, 255, 0, 2000 );
  lamp->setAnimationType( QUADRATIC, false, true);
  isSeeking = !isSeeking;
  isCommunicating = false; 
}

void buttonDownfunction(void)
{
  Serial.println("Simulate Error - Flashing Red"); 
  lamp->rgbTo( 255, 255, 0, 2000 );
  lamp->setAnimationType( QUADRATIC, false, true);
  isSeeking = !isSeeking; 
}


