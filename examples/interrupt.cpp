// Install the LowPower library for optional sleeping support.
// See loop() function comments for details on usage.
//#include <LowPower.h>
#include "arduino.h"
#include <Wire.h>
#include <Adafruit_MCP23008.h>

//void cleanInterrupts();

Adafruit_MCP23008 mcp;

byte ledPin=5;

RTC_DATA_ATTR int bootCount = 0;

// Interrupts from the MCP will be handled by this PIN
byte intPin=39;

volatile boolean awakenByInterrupt = false;

// A pin at the MCP
// Buttons connect the pin to ground, and pins are pulled up.
byte mcpPin=0;

void setup(){

  Serial.begin(115200);

  while(!Serial);

  Serial.println("MCP23008 Interrupt Test");

  // We will setup a pin for flashing from the int routine
  pinMode(ledPin, OUTPUT);  // use the p13 LED as debugging

  pinMode(intPin,INPUT_PULLUP);

  mcp.begin();      // use default address 0
  
  // The INT will not be Floating 
  // INTs will be signaled with a LOW
  mcp.setupInterrupts(false,LOW);

  // configuration for a button 
  // interrupt will trigger when the pin is taken to ground by a pushbutton
  mcp.pinMode(mcpPin, INPUT);
  mcp.pullUp(mcpPin, HIGH);  // turn on a 100K pullup internally
  mcp.setupInterruptPin(mcpPin,FALLING); 
  
}

// The int handler will just signal that the int has happen
// we will do the work from the main loop.
void IRAM_ATTR intCallBack(){
  awakenByInterrupt=true;
}

void handleInterrupt(){
  
  awakenByInterrupt = false; // reset 
  
  Serial.println("interrupt");
  
  // Get more information from the MCP from the INT
  uint8_t pin = mcp.getLastInterruptPin();
  uint8_t val = mcp.getLastInterruptPinValue();
  
  // We will flash the led 1 or 2 times depending on the PIN that triggered the Interrupt
  // 3 and 4 flashes are supposed to be impossible conditions... just for debugging.
  uint8_t flashes=4; 
  if(pin == mcpPin) 
    flashes = 1;
  if(val != LOW) 
    flashes = 3;

  // simulate some output associated to this
  for(int i=0;i<flashes;i++){  
    delay(300);
    digitalWrite(ledPin,HIGH);
    delay(300);
    digitalWrite(ledPin,LOW);
  }

  // we have to wait for the interrupt condition to finish
  // otherwise we might go to sleep with an ongoing condition and never wake up again.
  // as, an action is required to clear the INT flag, and allow it to trigger again.
  // see datasheet for datails.
  while( ! (mcp.digitalRead(mcpPin) ));
  // and clean queued INT signal
  //cleanInterrupts();
}

/**
 * main routine: sleep the arduino, and wake up on Interrups.
 * the LowPower library, or similar is required for sleeping, but sleep is simulated here.
 * It is actually posible to get the MCP to draw only 1uA while in standby as the datasheet claims,
 * however there is no stadndby mode. Its all down to seting up each pin in a way that current does not flow.
 * and you can wait for interrupts while waiting.
 */
void loop(){
  
  // enable interrupts before going to sleep/wait
  // And we setup a callback for the arduino INT handler.

  attachInterrupt(digitalPinToInterrupt(intPin), intCallBack, FALLING);
  // Simulate a deep sleep
  while(!awakenByInterrupt);
  
  // disable interrupts while handling them.
  detachInterrupt(digitalPinToInterrupt(intPin));
  
  if(awakenByInterrupt) 
    handleInterrupt();
}



