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

// ... and this interrupt vector
//byte arduinoInterrupt=1;

volatile boolean awakenByInterrupt = false;

// Two pins at the MCP (Ports A/B where some buttons have been setup.)
// Buttons connect the pin to ground, and pins are pulled up.
byte mcpPin=0;

void handleInterrupt();

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
esp_sleep_wakeup_cause_t print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
  return wakeup_reason;
}

void setup(){

  Serial.begin(115200);

  while(!Serial);

  Serial.println("MCP23008 Interrupt Test");

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  esp_sleep_wakeup_cause_t wakeup_reason = print_wakeup_reason();

  // We will setup a pin for flashing from the int routine
  pinMode(ledPin, OUTPUT);  // use the p13 LED as debugging

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0){
    handleInterrupt();
  }

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

  // enable interrupts before going to sleep/wait
  // And we setup a callback for the arduino INT handler.
  //attachInterrupt(arduinoInterrupt,intCallBack,FALLING);

  //attachInterrupt(digitalPinToInterrupt(intPin), intCallBack, FALLING);
  // Simulate a deep sleep
  //while(!awakenByInterrupt);
  // Or sleep the arduino, this lib is great, if you have it.
  //LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_39 , 0); //1 = High, 0 = Low
  esp_deep_sleep_start();
  // go to sleep

}

void handleInterrupt(){
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
    delay(500);
    digitalWrite(ledPin,HIGH);
    delay(500);
    digitalWrite(ledPin,LOW);
  }

  Serial.println("Interrupt handled");
}

/**
 * Nothing happens here!
 */
void loop(){
  
}



