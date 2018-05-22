#include <Arduino.h>
#include "mybutton.h"

Button::Button (int buttonPin) {
  pin = buttonPin;
  pinMode(pin, INPUT_PULLUP);
  s = HIGH;
}

bool Button::updateButton(){
  int currentState = digitalRead(pin);
  if(currentState != lastState){
    debounceTimer = millis();
  }
  if(millis()-debounceTimer > debounceDelay){
    if(currentState != s){
      s = currentState;
    }
  }
  lastState = currentState;

  // Repeat delay code
  if(s==LOW){
    if(reset){
      reset = false;
      pressedTime = millis();
      return true;
    }
    else if((millis()-pressedTime)>BUT_DELAY){
      pressedTime += BUT_REPEAT;
      return true;
    }
    
  }
  if(s==HIGH){
    reset = true;
  }
  return false;
}
