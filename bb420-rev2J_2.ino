/*
 * BB-420 Rev 2J Interface - for 2D board
 * 01 May 2018
 * 
 * 
 * 
 */
#include <EEPROM.h>
#include <Wire.h>
#include "RTClib.h"
#include "mybutton.h"
#include <LCD.h>
#include <LiquidCrystal_I2C.h> // includes the LiquidCrystal Library
LiquidCrystal_I2C  lcd(0x3F,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack
//LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack
RTC_DS1307 RTC;


#define MODE_AUTO      0
#define MODE_MANUAL    1
#define MODE_SET_SPEED 2
#define MODE_SET_DELAY 3
#define MODE_SET_TIME  4
#define MODE_SET_DATE  5
#define MODE_SET_SCHED 6

// Stepper Pins
#define stepPin 8
#define dirPin  7
#define enablePin 9
#define mode0Pin 12
#define mode1Pin 11
#define mode2Pin 10

//Limit Pins
#define leftLimitPin   A2
#define rightLimitPin  A3

//Button Pins
#define buttonAPin 3
#define buttonBPin 5
#define buttonCPin 6
#define buttonDPin 4


#define UP  1
#define DOWN  0

bool enablePinAvailable = true;


String modeTxt[] = {"Auto","Manual","Speed","Delay","Time","Date","Sched."};

long frameCounter = 0;
float runTime = 0.0f;
volatile long pulseCount = 0;
float pulseRate = 0.0f;
boolean rateFlag = false;
boolean rateReset = true;
unsigned long rateStart = 0;
int autoSpeed = 300;
int manSpeed = 300;
int defaultAutoSpeed = 300;
int defaultManSpeed = 300;
unsigned long defaultAutoDelay = 10;
float memoryHash = 118.0f;
volatile int loopCount = 0;
int targetLoopCount = 0;
double stepsPermm = 80;
int mode0State = LOW;
int mode1State = LOW;
int mode2State = HIGH;
int mode = MODE_AUTO;
int railDirection = UP;
int frame = 0;
long railPos = 0;
boolean autoPaused = true;
boolean emergencyStop = true;
//
int xpos = 0;
int speedpps = 300;
int speedNew = 300;
boolean speedDirty = false;
unsigned long limitDelay = 30;
unsigned long delayNew = 30;
boolean delayDirty = false;
unsigned long delayStart = millis();
bool delayStarted = true;



// Setup buttons
Button butA(buttonAPin);
Button butB(buttonBPin);
Button butC(buttonCPin);
Button butD(buttonDPin);

void setup() {
  //setup Timer1
  cli();
  TCCR1A = 0b00000000;
  TCCR1B = 0b00001001;        // set prescalar to 1
  TIMSK1 |= 0b00000010;       // set for output compare interrupt
  setMotorSpeed(manSpeed); 
  sei();                      // enables interrupts. Use cli() to turn them off
  
  // Setup LCD Module
  lcd.begin(16,2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display
  createChars();
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.noCursor();

  // Setup Real Time Clock Module
  Wire.begin();
 
  RTC.begin();
  if (! RTC.isrunning()) {
    //Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  //RTC.adjust(DateTime(__DATE__, __TIME__));
  Serial.begin(9600);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  if(enablePinAvailable){
    pinMode(enablePin, OUTPUT);
  }
  pinMode(mode0Pin, OUTPUT);
  pinMode(mode1Pin, OUTPUT);
  pinMode(mode2Pin, OUTPUT);
  pinMode(leftLimitPin, INPUT_PULLUP);
  pinMode(rightLimitPin, INPUT_PULLUP);
  
  
  
  digitalWrite(mode0Pin, mode0State);
  digitalWrite(mode1Pin, mode1State);
  digitalWrite(mode2Pin, mode2State);
  digitalWrite(dirPin, railDirection);
  if(enablePinAvailable){
    digitalWrite(enablePin, 0);
  }

  //emergencyStop = false;

  checkMemory();
  delay(1000);
  Serial.print("Stored Speed: ");
  Serial.println(getMemory());
  Serial.print("Stored Delay: ");
  Serial.println(getMemoryDelay());

  speedpps = getMemory();
  limitDelay = getMemoryDelay();
  setMotorSpeed(speedpps);
  
}

void loop() {
  frameCounter++;
  if(enablePinAvailable){
    if(emergencyStop){
      digitalWrite(enablePin, 0);
    }
    else{
      digitalWrite(enablePin, 1);
    }
  }
  //Serial.print("Test");
  // *********************************************************************
  // Button A Code
  if(butA.updateButton()){
    Serial.println("ButA");
    delayStarted = false;
    if(mode == MODE_AUTO){
      emergencyStop = true;
    }
    mode ++;
    if(mode > MODE_SET_DATE){
      mode = 0; 
    }
    if(mode == MODE_SET_SPEED){
      speedNew = speedpps;
    }
    else{
      speedDirty = false;
    }
    if(mode == MODE_SET_DELAY){
      delayNew = limitDelay;
    }
    else{
      delayDirty = false;
    }
   
  }
  
  // *********************************************************************
  // Button B Code
  if(butB.updateButton()){
    Serial.println("ButB");
    if(mode == MODE_AUTO){
     railDirection = DOWN;
     digitalWrite(dirPin, railDirection);
     emergencyStop = false;
    }
    if(mode == MODE_MANUAL){
      xpos --;
    }
    if(mode == MODE_SET_SPEED){
      speedDirty = true;
      speedNew --;
    }
    if(mode == MODE_SET_DELAY){
      delayDirty = true;
      delayNew --;
    }
  }
  
  // *********************************************************************
  // Button C Code
  if(butC.updateButton()){
    Serial.println("ButC");
    if(mode == MODE_AUTO){
     railDirection = UP;
     digitalWrite(dirPin, railDirection);
     emergencyStop = false;
    }
    if(mode == MODE_MANUAL){
      xpos ++;
    }
    if(mode == MODE_SET_SPEED){
      speedDirty = true;
      speedNew ++;
    }
    if(mode == MODE_SET_DELAY){
      delayDirty = true;
      delayNew ++;
    }
  }
  
  //setMotorSpeed(speedpps);

  // Manual direction control
  if(mode == MODE_MANUAL){
    if(butB.state() == LOW){
      railDirection = DOWN;
      digitalWrite(dirPin, railDirection);
      emergencyStop = false;
    }
    else if(butC.state() == LOW){
      railDirection = UP;
      digitalWrite(dirPin, railDirection);
      emergencyStop = false;
    }
    else{
      emergencyStop = true;
    }
  }
   
  if(butD.updateButton()){
    if(mode == MODE_AUTO){
      emergencyStop = !emergencyStop;
    }
    if(mode == MODE_SET_SPEED && speedDirty){
      setMemory(speedNew, limitDelay);
      speedDirty = false;
      speedpps = speedNew;
      setMotorSpeed(speedpps);
    }
    if(mode == MODE_SET_DELAY && delayDirty){
      setMemory(speedpps, delayNew);
      delayDirty = false;
      limitDelay = delayNew;
    }
    Serial.println("ButD");
  }
  
  // Limit switches
  if(digitalRead(leftLimitPin) == LOW && railDirection == UP){
    railDirection = DOWN;
    digitalWrite(dirPin, railDirection);
    if(mode == MODE_AUTO){
      delayStarted = true;
      delayStart = millis();
    }
    //Serial.println("Left limit");
  }
  if(digitalRead(rightLimitPin) == LOW && railDirection == DOWN){
    railDirection = UP;
    digitalWrite(dirPin, railDirection);
    if(mode == MODE_AUTO){
      delayStarted = true;
      delayStart = millis();
    }
    //Serial.println("Right limit");
  }
  if(delayStarted){
    if( (delayStart+(limitDelay*1000)) < millis() ){
      emergencyStop = false;
      delayStarted = false;
    }
    else{
      emergencyStop = true;
    }
  }
  if(frameCounter%1000 == 0){
    // ******************************************************************************
    // LCD printing
    lcd.clear();
    lcd.setCursor(0,1);
    if(!delayStarted){
      rateFlag = true;
      //pulseRate = pulseCount / millis();
      lcd.print("Mode      <   > ");
    }
    else{
      lcd.print("Mode");
    }
    if(rateFlag){
      if(rateReset){
        rateReset = false;
        rateStart = millis();
      }
      runTime = (((float)millis() - (float)rateStart)/1000.0f);
      pulseRate = (float)pulseCount / runTime;
      
    }
    lcd.setCursor(0,0);
    lcd.print(modeTxt[mode]);
    if(mode == MODE_AUTO){
      lcd.print(" x: ");
      lcd.print(pulseCount);
      //lcd.print(",");
      //lcd.print(String(pulseRate,3));
      lcd.setCursor(5,1);
      if(!emergencyStop){
        lcd.print("Off");
      }
      else{
        lcd.print("On");
      }
    }
    if(mode == MODE_MANUAL){
      lcd.print(" x: ");
      lcd.print(pulseCount);
    }
    if(mode == MODE_SET_SPEED){
      lcd.print(": ");
      if(speedDirty){
        lcd.print(speedNew);
      }
      else{
        lcd.print(speedpps);
      }
      if(speedDirty){
        lcd.setCursor(5,1);
        lcd.print("Set");
      }
    }
    if(mode == MODE_SET_DELAY){
      lcd.print(": ");
      if(delayDirty){
        lcd.print(delayNew);
      }
      else{
        lcd.print(limitDelay);
      }
      if(delayDirty){
        lcd.setCursor(5,1);
        lcd.print("Set");
      }
    }
    if(mode == MODE_SET_TIME){
      lcd.print(": ");
      printTime();
    }
    if(mode == MODE_SET_DATE){
      lcd.print(": ");
      printDate();
    }
    if(delayStarted){
      lcd.setCursor(9,1);
      lcd.print("D: ");
      lcd.print(limitDelay-(millis()-delayStart)/1000);
    }
    String debugString = "pulseCount: ";
    debugString += pulseCount;
    
    debugString += ", pulseRate: ";
    debugString += String(pulseRate,3);
    
    debugString += ", runTime: ";
    debugString += String(runTime,3);
  
    debugString += ", frameCount: ";
    debugString += String(frameCounter);

    debugString += ", OCR1A: ";
    debugString += String(OCR1A);
    Serial.println(debugString);
  }
  
  //Serial.println(butA.state());
  //delay(10);
}
//
// ---------------------------- Timer --------------------------------------------------
ISR(TIMER1_COMPA_vect) {
   
  if(!emergencyStop){
    if(loopCount == 0){
      loopCount = targetLoopCount;
      digitalWrite(stepPin, HIGH);       // Driver only looks for rising edge
      digitalWrite(stepPin, LOW);        //  DigitalWrite executes in 16 us  
      if(railDirection == UP){
        pulseCount++;
      }
      else{
        pulseCount--;
      }
      
      //Generate Rising Edge
      //PORTL =  PORTL |= 0b00001000;   //Direct Port manipulation executes in 450 ns  => 16x faster!
      //PORTL =  PORTL &= 0b11110111;
      //Location = Location + 250 * DirFlag ;  //Updates Location (based on 4000 Pulses/mm)
    }
    else{
      loopCount-- ;
    }
  }
}

void setMotorSpeed(int newMotorSpeed){
  long timerCount = 16000000/newMotorSpeed - 1;
  if(timerCount < 65536){ 
    loopCount = 0;
    targetLoopCount = loopCount;
    OCR1A = timerCount;
    /*Serial.println("------");
    Serial.print("OCR1A: ");
    Serial.println(OCR1A);
    Serial.print("loopCount: ");
    Serial.println(loopCount);
    Serial.print("newMotorSpeed: ");
    Serial.println(newMotorSpeed);*/
  }
  else{
    loopCount = floor(timerCount / 65535);
    targetLoopCount = loopCount;
    OCR1A = round(timerCount / (loopCount+1));
    /*Serial.println("------");
    Serial.print("OCR1A: ");
    Serial.println(OCR1A);
    Serial.print("loopCount: ");
    Serial.println(loopCount);
    Serial.print("newMotorSpeed: ");
    Serial.println(newMotorSpeed);*/
  }
}

struct memoryObject {
  boolean isSet = true;
  float hash = memoryHash;
  int speed;
  unsigned long delay;
};

void setMemory(int speedToSet, unsigned long delayToSet){
  int eeAddress = 0; //Move to start
  memoryObject memoryVar;
  EEPROM.get(eeAddress, memoryVar);
  if((speedToSet != memoryVar.speed)||(delayToSet != memoryVar.delay)){
    Serial.print("memory set to: ");
    Serial.println(speedToSet);
    Serial.print("delay set to: ");
    Serial.println(delayToSet);
   //Data to store.
    memoryObject memoryVarToStore;
    memoryVarToStore.isSet = true;
    memoryVarToStore.hash = memoryHash;
    memoryVarToStore.speed = speedToSet;
    memoryVarToStore.delay = delayToSet;
    EEPROM.put(eeAddress, memoryVarToStore);
  }
  else{
    Serial.print("memory not set to: ");
    Serial.println(speedToSet);
    Serial.print("delay not set to: ");
    Serial.println(delayToSet);
  }
}

int getMemory(){
  int eeAddress = 0; //Move to start
  memoryObject memoryVar;
  EEPROM.get(eeAddress, memoryVar);
  return memoryVar.speed;
}

unsigned long getMemoryDelay(){
  int eeAddress = 0; //Move to start
  memoryObject memoryVar;
  EEPROM.get(eeAddress, memoryVar);
  return memoryVar.delay;
}

void checkMemory(){
  int eeAddress = 0;
  memoryObject memoryVar;
  EEPROM.get(eeAddress, memoryVar);
  if(memoryVar.hash != memoryHash){
    memoryObject memoryVarToStore;
    memoryVarToStore.isSet = true;
    memoryVarToStore.hash = memoryHash;
    memoryVarToStore.speed = defaultAutoSpeed;
    memoryVarToStore.delay = defaultAutoDelay;
    eeAddress = 0;
    EEPROM.put(eeAddress, memoryVarToStore);
  }
}

void printTime(){
  DateTime now = RTC.now();
  lcd.print(padNum(now.hour()));
  lcd.print(':');
  lcd.print(padNum(now.minute()));
  lcd.print(':');
  lcd.print(padNum(now.second()));
}

void printDate(){
  DateTime now = RTC.now();
  //lcd.print(now.year(), DEC);
  //lcd.print('/');
  lcd.print(now.month(), DEC);
  lcd.print('/');
  lcd.print(now.day(), DEC);
  lcd.print(' ');
  lcd.print(dayString(now.dayOfWeek()));
}

String dayString(int dayNumber)
{
  String dayArray[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  return dayArray[dayNumber];
}

String padNum(int num){
  String numString = String(num);
  if(num<10){
    numString = "0" + numString;
  }
  return numString;
}

// Utility to create some custom characters for the LCD
void createChars(){
  byte c_up[8] = {
    B00100,
    B01110,
    B10101,
    B00100,
    B00100,
    B00100,
    B00100,
  };
  byte c_down[8] = {
    B00100,
    B00100,
    B00100,
    B00100,
    B10101,
    B01110,
    B00100,
  };
  byte c_left[8] = {
    B00000,
    B00100,
    B01000,
    B11111,
    B01000,
    B00100,
    B00000,
  };
  byte c_right[8] = {
    B00000,
    B00100,
    B00010,
    B11111,
    B00010,
    B00100,
    B00000,
  };
  byte c_bulb_on[8] = {
    B01110,
    B10001,
    B10001,
    B10001,
    B01110,
    B01010,
    B01110,
  };
  byte c_bulb_off[8] = {
    B01110,
    B11111,
    B11111,
    B11111,
    B01110,
    B01010,
    B01110,
  };
  lcd.createChar(0, c_up);
  lcd.createChar(2, c_down);
  lcd.createChar(1, c_left);
  lcd.createChar(3, c_right);
  lcd.createChar(4, c_bulb_on);
  lcd.createChar(5, c_bulb_off);
}
