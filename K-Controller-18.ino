/****************************************************************************
  TYPE K THROTTLE FOR TROLLEY OPERATIONS - F.MILLER 6/19/2018 PRO-MINI VERSION
*****************************************************************************  
    Original LocoNet Throttle code Copyright (C) 2002,2003,2004 Alex Shepherd
    Modified to use bare bones of NEW LocoNet Library.  No EEPROM CV use.  If an
    error is reported back when trying to acquire an address, then a "steal"
    address command is used in first prep step. So address is acquired one way
    or the other.
****************************************************************************
  This sketch uses the NEW LocoNet Library
  NOTE: PIN 8 - RX, 6 - TX TO LOCONET INTERFACE
****************************************************************************
    PrepWork:  Run from Loop but only once AFTER Controller levers are OK (See below)
    (1) Steal address (if required)
    (2) Reset all functions
    (3) Ensure Speed set at 0
    (4) Mute sounds in decoder
    (5) Turn on Inside and Headlites
    (6) sets holdclock so future loops will go to RUNTROLLEY and not PREPWORK   
****************************************************************************
  Startup (at power on) will be held until:
    (1) Direction Lever is in OFF
    (2) Throttle is in Notch 0 (OFF)
    (3) Brake is in Apply position
    BREAKER will flash until these conditions are met
****************************************************************************
    RUNTROLLEY:  Run from loop, continuously
    (1) Sound GONG when notch pressed
    (2) Incrementally pumps up Air Pressure (meter) when low (=0)
    (3) Incrementally drops Air Pressure
    (4) Increases speed applicable to Notch setting
    (5) Applies brake (sets speed reduction cycle time) or releases brake
        applicable to settings
    (6) Circuit Breaker is tripped if:
        (a) Notch setting is not sequencial
        (b) Brake is activated while throttle is in a run notch
        (c) throttle is advanced if Brake is ON
        (d) attempt door open when brakes are NOT on
        (e) Operator drops throttle knob - bringing car to emergency stop
    (7) Circuit Breaker is reset by PB request WHEN BRAKES AND THROTTLE OFF
    (8) Speed is decremented (by the brake reduction cycle)
    (9) Speed is decremented gradually if in Notch 0
    (10) Door sounds are played when toggled on and off    
********************************************************************
DISPLAY SCREEN LAYOUT ON ILM-216 DISPLAY PANEL:
POS 0: [ALARM DOOR BRAKE]
POS 16:[S:XX DIR P NCH:X]
*********************************************************************
***DIAGNOSTICS***  Used for testing only (GONG) initiates IDE Display
********************************************************************/

#define Version 18
#include <LocoNet.h>
#define RECALL_BUFFER_SIZE 8
#define headliteFct 0             //turn on car headlite
#define gongFct 2    //NOT USED   //double gong sounds
#define doorFct 3    //NOT USED   //open door sounds 
#define buzzerFct 4  //NOT USED   //passenger buzzer sound
#define insideliteFct 5           //turn on car inside lites
#define muteFct 6                 //used to mute car sounds
#define DFPlayer1Pin 7   //***    //DFPlayer#1 RX with compressor sounds
#define DFPlayer2Pin 4   //***    //DFPlayer#2 RX with gongs, etc.
#define gongPin 2                 //ring gong (in controller)
#define doorPin 3                 //open/close doors (sounds)
#define brakeOnPin 11             //apply brakes
#define brakeReleasePin 10        //release brakes
#define alarmLED 13               //show 'circuit breaker' has thrown
#define alarmReleasePin 12        //reset the 'circuit breaker'
#define meterPin 5                //operate the meter   
#define fwdPin 15                 //set direction FWD
#define revPin 16                 //set direction REV
#define addrPin 17                //select address 41 or 42
#define pumpSndTrk 1              //on player 1
#define gongSndTrk 1              //on player 2
#define notchSndTrk 2             //on player 2
#define hissSndTrk 3              //on player 2
#define breakerSndTrk 4           //on player 2
#define doorSndTrk 5              //on player 2
#define rxPin 255                 // RX back from display Not used, so set to invalid pin #
#define txPin 9                   // Connect LCD-216 SER input to this pin.
#define inverted 1                // In setup, 1=inverted, 0=noninverted
const char clearScreen[ ] = {12,14,4,0};      //display character positions
const char almPos[]={16,64,0,0};   //64+0   "ALARM"
const char dorPos[]={16,70,0};     //64+6   "DOOR"
const char brkPos[]={16,75,0};     //64+11  "BRAKE"
const char botLinePos[]={16,80,0}; //64+16
const char spdPos[]={16,82,0};     //64+18  "XX "
const char dirPos[]={16,85,0};     //64+21  "OFF","FWD", OR "REV"
const char airPos[]={16,89,0};     //64+25  "P"
const char nchPos[]={16,95,0};     //64+31  "X"
const long Xtime = 500;            // compressor air loss time
const long Ytime = 100;            // compressor air add time
unsigned long prevXtime = 0;
unsigned long prevYtime = 0;
const long Stime = 1000;          // speed degradation time
unsigned long prevStime = 0;
unsigned long prevBtime = 0;      // speed reduction re:braking
const long Btime = 500;
int alarmType = 0;                //1-3 notching too fast, 4 door while moving, 5 brakes while
                                  //  throttle not off, 6 Deadman throttle drop
int currentNotch = 0;
int currentSpeed = 0;
int speedChangeRate = 0;          //how much braking reduction
int airPressure = 0;              //0-255 shows 0-100 on meter
int apInc = 3;                    //initial pump up time incr
int curDir = 0;                   //0=OFF, 1=FWD, 2=REV
int prevDir = 0;                  //check for change
boolean brakesOn = false;
boolean EbrakesOn = false;
boolean alarmOn = false;
boolean doorOpen = false;         //keep track of current door state
boolean doorPrev = false;         //check for change

#define notch_ADC_PIN  A0         // A0 is the notch ADC input
#define NCH_3         815         //1.2K to +5, 4.8K to Gnd 
#define NCH_2         609         //2.4K to +5, 3.2K to Gnd
#define NCH_1         405         //3.6K to +5, 2.4K to Gnd
#define NCH_0         201         //4.8K to +5, 1.2K to Gnd
#define butHyst        10         //hysteresis (variance + or - ) from above
long butlagTime = 500;
unsigned int notchVoltage;
byte notch = 0;

//********************* Capacitive Touch Detection Stuff *********************************
#include <CapacitiveSensor.h>
// Digital pin 18 sends electrical energy, Digital pin 19 senses senses a change
// 2.2M resistor between pins works well
// Touch plate connected to Digital pin D19
// D18 is A4, D19 is A5 special pin outs next to chip and adjacent to A2 & A3
CapacitiveSensor capSensor = CapacitiveSensor(18,19);
int threshold = 100;    // threshold: 100 - 300 (lower most sensitive, 200 good value
int samples = 10;       // sampling: 10 - 50 (higher most sensitive, 30 good value
int deadCnt = 0;        // to iterate capacitive check 5 times (debounce)

//********************* Sound player Stuff *********************************
#include <SoftwareSerial.h>
SoftwareSerial softSerial1(99, DFPlayer1Pin); //no Arduino RX, Pin 14 for Arduino TX to DFPlayer RX .
SoftwareSerial softSerial2(99, DFPlayer2Pin); //no Arduino RX, Pin 14 for Arduino TX to DFPlayer RX .
SoftwareSerial softwareSerialD = SoftwareSerial(rxPin, txPin, inverted);
 
uint8_t send_buf[10]= { 0x7E, 0xFF, 0x06, 0x00, 0x00, 00, 00, 0x00, 0x00, 0xEF};  // Template Command.
// DFPlayer command format: (0-9)Byte 3 (cmd) 5 (parm), 6-7 (Checksum) need to be set

int playVol = 10;
boolean soundOn = false;
long lastGongTime = 0;            // to debounce Gong notch
long notchDebounceTime= 100;      // notch delay time
int whichSerial = 1;              //1=player#1(compressor), 2=player#2(misc)

//****************** Required LocoNet Library Stuff **************************

typedef struct {
  uint8_t         lastSlot;
  uint16_t        lastLocoAddr ;
  uint16_t        lastThrottleIDX ;
  TH_STATE        lastState;
  TH_SPEED_STEPS  lastSpeedSteps;
  int16_t         recallBuffer[RECALL_BUFFER_SIZE];
}
STORED_STATE;
STORED_STATE ss;
uint16_t LocoAddr ;
uint8_t recallIndex;
uint8_t maxAction = 22;           //max number of action steps + 1
uint8_t stepCounter;
boolean holdclock = false;
long lastTime = 0;                //save time of last click
long lagTime = 1000;              //clock ticks every second
uint32_t LastThrottleTimerTick;
boolean slotError = false;
LocoNetThrottleClass  Throttle ;
lnMsg                 *RxPacket ;

//------------------------------------------------------------------
void notifyThrottleAddress( uint8_t UserData, TH_STATE State, uint16_t Address, uint8_t Slot ) {
  if(State == TH_ST_IN_USE)
  {
    ss.lastState = State;
    ss.lastLocoAddr = Address;
    ss.lastSlot = Slot;
  }
}
//-------------------------------------------------------------------
void setup() {
  LocoNet.init(6);                //6 for TX, 8 for RX LOCONET
  Serial.begin(57600);
  Serial.print("Begin Ver:");
  Serial.println(Version);
  softwareSerialD.begin(9600);    // Set the data rate TO ILM-216 DISPLAY PANEL
  softwareSerialD.print(clearScreen); //clear sceen, backlight ON, 
  softwareSerialD.print("Software Ver:");
  softwareSerialD.print(Version);
  pinMode( notch_ADC_PIN, INPUT );         //ensure A0 is an input
  digitalWrite( notch_ADC_PIN, LOW );      //ensure pullup is off on A0
  pinMode(gongPin,INPUT_PULLUP);
  pinMode(brakeOnPin,INPUT_PULLUP);
  pinMode(brakeReleasePin,INPUT_PULLUP);
  pinMode(alarmReleasePin,INPUT_PULLUP);  
  pinMode(addrPin,INPUT_PULLUP);  
  pinMode(fwdPin,INPUT_PULLUP);  
  pinMode(revPin,INPUT_PULLUP);  
  pinMode(doorPin,INPUT_PULLUP);
  pinMode(meterPin,OUTPUT);
  pinMode(alarmLED,OUTPUT);
  digitalWrite(txPin, LOW);       // Stop bit state for inverted serial to ILM-216 
  pinMode(txPin, OUTPUT);
  softSerial1.begin(9600);
  delay (500);               
  softSerial2.begin(9600);
  delay (500);                    //** Required before 1st serial command to DFPlayer
  if (digitalRead(addrPin)==LOW){
    LocoAddr = 41;
  }else {
    LocoAddr = 42;
  }
  setvol_cmd(1,playVol);          //set DFPlayer 1 volume    
  setvol_cmd(2,playVol);          //set DFPlayer 2 volume
  stepCounter = 0;                //PrepWork step counter
  Throttle.init(0, 0, LocoAddr);  //set Throttle IDX same as LN Addr
  Throttle.setSpeedSteps(TH_SP_ST_128);
  softwareSerialD.print(botLinePos);
  softwareSerialD.print("Address: ");
  softwareSerialD.print(LocoAddr);
  Serial.print("Address: ");
  Serial.println(LocoAddr);
  Throttle.setAddress(LocoAddr);  //if this gens a slot error, will do a steal in step 1
  delay(1000);
  softwareSerialD.print(clearScreen); //clear sceen, backlight ON, 
  softwareSerialD.print("DIR & THROTTLE  OFF & BRAKE-LAP");
StartCk:
  digitalWrite(alarmLED,LOW);
  ckDir();
  softwareSerialD.print(dirPos);
  softwareSerialD.print(" BR");
  Readnotchs();
  if (curDir!=0 || notch!=0 || digitalRead(brakeOnPin)==LOW || digitalRead(brakeReleasePin)==LOW) {
    digitalWrite(alarmLED,HIGH);  //flash Breaker LED
    delay(500);
    playtrk_cmd(2,breakerSndTrk);       //breaker sound
    delay(100);
    goto StartCk;
  }
  if(digitalRead(brakeOnPin)==LOW) {
    brakesOn = true;
    }
  else if(digitalRead(brakeReleasePin)==LOW) {
    brakesOn = false;
  }
  else if(digitalRead(brakeOnPin)==HIGH && digitalRead(brakeReleasePin)==HIGH) {
    brakesOn = true;
  }
  currentNotch=0;
  Serial.println("Start OK");
  softwareSerialD.print(clearScreen);
  softwareSerialD.print("Start Succesful");
  long sensorValue = capSensor.capacitiveSensor(samples);  //prime capSensor
}

//------------------- Head of Loop & Required LocoNet work --------------------------

void loop()  {  
  static long lasttime;
  if (millis() > lasttime + butlagTime) {  // OK to look at another notch
      notch = Readnotchs();
      lasttime = millis();                      
  }
   RxPacket = LocoNet.receive() ;  // Check for any received LocoNet packets
  if( RxPacket ) {
    if( !LocoNet.processSwitchSensorMessage(RxPacket) )
      Throttle.processMessage(RxPacket) ; 
    }
  if ((millis() > lastTime + lagTime) && !holdclock){  // OK to do another action
    stepCounter++;
    lastTime = millis();
    PrepWork();  
    }
  if(holdclock) {           //means finished with the PrepWork 
    RunTrolley();           //so now free to do normal ops (all in RunTrolley)
  }  
  if(isTime(&LastThrottleTimerTick, 100)) {
    Throttle.process100msActions() ; 
  }
}
//------------------- Initial LocoNet Address work --------------------------

void PrepWork() {
  switch (stepCounter) {
    case 1: 
      if (slotError) Throttle.stealAddress(LocoAddr);
      break;
    case 2:         //try again if timing off because of SoftwareSerial
      if (slotError) Throttle.stealAddress(LocoAddr);
      slotError = false;
      break;    
    
    case 3:  //turn OFF any ON functions
      for (int i=0; i<9; i++){
         if (Throttle.getFunction(i))Throttle.setFunction(i,!Throttle.getFunction(i));
      }
      break;
    case 4:
      Throttle.setSpeed(0);
      break;
    case 5:
      softwareSerialD.print(botLinePos);
      softwareSerialD.print("Car:Mute ");
      Throttle.setFunction(muteFct, !Throttle.getFunction(muteFct));
      break;
    case 6:
      softwareSerialD.print("Lites");
      Throttle.setFunction(headliteFct, !Throttle.getFunction(headliteFct));
      Throttle.setFunction(insideliteFct, !Throttle.getFunction(insideliteFct));
      break;
    case 7:
      softwareSerialD.print(clearScreen);
      softwareSerialD.print(botLinePos);
      softwareSerialD.print("S:0  OFF   NCH:0");
      
      if(brakesOn){
        softwareSerialD.print(brkPos);
        softwareSerialD.print("BRAKE");       
      }
      holdclock = true;             //exit loop-prep and start loop-RunTrollry
      break;
    }
}
//--------------------Main Operations (called from Loop) ----------------------

void RunTrolley() {      
//:::::::::::::::::: Check for DeadMan Emergency stop :::::::::::::::::::

    if (currentNotch != 0 && !alarmOn){
      long sensorValue = capSensor.capacitiveSensor(samples);
      if(sensorValue < threshold) {   //if below, means operator let go
        deadCnt++;
        Serial.println(deadCnt);
      }else{
        deadCnt = 0;
      }  
      if (deadCnt > 3){
          deadCnt = 0;
          alarmType = 6;
          setAlarm();
// **DIAGNOSTICS**  Serial.println(sensorValue);
          playHiss();
          brakesOn = true;
          EbrakesOn = true;
          softwareSerialD.print(brkPos);
          softwareSerialD.print("E.BRK");       
          Serial.println("E.BRK");
          speedChangeRate = 10;                   //set for fast reduction         
      }
    }

    if (EbrakesOn && currentSpeed == 0) {         //recover from Emergency Braking
      EbrakesOn = false;
      brakesOn = false;
      softwareSerialD.print(brkPos);
      softwareSerialD.print("     ");
      playHiss();
      speedChangeRate = 0;
    }    

//:::::::::::::::::: Check for change in Direction :::::::::::::::::::
  ckDir();                              //get 'curDir'
  if (curDir != prevDir) {
    prevDir = curDir;
// **DIAGNOSTICS**    Serial.print("new dir: ");
//    if (curDir==0){Serial.println("OFF");}
//    if (curDir==1){Throttle.setDirection(0); Serial.println("FWD");}
//    if (curDir==2){Throttle.setDirection(1); Serial.println("REV");}   
    if (curDir==1){Throttle.setDirection(0);}
    if (curDir==2){Throttle.setDirection(1);}   
  }
//:::::::::::::::::: Reset Alarm but only if in Notch 0 :::::::::::::::: 

  if (digitalRead(alarmReleasePin)==LOW && alarmOn && currentNotch == 0) {
    alarmOn = false;
    digitalWrite(alarmLED, LOW);
    playAlarm();
    delay(500);
    softwareSerialD.print(almPos);
    softwareSerialD.print("     ");
  }

//:::::::::::::::::: Increase Brake Air Pressure when pump on ::::::::::::
  
  if (millis() - prevYtime >= Ytime && airPressure <= 255 && soundOn) {   //time to pump up airpressure (meter)
    prevYtime = millis();
    analogWrite(meterPin,airPressure);
    airPressure+=apInc;
  }
  if (airPressure >= 255) {
    soundOn = false;
    softwareSerialD.print(airPos);
    softwareSerialD.print(" ");
    airPressure = 255;
    apInc = 2;                      //all future pump incr slower than first
  }
  if (airPressure <= 80 && !soundOn){        //Pressure low - need to run pump
    playtrk_cmd(1,pumpSndTrk);      //play pump sound on DFPlayer 1
    soundOn = true;
    softwareSerialD.print(airPos);
    softwareSerialD.print("P");
    analogWrite(meterPin,airPressure);
    //airPressure+=5;
  }
    if (millis() - prevXtime >= Xtime && !soundOn) {   // if enough time elapsed reverse Meter position
      prevXtime = millis();
      airPressure --;
      analogWrite(meterPin, airPressure);
    }    
//::::::::::::::::: Reduce Speed per braking rate :::::::::::::::::

  if ((millis() - prevBtime >= Btime) && brakesOn && currentSpeed > 0) {
    currentSpeed -= speedChangeRate;        //reduce speed in accordance with braking rate    
    if (currentSpeed < 0){ currentSpeed = 0;}
    Throttle.setSpeed(currentSpeed);
    prevBtime = millis();
    disSpd();
// **DIAGNOSTICS**    Serial.print("EBrk Spd: ");
// **DIAGNOSTICS**    Serial.println(currentSpeed);
  }
//::::::::::::::::: Reduce Speed per coasting :::::::::::::::::

  if (millis() - prevStime >= Stime && !brakesOn && currentSpeed >= 1 && currentNotch == 0) {   //time to drop speed
    prevStime = millis();
    currentSpeed-=5;
    if (currentSpeed < 0){ currentSpeed = 0;}
    disSpd();
    Throttle.setSpeed(currentSpeed);
  }

//::::::::::::::::::: Service Throttle Notches :::::::::::::::::
   //---NOTCH 0
   if (notch==0 && currentNotch != 0) { 
      notchClick();
      softwareSerialD.print(nchPos);
      softwareSerialD.print("0");
      currentNotch = 0;         //don't change speed
      delay(500);               //debouce
  }
  //---NOTCH 1
  if (notch ==1 && currentNotch != 1 && curDir !=0){
      notchClick();
      softwareSerialD.print(nchPos);
      softwareSerialD.print("1");
      currentNotch = 1;
      if (!brakesOn && !alarmOn && airPressure>= 80){ //only if brakes off & no alarm & air OK
        if (currentSpeed > 25) {                      //Note 80 is = 30 on meter
          for (int i=currentSpeed; i>=25; i-=5){ //if speed > Notch 1 speed, reduce it down
             Throttle.setSpeed(i);
             delay(300);
          }          
          currentSpeed = 25;
          Throttle.setSpeed(currentSpeed);
          disSpd();
        }else{
          for (int i=currentSpeed; i<26; i+=5){   //if speed < Notch 1 speed, increase it up
            Throttle.setSpeed(i);
            //if (notch == 2){goto TooFast;}  //tried notch 2 too quickly
            delay(300);
          }
          currentSpeed = 25;
          disSpd();
        }
      }else{
        delay(500);     //wait for notch click
        TooFast:
        alarmType = 1;
        setAlarm();
      }
   }
  //---NOTCH 2
  if (notch==2 && currentNotch != 2 && !alarmOn && curDir !=0){
      notchClick();         //play notch sound
      softwareSerialD.print(nchPos);
      softwareSerialD.print("2");
      delay(500);     //wait for notch click to finish     
      if (currentNotch < 1 || brakesOn){
        alarmOn = true;     //can't jump from 0 to 2
        alarmType = 2;
        setAlarm();           //if did not progress thru Notch 1
      }
      currentNotch = 2;
      if (!brakesOn && !alarmOn) {
       if (currentSpeed > 50) {
          for (int i=currentSpeed; i>=50; i-=5){ //if speed > Notch 1 speed, reduce it down
             Throttle.setSpeed(i);
 // **DIAGNOSTICS**  Serial.println(i);
             delay(300);
          }          
          currentSpeed = 50;
          Throttle.setSpeed(currentSpeed);
          disSpd();
       }else{   
          for (int i=currentSpeed; i<51; i+=5){
          if (notch==3){goto TooFast;}  //tried notch 3 too quickly
          Throttle.setSpeed(i);
          delay(300);
          }
          currentSpeed = 50;
          disSpd();
       }
    }
  }
  //---NOTCH 3
  if (notch==3 && currentNotch != 3 && curDir !=0){
      notchClick();         //play notch sound
      softwareSerialD.print(nchPos);
      softwareSerialD.print("3");
      delay(500);     //wait for notch click to finish     
      if (currentNotch < 2 || brakesOn){
        alarmOn = true;     //can't jump from 0 or 1 to 3
        alarmType = 3;
        setAlarm();           //if did not progress thru Notch 1
      }
      currentNotch = 3;
      if (!brakesOn && !alarmOn) {
          for (int i=currentSpeed; i<76; i+=5){
          Throttle.setSpeed(i);
          delay(300);
          }
          currentSpeed = 75;
          disSpd();
      }
    }
//:::::::::::::::::::: Service Door Request ::::::::::::::::::::
  
  if(digitalRead(doorPin)==LOW && !doorOpen) {  //had change
      if (currentSpeed==0){
        playtrk_cmd(2,doorSndTrk);    //play door sound on DFPlayer 2
        softwareSerialD.print(dorPos);
        softwareSerialD.print("DOOR");
        doorOpen = !doorOpen;         //flip the state      
        delay(300);                   //debounce
      }else{
        if (!alarmOn){
          alarmType = 4;
          setAlarm();    //DON'T open dooor if car moving
        }
      }
  }
  if(digitalRead(doorPin)==HIGH && doorOpen) {  //had change
      playtrk_cmd(2,doorSndTrk);    //play door sound on DFPlayer 2
      softwareSerialD.print(dorPos);
      softwareSerialD.print("    ");
      doorOpen = !doorOpen;         //flip the state      
      delay(300);     //debounce
   }

//:::::::::::::::::::: Service Gong Request ::::::::::::::::::::

//  **DIAGNOSTICS**  if (digitalRead(gongPin)==LOW) {testDisplay();}
  if(digitalRead(gongPin)==LOW && (millis() - lastGongTime > notchDebounceTime)) {
     playtrk_cmd(2,gongSndTrk);    //play gong sound on DFPlayer 2
     delay(300);     //debounce
     lastGongTime = millis();
  }
//::::::::::::::::::: Service Brake Requests :::::::::::::::::::      

//  **DIAGNOSTICS**  if(digitalRead(brakeOnPin)==LOW) {Serial.println("B-ON");}
//  **DIAGNOSTICS**  if(digitalRead(brakeReleasePin)==LOW) {Serial.println("B-OFF");}
//  **DIAGNOSTICS**  if(digitalRead(brakeOnPin)==HIGH && digitalRead(brakeReleasePin)==HIGH) {Serial.println("B-LAP");}

  if(digitalRead(brakeOnPin)==LOW) {
    if (currentNotch > 0) {
      alarmType = 5;
      setAlarm();         //throttle ON - can't use brakes
      delay(300);         //debouce
    }else{
       airPressure-=10;
       if (!brakesOn) {playHiss();}          //do hiss only first time
       brakesOn = true;
       softwareSerialD.print(brkPos);
       softwareSerialD.print("BRAKE");       
       speedChangeRate++;                    //apply more brake pressure      
//  **DIAGNOSTICS**  Serial.println(speedChangeRate);
       delay(300);
    }
  }
    
  if(digitalRead(brakeReleasePin)==LOW && brakesOn && !EbrakesOn) {
      playHiss();
      brakesOn = false;
      speedChangeRate =0;
      softwareSerialD.print(brkPos);
      softwareSerialD.print("     ");             
  }

}

//==================== Complete & send DFPlayer command line ===================

void send_cmd() {
  uint16_t sum = 0;      //calc checksum from bytes 1-6
  for (int i=1; i<7; i++) {
    sum += send_buf[i];
  }
  send_buf[7] = -sum>>8;  //place hi into byte 7
  send_buf[8] = -sum;     //place lo into byte   
  if (whichSerial==1){
    for (int i=0; i<10; i++) {
      softSerial1.write(send_buf[i] );
    }  
  }else{
    for (int i=0; i<10; i++) {
      softSerial2.write(send_buf[i] );
    }
  }
}
// ======================== Individual DFPlayer Commands ========================

void setvol_cmd(int ser,int vol) {
  whichSerial = ser;                  //selected DFPlayer
  send_buf[3] = 0x06;                 // cmd
  send_buf[6] = vol;                  // specific vol
  send_cmd();
}
void playtrk_cmd(int ser, int trk) {
  whichSerial = ser;                  //selected DFPlayer
  send_buf[3] = 0x03;                 // cmd 
  send_buf[6] = trk;                  // specific track
  send_cmd();
}
void playAlarm(){
  playtrk_cmd(2,breakerSndTrk);       //breaker sound
  delay(100);
}
void notchClick(){
  playtrk_cmd(2,notchSndTrk);         //play notch click on DFPlayer 2
}
void playHiss(){
  playtrk_cmd(2,hissSndTrk);          //play hiss on DFPlayer 2
}  
//-----------------Signal Alarm -----------------------------
void setAlarm() {
  digitalWrite(alarmLED, HIGH);
  alarmOn = true;
  playAlarm();
  delay(500);
  softwareSerialD.print(almPos);
  softwareSerialD.print("ALARM");    

  Serial.print(alarmType);
  Serial.println(" ALARM");
  alarmType = 0;
}
//-------------------- Loconet Error Acquiring address ------------------------------

void notifyThrottleError( uint8_t UserData, TH_ERROR Error ){
  if (Error == 1) {
    Serial.println("Must Steal");
    slotError = true;
  }
}
//------------------- Loconet interval timing ---------------------------------------

boolean isTime(unsigned long *timeMark, unsigned long timeInterval) {
    unsigned long timeNow = millis();
    if ( timeNow - *timeMark >= timeInterval) {
        *timeMark = timeNow;
        return true;
    }    
    return false;
}
//------------------------------ check direction -----------------------------------

void ckDir() {
  softwareSerialD.print(dirPos);
  if (digitalRead(fwdPin)== LOW && digitalRead(revPin)==HIGH) {
    curDir = 1;
    softwareSerialD.print("FWD");
  }
  if (digitalRead(fwdPin)== HIGH && digitalRead(revPin)==LOW) {
    curDir = 2;
    softwareSerialD.print("REV");
  }
  
  if (digitalRead(fwdPin)== HIGH && digitalRead(revPin)==HIGH){
    softwareSerialD.print("OFF");
    curDir = 0;
  }
  delay(300);
}
//------------------------------ Display Speed & Direction ----------------------

void disSpd(){
    softwareSerialD.print(spdPos);
    softwareSerialD.print(currentSpeed);           //display speed
    softwareSerialD.print(dirPos);
    switch (curDir) {
      case 1: softwareSerialD.print("FWD");
        break;
      case 2: softwareSerialD.print("REV");
        break;
      case 0: softwareSerialD.print("OFF");
        break;      
    }
    softwareSerialD.print(" ");
}
//------------------------ Read (Analog) Throttle setting -------------------------------------
byte Readnotchs() {
   notchVoltage = analogRead( notch_ADC_PIN );  //read the notch ADC pin voltage
   if(   notchVoltage >= ( NCH_0- butHyst )
           && notchVoltage <= ( NCH_0 + butHyst ) )  {
      notch = 0;
   }
   else if(   notchVoltage >= ( NCH_1 - butHyst )
           && notchVoltage <= ( NCH_1 + butHyst ) ) {
      notch = 1;
   }
   else if(   notchVoltage >= ( NCH_2 - butHyst )
           && notchVoltage <= ( NCH_2 + butHyst ) ) {
      notch = 2;
   }
   else if(   notchVoltage >= ( NCH_3 - butHyst )
           && notchVoltage <= ( NCH_3 + butHyst ) ) {
      notch = 3;
   }
   return( notch );
}
//================== **DIAGNOSTICS** ========================================
void testDisplay() {
  Serial.println("-------STATUS-------------");
  Serial.print("Notch: ");
  Serial.println(currentNotch);
  Serial.print("Speed: ");
  Serial.println(currentSpeed);
  Serial.print("Air Pressure: ");
  Serial.println(airPressure);
  Serial.print("Brake: ");
  if(brakesOn){
    Serial.println("ON");
  }else{
    Serial.println("OFF");
  }      
  Serial.print("Braking Rate: ");
  Serial.println(speedChangeRate);
  Serial.print("Alarm: ");
  if(alarmOn){
    Serial.println("ON");
  }else{
    Serial.println("OFF");
  }
  Serial.print("Door ");
  if(doorOpen){
    Serial.println("Open");
  }else{
    Serial.println("Closed");
  }
  Serial.println("-------------------------");
  delay(500);             //debounce
}
