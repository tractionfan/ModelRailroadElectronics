/***************************************************************************************
 * HO Trolley 3-Car, 5 Detector -  Concurrent throttles - ProMini
 * Version 6 - as loaded into layout - F.Miller - Installed 6/24/2019
 * display code restored for 128x32 OLED
 * NOTE:  This version starts with Cars 41@1WB, 42@2EB, 43@4EB
 *************************************************************************************** 
 * SCHEDULE INCLUDES ACTIVITY FOR 3 CARS, #41, $42 AND #43
 * WITH PROVISION FOR UP TO TWO RUNNING CONCURRENTLY
 * FEATURES HOLD SWITCH WHICH STOPS AT BOTTOM OF THE SCHEDULE, 
 * AT STARTUP - HOLDS TILL DCC SENSED
 * CAR OPERATION INCLUDES SLOW START, SLOW STOP, DOOR SOUNDS AND PASS BUZZER
 * LED AND DISPLAY SHOW OPERATING CAR AND SWITCH ACTIVITY 
 *  - WHEN CAR IS RUNNING LED IS SOLID ON
 *  - SLOW BLINKING IF IN HOLD STATE (AT BEGINNING AFTER LN ACQUISITION, OR AT END 
 *    OF SCHEDULE).  'REPEATING' MESSAGE AT SCHEDULE END GIVES TIME TO SET HOLD. 
 *  - FAST BLINKING DURING 'REPEAT' MSG
 *  - FAST HEARTBEAT WHILE STOPPED, OPENING/CLOSING DOORS AT 5 SECOND STATION STOP
 *  - QUICK SINGLE (OR DOUBLE) BLINK OUT TO SHOW PASSED DETECTION SPOT
 *  COMPANY LOGO IS DISPLAYED AT START UP FOR VIEW ENJOYMENT, THEN 'ACQUIRE 4X'
 *  MESSAGE SHOWN FOR EACH THROTTLE.
 *  OPS STEP TIMING REDUCED TO 200 MS TO FACILITATE CATCHING LN ACTIVITY, OTHER
 *  TIMING (SPEED CHANGE, STATION TIME, ETC.) ADJUSTED TO COMPENSATE.
 *  
 **************************************************************************************/
//==================================================================================
//  128X32 1306 OLED DISPLAY - SDA - A4,  SCL - A5
//==================================================================================
const unsigned char TROLLEYLOGO[]PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00,
0x03, 0xE1, 0x88, 0x21, 0x09, 0x20, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00,
0x02, 0x02, 0x44, 0x41, 0x09, 0x20, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
0x02, 0x04, 0x22, 0x81, 0x09, 0x20, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00,
0x02, 0x04, 0x23, 0x81, 0x09, 0x20, 0x80, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
0x03, 0xC4, 0x21, 0x01, 0xF9, 0x20, 0x80, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00,
0x02, 0x04, 0x23, 0x81, 0x09, 0x20, 0x80, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00,
0x02, 0x04, 0x22, 0x81, 0x09, 0x20, 0x80, 0x00, 0x01, 0x08, 0x20, 0x41, 0x08, 0x41, 0x00, 0x00,
0x02, 0x02, 0x44, 0x41, 0x09, 0x20, 0x80, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00,
0x02, 0x01, 0x88, 0x21, 0x09, 0x3E, 0xF8, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x93, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xA2, 0x80,
0x7C, 0xF8, 0x20, 0x71, 0xF2, 0x18, 0x42, 0x02, 0x93, 0x08, 0x08, 0x21, 0x08, 0x41, 0xBE, 0x80,
0x10, 0x84, 0x70, 0x90, 0x42, 0x24, 0x62, 0x02, 0x93, 0x08, 0x08, 0x21, 0x08, 0x41, 0xF2, 0x80,
0x10, 0x84, 0x51, 0x08, 0x42, 0x42, 0x42, 0x02, 0x93, 0x08, 0x08, 0x21, 0x08, 0x41, 0xF2, 0x80,
0x10, 0x84, 0x51, 0x00, 0x42, 0x42, 0x52, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB, 0x80,
0x10, 0xF8, 0x89, 0x00, 0x42, 0x42, 0x42, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xB3, 0x80,
0x10, 0x90, 0xF9, 0x00, 0x42, 0x42, 0x4A, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAB, 0xC0,
0x10, 0x88, 0x89, 0x08, 0x42, 0x42, 0x46, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAB, 0x80,
0x10, 0x85, 0x08, 0x90, 0x42, 0x24, 0x46, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0,
0x10, 0x83, 0x04, 0x70, 0x42, 0x18, 0x42, 0x07, 0xFE, 0x12, 0x3C, 0x84, 0x79, 0x20, 0xFF, 0xC0,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0x0C, 0x27, 0xFF, 0xC8, 0xC0, 0x44, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x00, 0x3C, 0x84, 0x78, 0x00, 0x7C, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x30, 0x00, 0x00, 0x00,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(128, 32, &Wire, -1); //Width,Height,No Reset Pin
String moveActivity = " 4x  y";
String swActivity =   "  x  y";
//==================================================================================
//                      OPERATIONAL PARAMETERS
//==================================================================================
unsigned long previousMillis = 0;     // store last time clock ticked
unsigned long detPreviousMillis = 0;  // store last detection time
unsigned long lagTime = 200;       // normal OPS clock time 
int actionStep = 0;               //step through normal OPS
int ccactionStep = 0;             //step through concurrent OPS
int prepStep = 0;                 //step through Prep activities
int schedStep = 0;                //step through schedule
int SWAddress;                    //set at either LT100 or LT101 switches
#define headliteFct 0             //turn on car headlite
#define buzzFct 4                 //sound buzzer
#define insideliteFct 5           //turn on car inside lites
#define doorFct 3                 //open door sounds
#define onoffsw 2                 //used for hold prep work on (next) car
#define opLED 3                  //vareous operations indications
#define maxSpeed 45               //best maximum speed setting 
int onFctValue[9] = {16,1,2,4,8,1,2,4,8}; //these are the values returned when ON (0 for OFF)
int SensorState[8] = {0,0,0,0,0,0,0,0};   // initially all Sensors off
int Sensor[4];                    //index into SensorState (by throttle)
boolean PrepMode;                 //true=do prep, false=do regular ops
boolean actionMode;               //true=do actions (within schedule)
boolean ccactionMode;             //true=do concurrent throttle actions (within schedule)
boolean schedMode;                //true=do schedule
boolean activeT[] = {false,false,false,false}; //note if throttle is still active
//==================================================================================
//          LOCONET LIBRARY STUFF   PIN 8 - RX, PIN 6 - TX
//=================================================================================
#define LN_RX_PIN 8               // PIN 8 - RX LOCONET INTERFACE
#define LN_TX_PIN 6               // PIN 6 - TX LOCONET INTERFACE
#define CLOSED 1
#define THROWN 0
#include <LocoNet.h>              // support for loconet messaging
#define RECALL_BUFFER_SIZE 8
lnMsg *LnPacket;
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
uint16_t LocoAddr[] = {41,42,43};             //index 0,1,2
uint8_t recallIndex;
uint32_t LastThrottleTimerTick;
boolean slotError = false;
LocoNetThrottleClass  Throttle[3] ;           //establish 3 throttles
boolean newAddr[4]= {true,true,true,true};    //tested to see if need to set functions
int t = 1;                                    //currently active throttle (1,2,3)
int ct = 1;                                   //currently concurrent active throttle
int spd1;                                     //speed counter for throttle t
int spd2;                                     //... and throttle ct
int stop1;                                    //stopping counter for throttle t
int stop2;                                    //... and throttle ct
int ctdelay;                                  //counter to delay ct throttle startup
//****************************************************************************
void setup() {
  Serial.begin(57600);            // Configure the serial port for 57600 baud
  Serial.println("Begin Animation V6");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //I2C address for OLED board
  display.clearDisplay();                     //remove default Adafruit logo
  display.drawBitmap(0,0,TROLLEYLOGO,128,32,WHITE);    // draw my own logo
  display.display();                    
  delay(5000);                                //enjoy for 5 sec
  display.clearDisplay();
  display.setTextColor(WHITE);                // Draw white text
  display.setTextSize(3);  
  LocoNet.init(LN_TX_PIN);
  delay(100);
  while (digitalRead(LN_RX_PIN)==LOW){        //hold till DCC is on
    noDCCScreen();
  }
  holdScreen();                   // Then continue to setup throttles  
  pinMode (onoffsw,INPUT_PULLUP); // LOW=HOLD, HIGH=RUN
  pinMode (opLED,OUTPUT);
  digitalWrite(opLED,HIGH);       // LED ON to show we are active
  schedStep = 0;                  // start schedule at top
  PrepMode = true;                // initially force into PrepMode steps
  schedMode = false;
  actionMode = false;
  ccactionMode = false;
  t = 1;                          // start out with (1=41) as 1st throttle in PrepMode
}

void loop()  {  
  LnPacket = LocoNet.receive() ;          // Check for any received LocoNet packets
  if( LnPacket ) {
    if( !LocoNet.processSwitchSensorMessage(LnPacket) )
      Throttle[t-1].processMessage(LnPacket) ; 
    }
  if(isTime(&LastThrottleTimerTick, 100)) {
    Throttle[t-1].process100msActions() ; 
  }    
  
  if ((millis() > previousMillis + lagTime) ){        // OK to do another step...
    previousMillis = millis();
    
    if (PrepMode) {
      prepStep++;
      DoPrep();
    }
    if (actionMode) {
      actionStep++;
      doAction();
    }
    if (ccactionMode) {
      ccactionStep++;
      doccAction();
    }
    if (schedMode){
      schedStep++;
      doSchedule();
    }  
  }
  LnPacket = LocoNet.receive() ;          // Check for any received LocoNet packets
  if( LnPacket ) {
    if( !LocoNet.processSwitchSensorMessage(LnPacket) )
      Throttle[t-1].processMessage(LnPacket) ; 
    }
  if(isTime(&LastThrottleTimerTick, 100)) {
    Throttle[t-1].process100msActions() ; 
  }    
}
//====================== Execute Schedule ================================
void doSchedule() {
  Serial.print("SchedStep ");
  Serial.println(schedStep);
  digitalWrite(opLED,HIGH);               //led back ON to show next action
  switch (schedStep) {        //Starting positions SB #41 @1, #42 at 2m #43 at 4
   case 1:
     setSwitch(100,CLOSED);
     break;   
   case 2:
      setActionParms(42,5,true);              //let throttle run conncurrent with next 
      break;    
    case 3:
      setSwitch(101,CLOSED);
      break;    
    case 4:
      setActionParms(41,3,false);
      break;
    case 5:
      confdone(42,41);
      break;    
    case 6:
      setSwitch(100,THROWN);
      break;       
    case 7:
      setActionParms(42,1,true);              //let throttle run conncurrent with next
      break;
    case 8:
      setActionParms(43,2,false); 
      break;    
    case 9:
      confdone(43,42);
      break;    
    case 10:
      setSwitch(101,THROWN);
      break;       
    case 11:
      setActionParms(41,4,true);              //let throttle run conncurrent with next 
      break;
    case 12:
      setSwitch(100,CLOSED);
      break;
    case 13:
      setActionParms(43,5,false); 
      break;
    case 14:
      confdone(41,43);
      break;    
    case 15:
      setSwitch(101,CLOSED);
      break;
    case 16:
      setActionParms(42,3,true);              //let throttle run conncurrent with next
      break;
    case 17:
      setSwitch(100,THROWN);
      break;
    case 18:
      setActionParms(41,2,false); 
      break;    
    case 19:
      confdone(42,41);
      break;    
    case 20:
      setSwitch(101,THROWN);
      break;
    case 21:
      setActionParms(42,4,true);              //let throttle run conncurrent with next
      break;
    case 22:
      setActionParms(43,1,false); 
      break;
    case 23:
      confdone(43,42);
      break;    
    case 24:
      setSwitch(101,CLOSED);
      break;
    case 25:
      setSwitch(100,CLOSED);
      break;    
    case 26:
      setActionParms(41,5,true);              //let throttle run conncurrent with next
      break;
    case 27:
      setActionParms(43,3,false); 
      break;        
    case 28:
      confdone(43,41);
      break;    
    case 29:
      setActionParms(42,2,true);              //let throttle run conncurrent with next 
      break;
    case 30:
      setSwitch(100,THROWN);
      break;    
    case 31:
      setActionParms(41,1,false); 
      break;        
    case 32:
      confdone(42,41);
      break;    
    case 33:
      setSwitch(101,THROWN);
      break;        
    case 34:
      setActionParms(43,4,false);
      break;
    case 35:
      Serial.println("Repeat Schedule");
      while (digitalRead(onoffsw)==LOW){      //stop action if Hold Sw on     
        holdScreen() ;                        //display 'holding'
        digitalWrite(opLED,LOW);              //blink while waiting
        delay(500);
        digitalWrite(opLED,HIGH);             //blink while waiting
        delay(500);
      }
      schedStep = 0;                          //repeat schedule
      actionMode = false;
      ccactionMode = false;
      schedMode = true;
      break;   
  }
}
//===================== Establish Action Parameters =====================
void setActionParms(uint16_t newAddr,int newSense, boolean ccOps) {
  Sensor[newAddr-40] = newSense;
  if (ccOps) {                            //concurrent throttle
    ctdelay = 0;                          //restart ct delay
    ccactionStep = 0;
    ccactionMode = true;                  //initates Action steps for concurrent throttle
    ct = newAddr-40;
    LocoAddr[ct-1] = newAddr;
    schedMode = true;                     //and let schedule proceed in parallel
  }else{
    actionStep = 0;
    actionMode = true;                    //initates Action steps for regular throttle
    t = newAddr-40;
    LocoAddr[t-1] = newAddr;
     schedMode = false;                   //let action turn schedule on when finished
   }
}
//============ Execute Individual Schedule steps for standard throttle==============
void doAction() {
//  Serial.print("actionStep ");
//  Serial.println(actionStep);
  switch (actionStep) {
    case 1:
      activeT[t-1]=true;               //note this throttle is active
      digitalWrite(opLED,HIGH);        //led back ON to show new car motion
      moveScreen(t, Sensor[t]);
      Serial.print("Start ");
      Serial.print(LocoAddr[t-1]);
      Serial.print(" to ");
      Serial.println(Sensor[t]);
      SensorState[Sensor[t]] = 0;        //set state 0 (off) in prep for looking
      spd1 = 3;
      Throttle[t-1].setSpeed(spd1);     //start increasing speed      
      break;
    case 2:
      spd1+=3;
      if (spd1<maxSpeed){
        Throttle[t-1].setSpeed(spd1);
        actionStep--;      //let LN process but keep coming back to here until maxspeed     
      }else{
        spd1=maxSpeed;
        Throttle[t-1].setSpeed(spd1);
      }
      break;
    case 3:
//      Serial.print(t);
//      Serial.print(" looking for ");
//      Serial.println(Sensor[t]);
      if (SensorState[Sensor[t]]==16) {  //test if Sensor was turned ON
        Throttle[t-1].setFunction(buzzFct, onFctValue[buzzFct]);  //sound buzzer
        delay(200);
        Throttle[t-1].setFunction(buzzFct, 0);                    //buzzer off
        Serial.print("Sensor Det ");
        Serial.println(Sensor[t]);
        SensorState[Sensor[t]] = 0;        //set state 0 (off)
      }else{
        actionStep--;    //otherwise escape to let LN process but come back to keep looking
      }
      break;
    case 4:
      Serial.print("Stop Car ");
      Serial.print(t);
      Serial.print(" @ ");
      Serial.println(Sensor[t]);
      spd1= maxSpeed - 4;
      Throttle[t-1].setSpeed(spd1);     //start reducing speed      
      break;
    case 5:
      spd1-=3;
      if (spd1<0){
        spd1=0;                        //if went below 0, reset to 0
        Throttle[t-1].setSpeed(0);     //ensure at stop
      }else{
        Throttle[t-1].setSpeed(spd1);          
        actionStep--;    //otherwise escape to let LN process but come back to this step      
      }
      break;
    case 6:
//      stopScreen(t, Sensor[t]);
      Throttle[t-1].setFunction(doorFct, onFctValue[doorFct]);  //open doors
      stop1 = 0;                        //init for next step
      break;
    case 7:                              //station stop 5 sec ( 10x500ms )
      stop1++;
      if (stop1<25) {
        if (stop1%2==0){                 //its even 
          digitalWrite(opLED,HIGH);      //blink while cycling through counter
        }else{                           //its odd (first time) 
          digitalWrite(opLED,LOW);       //heart beat blink while waiting
        }
      actionStep--;   //otherwise escape to let LN process but come back to this step  
      }
      digitalWrite(opLED,HIGH);          //ensure leave with LED ON
      break;
    case 8:
      Throttle[t-1].setFunction(doorFct, 0);  //close doors
      break;
    case 9:
      Serial.print(t);
      Serial.println("-inactive");
      activeT[t-1]=false;               //note this throttle is NOT active
      PrepMode = false;
      actionMode = false;
      schedMode = true;
      break;
    }
}
//============ Execute Individual Schedule steps for concurent throttle==============
void doccAction() {
//  Serial.print("ccactionStep ");
//  Serial.println(ccactionStep);
  switch (ccactionStep) {
    case 1:
      ctdelay++;
      if (ctdelay<20){     //check delay counter
        ccactionStep--;    //otherwise escape to let LN process but come back to this step  
        
      }else{               //counter done, continue startup
        activeT[ct-1]=true;              //note this throttle is active
        digitalWrite(opLED,HIGH);        //led back ON to show new car motion
        moveScreen(ct, Sensor[ct]);
        Serial.print("Start ");
        Serial.print(LocoAddr[ct-1]);
        Serial.print(" to ");
        Serial.print(Sensor[ct]);
        Serial.println(" for concurrent ops");
        SensorState[Sensor[ct]] = 0;        //set state 0 (off) in prep for looking
        spd2 = 3;
        Throttle[ct-1].setSpeed(spd2);     //start increasing speed
      }      
      break;
    case 2:
      spd2+=3;
      if (spd2<maxSpeed){
        Throttle[ct-1].setSpeed(spd2);
        ccactionStep--;      //let LN process but keep coming back to here until maxspeed     
      }else{
        spd2=maxSpeed;
        Throttle[ct-1].setSpeed(spd2);
      }
      break;
    case 3:
//      Serial.print(ct);
//      Serial.print(" looking for ");
//      Serial.println(Sensor[ct]);
      if (SensorState[Sensor[ct]]==16) {  //test if Sensor was turned ON
        Throttle[ct-1].setFunction(buzzFct, onFctValue[buzzFct]);  //sound buzzer
        delay(200);
        Throttle[ct-1].setFunction(buzzFct, 0);                    //buzzer off
        Serial.print("Sensor Det ");
        Serial.println(Sensor[ct]);
        SensorState[Sensor[ct]] = 0;        //set state 0 (off)
      }else{
        ccactionStep--;    //otherwise escape to let LN process but come back to keep looking
      }
      break;
    case 4:
      Serial.print("Stop Car ");
      Serial.print(ct);
      Serial.print(" @ ");
      Serial.println(Sensor[ct]);
      spd2= maxSpeed - 2;
      Throttle[ct-1].setSpeed(spd2);     //start reducing speed      
      break;
    case 5:
      spd2-=3;
      if (spd2<0){
        spd2=0;                        //if went below 0, reset to 0
        Throttle[ct-1].setSpeed(0);     //ensure at stop
      }else{
        Throttle[ct-1].setSpeed(spd2);          
        ccactionStep--;    //otherwise escape to let LN process but come back to this step      
      }
      break;
    case 6:
      Throttle[ct-1].setFunction(doorFct, onFctValue[doorFct]);  //open doors
      stop2 = 0;                        //init for next step
      break;
    case 7:                              //station stop 5 sec ( 10x500ms )
      stop2++;
      if (stop2<25) {
        if (stop2%2==0){                 //its even 
          digitalWrite(opLED,HIGH);      //blink while cycling through counter
        }else{                           //its odd (first time) 
          digitalWrite(opLED,LOW);       //heart beat blink while waiting
        }
      ccactionStep--;   //otherwise escape to let LN process but come back to this step  
      }
      digitalWrite(opLED,HIGH);          //ensure leave with LED ON
      break;
    case 8:
      Throttle[ct-1].setFunction(doorFct, 0);  //close doors
      break;
    case 9:
      Serial.print(ct);
      Serial.println("-inactive");
      activeT[ct-1]=false;               //note this throttle is NOT active
      PrepMode = false;
      ccactionMode = false;               //its done, don't come back
      schedMode = true;
      break;
    }
}
//===================== Check if both throttles caught up to their sensors ======
void confdone(int Addr1,int Addr2){
  if (!activeT[Addr1-41] && !activeT[Addr2-41]) {  //if both inactive
    actionMode = false;                 //OK to go to next schedule step
    ccactionMode = false;
    schedMode = true;
  }else{    
    schedStep--;                        //hold schedule and come back here
    if (activeT[Addr1-41]){actionMode = true;}      //ensure still active
    if (activeT[Addr1-42]){ccactionMode = true;} 
  }
}

//===================== Prepare for new car ==============================
void DoPrep() {
//  Serial.print("prepStep ");
//  Serial.println(prepStep);

  switch (prepStep) {                               //enter with t=1, but cycle up to 3
    case 11:                                        //10 loop steps for Loconet catch up
      acquireScreen(LocoAddr[t-1]);
      Throttle[t-1].init(0, 0, LocoAddr[t-1]);      //set Throttle IDX same as LN Addr
      Throttle[t-1].setSpeedSteps(TH_SP_ST_128);
      Serial.print("Address: ");
      Serial.println(LocoAddr[t-1]);
      Throttle[t-1].setAddress(LocoAddr[t-1]);      //if get a slot error, do a steal in next step
      break;
    case 12:
      if (slotError) Throttle[t-1].stealAddress(LocoAddr[t-1]);
      break;   
    case 13:                               //try again if timing off
      if (slotError) Throttle[t-1].stealAddress(LocoAddr[t-1]);
      slotError = false;
      break;    
    case 14:
      Serial.println("Got Throttle");
       break; 
    case 15:                               
      for (int i=0; i<9; i++){            //turn off any ON functions
         Throttle[t-1].setFunction(i, 0);
         delay(300);
      }
      break;    
    case 16:                               //headlites ON
      Throttle[t-1].setFunction(headliteFct, onFctValue[headliteFct]);
      break;
    case 17:                               //inside lites ON
      Throttle[t-1].setFunction(insideliteFct, onFctValue[insideliteFct]);
       break;
    case 18:
      Throttle[t-1].setSpeed(0);    
    case 19:
      t++;                                //next throttle
      if (t<4){
        prepStep = 0;                     //loop through Prep for next throttle
      }else{
        PrepMode = false;                 //done initializing all throttles
        schedMode = true;                 //start running schedule
        while (digitalRead(onoffsw)==LOW){     
          holdScreen() ;                        //display 'holding'
          digitalWrite(opLED,LOW);              //blink while waiting
          delay(500);
          digitalWrite(opLED,HIGH);             //blink while waiting
          delay(500);
        }
      }
      break;
  }
}
//======================= Set Switches ===============================
void setSwitch(int swit,int pos) {
   Serial.print(swit);
   Serial.print(" to ");
   swScreen(swit-100, pos);               //send to display
   if (pos == THROWN) {
     Serial.println("THROWN");        
   }else{
     Serial.println("CLOSED");             
   }
   sendOPC_SW_REQ(swit - 1, pos, 1);   //send requested CLOSED or THROWN cmd        
}

//------------------------ SEND LOCONET SWITCH COMMAND ------------------------
void sendOPC_SW_REQ(int address, byte dir, byte on) {
 lnMsg SendPacket ;
 int sw2 = 0x00;
 if (dir) sw2 |= B00100000;
 if (on) sw2 |= B00010000;
 sw2 |= (address >> 7) & 0x0F;
 SendPacket.data[ 0 ] = OPC_SW_REQ ;
 SendPacket.data[ 1 ] = address & 0x7F ;
 SendPacket.data[ 2 ] = sw2 ;
 LocoNet.send( &SendPacket );
}
//-------------------------- LocoNet Throttle acquisition ---------------------------
void notifyThrottleAddress( uint8_t UserData, TH_STATE State, uint16_t Address, uint8_t Slot ) {
  if(State == TH_ST_IN_USE)
  {
    ss.lastState = State;
    ss.lastLocoAddr = Address;
    ss.lastSlot = Slot;
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
//-------------------- Loconet Error Acquiring address ------------------------------
void notifyThrottleError( uint8_t UserData, TH_ERROR Error ){
  if (Error == 1) {
    Serial.println("Must Steal");
    slotError = true;
  }
}
//---------------------- Capture Sensor changes ---------------------------------------
void notifySensor( uint16_t Address, uint8_t State ) {
  if (State == 16) {
        SensorState[Address] = State; //set state 16(on)
        Serial.print(Address);
        Serial.println("-ON ");
        detPreviousMillis = millis();   //reset detection timer
        digitalWrite(opLED,LOW);        //blink op LED
        delay(100);                     //brief blink
        digitalWrite(opLED,HIGH);       //restore applicable LED    
  }
}

/********************************************************************
 * Subroutines to format and display messages on Screen
 ********************************************************************/

// ---------------------------------SHOW 'MOVE TO' SCREEN -----------------------------
void moveScreen(int c, int t){
  display.clearDisplay();
  display.drawRect(0,0,128,32,WHITE);  
  moveActivity.setCharAt(2, (char)c+48);   //insert Car #
  moveActivity.setCharAt(5, (char)t+48);   //insert To Location
  display.setCursor(7,5);
  display.print(moveActivity);
  display.setTextSize(1);  
  display.setCursor(8,3);
  display.print("C");
  display.setCursor(8,12);
  display.print("A");  
  display.setCursor(8,21);
  display.print("R");  
  display.setCursor(66,8);
  display.print("MOVE");
  display.setCursor(72,18);    
  display.print("TO");
  dispStep();                             //post step in upper right of screen in small font
  display.display();
}

// ---------------------------------SHOW 'SWITCH SET' SCREEN -----------------------------
void swScreen(int w, int s) {   
  display.clearDisplay();
  display.drawRect(0,0,128,32,WHITE);  
  swActivity.setCharAt(2,(char)w+48);    //insert Switch #
  if (s == 0) { swActivity.setCharAt(5,'N');}    //insert To Location  
  if (s == 1) { swActivity.setCharAt(5,'R');}    //insert To Location  
  //swActivity.setCharAt(4, (char)7);              //small dot symbol
  display.setCursor(7,5);
  display.print(swActivity);
  display.setTextSize(1);  
  display.setCursor(6,8);
  display.print("S");
  display.setCursor(6,18);
  display.print("W");
  display.setCursor(16,10);
  display.setTextSize(2);
  display.print("10");
  display.setTextSize(1);
  display.setCursor(70,8);
  display.print("SET");
  display.setCursor(74,18);    
  display.print("TO");
  dispStep();
  display.display();      //send buffer to screen        
}
// ---------------------------------SHOW 'HOLDING...' SCREEN -----------------------------
void holdScreen(){
  display.clearDisplay();
  display.drawRect(0,0,128,32,WHITE);  
  display.setCursor(6,2);
  display.setTextSize(2);
  display.print("HOLDING..");
  display.setCursor(8,20);
  display.setTextSize(1);  
  display.print("SET SWITCH TO RUN");
  dispStep();
  display.display();      //send to buffer
}
// ---------------------------------SHOW 'NO DCC' SCREEN -----------------------------
void noDCCScreen(){
  display.clearDisplay();
  display.drawRect(0,0,128,32,WHITE);  
  display.setCursor(6,2);
  display.setTextSize(2);
  display.print("  NO DCC");
  display.setCursor(8,20);
  display.setTextSize(1);  
  display.print("   TURN ON POWER  ");
  display.display();      //send to buffer
}
// ---------------------- POST 'STEP' IN UPPER RIGHT OF SCREEN ----------------------------
void dispStep() {
  display.setTextSize(1);  
  if (schedStep > 9){
    display.setCursor(114,2);
  }else{
    display.setCursor(118,2);    
  }
  display.print(schedStep);
  display.setTextSize(3);  
}
// ------------------------ POST THROTTLE ACQUIRE ------------------------------
void acquireScreen(int t){
  display.clearDisplay();
  display.drawRect(0,0,128,32,WHITE);  
  display.setCursor(5,10);
  display.setTextSize(2);
  display.print("ACQUIRE:");
  display.print(t);
  display.display();      //send to buffer
}  
