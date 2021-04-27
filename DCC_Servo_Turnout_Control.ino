/*********************************************************************************** 
 * DCC Servo Turnout Control Decoder (using ATTINY85)  - F. Miller     7/12/2020
 * ATTINY85 PIN ADDRESSES: DCC D2 (PIN-7); SERVO D0 (PIN-5); RELAY D4 (PIN-3)
 ***********************************************************************************
 * EEPROM is used to store last position (restored on powerup or reset from DCC short)
 * Note this version does NOT have Adjustments to Address and Range; 
 * Attachment to Servo library set on/off before need for servo move
 * Frog Relay is set (ON for THROWN, OFF for CLOSED) at mid position of Servo Travel
 * The Address and range are 'hard coded in the sketch' C  T
 *                    This version throws clockwise:     \/
 * To change to coounter-clockwise throw, interchange 'SetThrown' and 'SetClosed'                   
 * routines and reverse the 'prevStateProper' settings in those routines
 */
//============================== ADJUST THESE FOR EACH TURNOUT ======================
#define SwAddr 100            // hard code address
#define midPos 90             // hard code positions
#define varPos 10             // gives ~1/4" throw on end of 1" long activating rod
//===================================================================================
#include <EEPROM.h>
#include <SoftwareServo.h>
#include <NmraDcc.h>
NmraDcc  Dcc ;
DCC_MSG  Packet ;
SoftwareServo myservo;        // create servo object to control a servo 
int pos = 0;                  // working Servo Position
int minPos = midPos - varPos; // establish min and max using mid +/- range
int maxPos = midPos + varPos;
int prevStateProper;          // 0=THROWN, 1=CLOSED
uint8_t StateProper;
#define servoPin 0            //Note dccPin=2 is defined in library
#define fRly 4

void setup() {
  pinMode(fRly,OUTPUT);
  digitalWrite(fRly,LOW);    //relay init at off
  Dcc.pin(0, 2, 0);                     //DCC signal D2
  Dcc.init( MAN_ID_DIY,100,FLAGS_DCC_ACCESSORY_DECODER,0);
  prevStateProper = EEPROM.read(5);
  if (prevStateProper != 0){
    prevStateProper = 1;                //ensure = 0 or 1
    EEPROM.update(5,prevStateProper);
  }
  if (prevStateProper == 0){            //last saved was a THROW, so restore
    setThrown();
  }else{
    setClosed();                        //last saved was a CLOSE, so restore 
  }
}

void loop(){
//=======================================================================
//     PROGRAM LOOP TO CHECK FOR DCC COMMANDS
//=======================================================================

  Dcc.process();                       //MUST call the NmraDcc.process() method frequently from
}                                      //the Arduino loop().

//=============================================================================
//          SERVICE DCC ACCESSORY COMMANDS
//=============================================================================
extern void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State ){
 if (Addr == SwAddr){                 //only respond for this address
    StateProper = OutputAddr & 0b00000001;  //state is the right most bit of cmd byte 2
    if (StateProper != prevStateProper){    //requesting new state?
      if (StateProper == 1 ){
        setClosed();
      }else{
        setThrown();
      }
    }
  }
}
void setThrown(){
//========================================================================
//         STEP SERVO TO THROWN POSITION.
//========================================================================
  myservo.attach(servoPin);
  delay(100);
  
  for(pos = minPos; pos <= maxPos; pos += 1){  // goes from minPos to maxPos
    myservo.write(pos);                        // move servo to 'pos' position 
    SoftwareServo::refresh();                  // must call at least once every 50ms
    delay(50);                                 // wait for servo to reach the position 
    if (pos == midPos){
      digitalWrite(fRly,HIGH);                 // activate frog relay at mid position        
    }
  }
  myservo.write(maxPos);                       // ensure at 'maxPos' position 
  SoftwareServo::refresh();
  delay(50);
  prevStateProper = 0;                         // remember just did thrown
  EEPROM.update(5,prevStateProper);            // save for next powerup
  myservo.detach();
}
void setClosed(){
//==========================================================================
//         STEP SERVO TO CLOSED POSITION.
//==========================================================================
  myservo.attach(servoPin);
  delay(100);
  
  for(pos = maxPos; pos >= minPos; pos-=1){    // goes from maxPos to minPos 
    myservo.write(pos);                        // move servo to 'pos' position 
    SoftwareServo::refresh();   
    delay(50);                                 // wait for the servo to reach the position 
   if (pos == midPos){
      digitalWrite(fRly,LOW);                  // turn off frog relay at mid position        
    }
  }
  myservo.write(minPos);                       // ensure at 'minPos' position 
  SoftwareServo::refresh();   
  delay(50);
  prevStateProper = 1;                         //  remember just did closed
  EEPROM.update(5,prevStateProper);            // save for next powerup
  myservo.detach();
 }
