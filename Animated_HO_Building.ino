/******************************************************************************************** 
 *  Animated_HO_Building (Formally called Multi-Function Animation Decoder (MFAD)-Version 8a  
 *  F. Miller 6/27/16
 *                  Default CVs set for LN#1000 Standard Demo Board
 ********************************************************************************************  
 * MFAD - Led Flickers, Led Blink, LED on/off, Play sound tracks, operate Servo
 * VERSION 8a uses Fct 5 to operate LED on pin 5, taking away alternate blinking on pin 4 & 5 
 * and abandoning playing sound track 1.
 * If FxMode CV is set sound clips will stop at the end, otherwise repeating until Function 
 * command is turned off;
 * When servoTrack (CV40) or flickerTrack(CV39) are non-zero (and point to a sound
 * track) the servo or flicker "macros" are activated so that the sound track plays
 * during the operation of the flicker or servo activity.  The sounds and activities
 * are terminated when reaching end of sound (flicker) or top/bottom of servo motion.
 * Notes: MiniPro must be RESET to read any new CVs
 *        This version does NOT use the Arduino DELAY() function in Servo operations
 *        Very simplified NMRA lib setup for 4-digit addresses (128-9983)
 *        CVs are used for operational parameters. If jumper in - loads program
 *        CV defaults (table in program); jumper out - retains existing (EPROM stored) 
 *        values including address.  Normally run (and change CVs) with jumper out
 * *****************************************************************************
 * F0 - Flicker LED on pin 3 
 * F1 - Blink LED on pin 4
 * F2 - ON/OFF LED on pin 6
 * F3 - ON/OFF LED on pin 7
 * F4 - ON/OFF LED on pin 8
 * F5 - ON/OFF LED on pin 5
 * F6 - Play Track 2  (leave ON for continuous)
 * F7 - Play Track 3  (leave ON for continuous)
 * F8 - ON - Move Servo up, OFF - Move Servo down Pin 9
 * ******************************************************************************
 * This sketch makes use of an early NmraDcc support library by Alex Shepherd.  I have
 * modified the library name (NmraDcc1) for my convenience.  The sketch also uses
 * Arduino libraries SoftwareSerial and Servo. The SoftwareSerial library is used to
 * send commands to the DFPplayer sound module.
 ***********************************************************************************/
unsigned long Decoder_Address;      // can be 128-9983 (Digitrax) set from table CVs 37,38
boolean LoadCVs = true;             // false will preserve existing decoder CVs
#include <NmraDcc1.h>
#define numleds  9
byte ledpins [] = {3,4,5,6,7,8,9,10,11};
//      NMRA Interface Pin 2
#define flickerPin 3                // F0 - Flicker LED
#define blinkPin1 4                 // F1 - Blink1 LED
#define F5Pin 5                     // F5 ON/OFF LED
#define F2Pin 6                     // F2 ON/OFF LED
#define F3Pin 7                     // F3 ON/OFF LED
#define F4Pin 8                     // F4 ON/OFF LED
#define F8Pin 9                     // F8 Servo Control
#define CVPin 10                    // if pulled low, load default CVs
//      DFPlayer RX Pin 14/A0
boolean flickerON = false;
boolean flickerMacroON = false;
int flickerState = LOW;             // keep track of LED/Lightning state
unsigned long flickerpreviousMillis = 0;   // store last time LED was updated
unsigned long flickerMillis;        // capture current time
//--------------------------------------------------------------------
int randNumber;             
boolean blinkON = false;            // F1 state
int blinkState = LOW;               // keep track of LED/Lightning state
unsigned long blinkpreviousMillis = 0;   // store last time LED was updated
unsigned long blinkMillis;          // capture current time
int blinkTime;                      //blink rate in ms (4xCV32)             
//--------------------------------------------------------------------
struct CVPair {
  uint16_t  CV;
  uint8_t   Value;
};
CVPair FactoryDefaultCVs [] = {
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, (Decoder_Address >>8) | 192 },
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, Decoder_Address & 255},
  {CV_29_CONFIG, 34},               // Long  Address 28/128 Speed Steps  
  {30, 95},                         //SERVO HI POSITION (Best for shades)
  {31, 80},                         //SERVO LO POSITION (Best for shades)
  {32, 100},                        //BLINK RATE (stored as 1/4 value desired)
  {33, 1},                          //1=sound off when reach end of track, 0=continuous until Fct stopped
  {34, 1},                          //not used (see CV40)
  {35, 2},                          //F6 PLAY TRACK NO.
  {36, 3},                          //F7 PLAY TRACK NO.
  {37,10},                          //default Address MSB
  {38,00},                          //default Address LSB
  {39,2},                           //Track to play with "macro" flicker (0=no macro)
  {40,1},                           //Track to play with "macro" servo (0=no macro)
  {41,25},                          //Playback volume (25 good for demo)
};
uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
NmraDcc  Dcc ;
DCC_MSG  Packet ;

//---------------------------------------------------------------------
#include <SoftwareSerial.h>
SoftwareSerial mySerial(99, 14); //no Arduino RX, Pin 14 for Arduino TX to DFPlayer RX .
uint8_t send_buf[10]= { 0x7E, 0xFF, 0x06, 0x00, 0x00, 00, 00, 0x00, 0x00, 0xEF};  // Template Command.
// DFPlayer command format: (0-9)Byte 3 (cmd) 5 (parm), 6-7 (Checksum) need to be set
boolean soundON = false;
#define playingPin 15               // DFPlayer playing status  (Pin 15/A1) LOW = playing, HIGH stopped
int track;
int F6Track;
int F7Track;
int FlickerTrack;                   // match track to flicker "macro" (0= no macro)
int ServoTrack;                     // match track to servo "macro" (0= no macro)
boolean FxOFFmode;                  // (CV33) true (=1) sound off when FX goes off, else plays to end
int playVol;                        // (CV41) DFPlayer volume setting
//-----------------------------------------------------------------------
#include <Servo.h> 
Servo myservo;  // create servo object to control a servo 
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))
int current_position;
int botPosition;                    //lowest Servo position (CV31)
int increment = 1;
int topPosition;                    //highest Servo position (CV30)
boolean servoinuse;
boolean servoTop = false;           //true when went from botPositon -> topPosition
boolean servoBot = true;            //true when went from topPosition -> botPosition (start position)
boolean servoON = false;
boolean servoMacroON = false;
//----------------------------------------------------------------------
struct Fcts {                       //table to keep track of CHANGED function settings
  int oldFct;
  int newFct;
  boolean chgFct;
};
Fcts FctStates [] = {
  {0,0,false}, //Fct0
  {0,0,false}, //Fct1
  {0,0,false}, //Fct2
  {0,0,false}, //Fct3
  {0,0,false}, //Fct4
  {0,0,false}, //Fct5
  {0,0,false}, //Fct6
  {0,0,false}, //Fct7
  {0,0,false}, //Fct8
};
//----------------------------------------------------------------------------------------------
void setup() {
  pinMode(playingPin,INPUT);        //DFPlayer Play Status
  mySerial.begin(9600);
  delay (500);                      //** Required before 1st serial command to DFPlayer
  rst_cmd();          
  delay (1000);                     //** Required time for module reset 
  randomSeed(analogRead(0));        //make 'perfectly' random
  for (int i=0; i< numleds; i++) {  // initialize the digital pins as an outputs
     pinMode(ledpins[i], OUTPUT);
  }
  Dcc.pin(0, 2, 0);
  Dcc.init(0,0,1,0);
  pinMode(CVPin,INPUT_PULLUP);
  
  if (digitalRead(CVPin) == LOW) {  //jumper IN: load table default CVs including address    Decoder_Address = (FactoryDefaultCVs[10].Value * 100) + FactoryDefaultCVs[11].Value;
    Decoder_Address = (FactoryDefaultCVs[10].Value * 100) + FactoryDefaultCVs[11].Value;
    FactoryDefaultCVs[0].Value = (Decoder_Address >>8) | 192;
    FactoryDefaultCVs[1].Value = Decoder_Address & 255;
    for (int j=0; j < FactoryDefaultCVIndex; j++ )
         Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
  } else {         //jumper OUT: retain all stored previous CVs including Address  
    Decoder_Address = (Dcc.getCV(37) * 100) + Dcc.getCV(38);
    FactoryDefaultCVs[0].Value = (Decoder_Address >>8) | 192;
    FactoryDefaultCVs[1].Value = Decoder_Address & 255;
    for (int j=0; j < 3; j++ )
         Dcc.setCV( FactoryDefaultCVs[j].CV, FactoryDefaultCVs[j].Value);
  }
  current_position = botPosition;
  myservo.attach(F8Pin);            // attaches servo on pin 9 to the servo object 
  servoinuse = false;
  playVol = Dcc.getCV(41);
  blinkTime = 4*Dcc.getCV(32);      //extract operating parms from CVs  
  botPosition = Dcc.getCV(31);
  topPosition = Dcc.getCV(30);
  FxOFFmode = Dcc.getCV(33);        // true (=1) sound off when FX goes off, else plays to end
  F6Track = Dcc.getCV(35);          // sound track to play with F6
  F7Track = Dcc.getCV(36);          // sound track to play with F7
  FlickerTrack = Dcc.getCV(39);     // match track to flicker "macro" (0= no macro)
  ServoTrack = Dcc.getCV(40);       // match track to servo "macro" (0= no macro)
  playVol = Dcc.getCV(41);
  setvol_cmd(playVol);
  delay (100);
  myservo.write(botPosition);       // position servo to start (bottom) position 
  delay(15);                        // waits 15ms for the servo to reach the position 
  playtrk_cmd(4);                   // startup message
  delay(100);
  /*
  Serial.begin(57600);
  Serial.println("start");
  Serial.print("Address:");
  Serial.println(Decoder_Address);
  
  for (int i=30; i< 42; i++) {      // display CVs 30-41
     Serial.print(i);
     Serial.print(":");
     Serial.println(Dcc.getCV(i));
  }
  */
}
//-------------------------------------------------------------------------
void loop() {
  Dcc.process();                    //MUST do in each loop
  checkButtons();                   // see if any fct buttons changed & do activity
    
  if (digitalRead(playingPin) == HIGH && !FxOFFmode && track !=0 && !flickerON){  //goes HIGH when playing stops after was playing
     switch (track) {
      case 1: playtrk_cmd(ServoTrack); break;
      case 2: playtrk_cmd(F6Track); break;
      case 3: playtrk_cmd(F7Track); break;
    }
  }
    
  if (digitalRead(playingPin) == HIGH && track >0 && flickerON && flickerMacroON){  //goes HIGH when playing stops after was playing
    flickerMacroON = false;
    flickerON = false;
    soundON = false;
    track = 0;
    }
   
  if (flickerON) {
    randNumber = random(1,50);      //On/off time range
    flickerpreviousMillis = flickerMillis;
    flickerMillis = millis();
    if(flickerMillis - flickerpreviousMillis >= randNumber) {
      if (flickerState == LOW){
         flickerState = HIGH;
         digitalWrite(flickerPin, flickerState);  //Turn LED on
      }else{
         flickerState = LOW;
         digitalWrite(flickerPin, flickerState);  //Turn LED off
      }
    }
  } else {                                  
    digitalWrite(flickerPin, LOW);                //flicker is OFF, ensure LED off at end
  }

  if (blinkON) {
    blinkMillis = millis();
    if(blinkMillis - blinkpreviousMillis >= blinkTime) {
      if (blinkState == LOW){
         blinkState = HIGH;
         digitalWrite(blinkPin1, blinkState);  //Turn LED1 on
      }else{
         blinkState = LOW;
         digitalWrite(blinkPin1, blinkState);  //Turn LED1 off
      }
    blinkpreviousMillis = blinkMillis;
    }
  } else {
    digitalWrite(blinkPin1, LOW);             //ensure LED1 off
  }
  
  //--------------------- Servo Operation and Limits check ------------------------
  
  runEvery(75) {                             //run every 75 milliseconds
  if (servoinuse) {
     if (increment > 0) {
       if (current_position > topPosition){ 
         current_position = topPosition;
         servoTop = true;
         servoBot = false;
         servoinuse = false;
         //Serial.println("top");
         if (servoMacroON) {
            repStop_cmd();
            soundON = false;
            track = 0;
            servoMacroON = false;
         }
       }
    } 
    if (increment < 0) { 
       if (current_position < botPosition) { 
         current_position = botPosition;
         servoBot = true;
         servoTop = false;
         servoinuse = false;
         //Serial.println("Bot");
         if (servoMacroON) {
            repStop_cmd();
            soundON = false;
            track = 0;
            servoMacroON = false;
         }
       }
    }
    current_position = current_position + increment;
    //Serial.println(current_position);
    myservo.write(current_position);  
    }
  }
}

/* ----------------------------------------------------------------
  Check if any Function keys have changed and do work as applicable
   ----------------------------------------------------------------*/
void checkButtons() {
  if (FctStates[0].chgFct) {
    flickerON = FctStates[0].newFct;                //flicker set ON or OFF per new F0 state
    FctStates[0].chgFct = false;
    
    if (flickerON && FlickerTrack >0) {
      flickerMacroON = true;
      playtrk_cmd(FlickerTrack);
      soundON = true;
      track = FlickerTrack;
    }
    if (!flickerON && FlickerTrack >0) {
      flickerMacroON = false;
      repStop_cmd();
      soundON = false;
      track = 0;
    }
    
  }
  
  if (FctStates[1].chgFct) {
    blinkON = FctStates[1].newFct;                  //blinker set ON or OFF per new F1 state
    FctStates[1].chgFct = false;
  }  
  if (FctStates[2].chgFct) {
    digitalWrite( F2Pin, FctStates[2].newFct);       //lite 2 set ON or OFF per new F2 state
    FctStates[2].chgFct = false;
  }  
  if (FctStates[3].chgFct) {
    digitalWrite( F3Pin, FctStates[3].newFct);       //lite 3 set ON or OFF per new F3 state
    FctStates[3].chgFct = false;
  }  
  if (FctStates[4].chgFct) {
    digitalWrite( F4Pin, FctStates[4].newFct);       //lite 4 set ON or OFF per new F4 state
    FctStates[4].chgFct = false;
  }  
  if (FctStates[5].chgFct) {
    digitalWrite( F5Pin, FctStates[5].newFct);       //lite 5 set ON or OFF per new F5 state
    FctStates[5].chgFct = false;
  }  
  if (FctStates[6].chgFct) {                         //Play F6Track or stop per new F5 state
    if (FctStates[6].newFct && !soundON) {
       playtrk_cmd(F6Track);
       soundON = true;
       track = 2;
    }
    else if (!FctStates[6].newFct && soundON) {
        repStop_cmd();
        soundON = false;
        track = 0;
    }
    FctStates[6].chgFct = false;
  }
  if (FctStates[7].chgFct) {                         //Play F7Track or stop per new F5 state
    if (FctStates[7].newFct && !soundON) {
       playtrk_cmd(F7Track);
       soundON = true;
       track = 3;
    }
    else if (!FctStates[7].newFct && soundON) {
        repStop_cmd();
        soundON = false;
        track = 0;
    }
    FctStates[7].chgFct = false;
    }
  if (FctStates[8].chgFct) {
    servoON = FctStates[8].newFct;
    moveServo(servoON);                  //run Servo UP/Dwn per new F8 state
    FctStates[8].chgFct = false;
    if (ServoTrack > 0) {                //if not 0 means servoMacro called for
      servoMacroON = true;
      playtrk_cmd(ServoTrack);
      soundON = true;
      track = ServoTrack;
    }
  }
}
/* ----------------------------------------------------------------
        Monitor status of function keys
   ----------------------------------------------------------------*/
extern void notifyDccFunc( uint16_t Addr, uint8_t FuncNum, uint8_t FuncState)  {
  if (FuncNum == 1) {                   //Function Group 1 F0 F4 F3 F2 F1
    testFct((FuncState&0x10)>>4, 0);    //Function F0
    testFct((FuncState&0x01), 1);       //Function F1
    testFct((FuncState&0x02)>>1 , 2);   //Function F2
    testFct((FuncState&0x04)>>1, 3);    //Function F3
    testFct((FuncState&0x08)>>1, 4);    //Function F4  
  }
  else if (FuncNum==2) {                  
    if ((FuncState & 0x10)==0x10)  {    //Function Group 2 F8 F7 F6 F5 (set 1)
      testFct((FuncState&0x01),5);      //Function F5
      testFct((FuncState&0x02)>>1,6);   //Function F6
      testFct((FuncState&0x04)>>2,7);   //Function F7
      testFct((FuncState&0x08)>>3,8);   //Function F8
    }
  }
}
/* ----------------------------------------------------------------
        Note any changes to Function keys
   ----------------------------------------------------------------*/
void testFct(int state, int fct) {
   FctStates[fct].newFct = state;
   if (FctStates[fct].newFct != FctStates[fct].oldFct) {
     FctStates[fct].oldFct = FctStates[fct].newFct;    //save new
     FctStates[fct].chgFct = true;
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
  myservo.detach();       //detaches servo (pin 9) to prevent my Serial Interference 
  for (int i=0; i<10; i++) {
     mySerial.write(send_buf[i] );
  }
  myservo.attach(F8Pin);            // re-attaches servo on pin 9 to the servo object
  
}
// ======================== Individual DFPlayer Commands ========================
void setvol_cmd(int vol) {
  send_buf[3] = 0x06;    // cmd
  send_buf[6] = vol;     // specific vol
  send_cmd();
}
void playtrk_cmd(int trk) {
  //Serial.print("Playing ");
  //Serial.println(trk);
  send_buf[3] = 0x03;   // cmd 
  send_buf[6] = trk;    // specific track
  send_cmd();
  while (digitalRead(playingPin) == HIGH) {};   //hold till DFPlayer shows its playing
}
void rst_cmd() {
  send_buf[3] = 0x0C;    // cmd
  send_buf[6] = 0x00;    // no parm
  send_cmd();
}
void repPlay_cmd() {
  send_buf[3] = 0x11;    // cmd 
  send_buf[6] = 0x01;    // repeat all tracks
  send_cmd();
}
void repStop_cmd() {
  for (int v = playVol; v > 5; v-=1) {   //reduce volume in steps
    setvol_cmd(v);
    delay(100);
  }
  send_buf[3] = 0x11;    // cmd       // do actual stop
  send_buf[6] = 0x00;    // stop all playing
  send_cmd();
  delay(100);
  setvol_cmd(playVol);      //restore volume to original
  delay(100);
}
//--------------------------- Servo Initialization --------------------------------
void moveServo (int FuncState) {  //(=0 F8 off, or 1 F8 on)
    if (FuncState==1 && !servoinuse && servoBot){
      current_position = botPosition;
      servoinuse = true;
      //Serial.println("Servo+");
      increment = 1;
    }
    if (FuncState==0 && !servoinuse && servoTop){
      current_position = topPosition;
      //Serial.println("Servo-");
      servoinuse = true;
      increment = -1;
    }
}
