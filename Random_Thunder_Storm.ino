/*  THUNDER STORM REV 3 by F. Miller 8/8/2017       (For ProMini and LED flood)
    ===========================================================================================================
    When activated (by touch pad) this sketch initiates random blinking of a LED flood light to simulate lightning
    and about a quarter way through that blinking a sound file is played simulating thunder.
    The thunder plays through to end of sound clip even though the lightning flashes have ceased.
    The above scenario is repeated three times using 3 unique sound clips and the 
    entire scenario (three pass-throughs) is again initiated by the touch pad.  The third pass
    has a shorter flashing period.
    The 12V LED flood is controlled by a MOSFET transistor running at two different brightness levels
    The MicroSDHC card in the DFPlayer has three sound files 
    ===========================================================================================================
*/
#include <CapacitiveSensor.h>       //Library providing touch pad sensor logic
CapacitiveSensor cs_4_2 = CapacitiveSensor(4,2);  // 1M resistor between pins 4 & 2, pin 2 is sensor pin
long capValue;                      // value returned from Cap Library reading
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11);    // RX to DFPlayer (not used), TX to DFPlayer (pin 11)
#define ledFlood 9                  // LED PWM pin control to MOSFET transistor
int ledState = LOW;                 // keep track of LED/Lightning state
unsigned long previousMillis = 0;   // store last time LED was updated
unsigned long currentMillis;        // capture current time
int randNumber;                     // develop a random number
boolean lightningActive = false;    // ligtning flashes active
boolean soundActive = false;        // would be true when DFPlayer is turned on to play sound
boolean stormActive = false;        // whole storm 'scenario' active
unsigned int lightMaxTime = 50;     // length of the 'scenario'
unsigned int thunderStartTime;      // begin thunder (~1/4) into lightning cycles
const int maxLoops = 3;             // maximum number of sequences (lightning/thunder sets)
int SeqCtr = 0;                     // count the number of sequences 
int lightTime = 0;                  // count steps through cycle (up to lightMaxTime
uint8_t send_buf[10]= { 0x7E, 0xFF, 0x06, 0x00, 0x00, 00, 00, 0x00, 0x00, 0xEF};  // Template for DFPlayer Commands.
// ===============================================================================================================
void setup() {
  //Serial.begin(57600);
  //Serial.println("Start");
  
  cs_4_2.set_CS_AutocaL_Millis(0xFFFFFFFF);     // turn off autocalibrate on channel 1
  analogWrite(ledFlood, 0);         //start with LED flood OFF
  randomSeed(analogRead(0));        //make 'perfectly' random
  lightMaxTime = 50;                //length of first and second seqence 
  thunderStartTime = 12;
  mySerial.begin(9600);
  delay (500);                      //  ** Required before 1st serial command to DFPlayer
  rst_cmd();          
  delay (1000);                     //  ** Required time for module reset 
  setvol_cmd(25); 
  delay (100);
  }
// ===============================================================================================================
void loop() { 
  capValue =  cs_4_2.capacitiveSensor(30);
  if (capValue > 500) {             //reset parms for another set of cycles through 'loop'
    lightMaxTime = 50;              //length of first and second seqence 
    thunderStartTime = 12;
    delay (1000);                   // give it 1-second before starting the storm
    SeqCtr = 0;
    reSet();
  }
  if (lightningActive & stormActive) {
    randNumber = random(10,100);
    delay(randNumber);              //random delay between each 'loop' while lightning active for random flash
    
    currentMillis = millis();
    if(currentMillis - previousMillis >= randNumber) {
      if (ledState == LOW){
        ledState = HIGH;
        if (SeqCtr < maxLoops-1) { //if NOT 3rd sequence do both high and low
          analogWrite(ledFlood, 255);  //LED flood Full ON
          delay(100);
        }
        analogWrite(ledFlood, 40); //LED flood Medium 
        } else {
        ledState = LOW;
        analogWrite(ledFlood, 0);   //LED flood OFF
        }
      previousMillis = currentMillis;        // reset for next time around     
    }
  } 
  if (stormActive & lightTime <= lightMaxTime) {
    lightTime++;                    //increment count of times through 'loop'
  }
  if (stormActive & lightningActive & (lightTime >= lightMaxTime)) {      //turn light OFF when completed prescribed times through 'loop'
    lightningActive = false;
    analogWrite(ledFlood, 0);       //ensure LED flood OFF
  }
  if (stormActive & lightningActive & lightTime == thunderStartTime) {                //time to do some thunder
    soundActive = true;
    playnext_cmd();                 //play (next)thunder sound
    delay(500);
  }
  if (soundActive & (SeqCtr < maxLoops) & analogRead(A0)>=500) {  // voltage reading >=500 shows DFPlayer stopped
    soundActive = false;
    SeqCtr++;
    reSet();
    if (SeqCtr == maxLoops) {
       stormActive = false;
    }
    if (SeqCtr == maxLoops-1) {
       lightMaxTime = 5;            //length of third sequence 
       thunderStartTime = 4;
    }  
  }
}
// ========================= common routine to reset all parms ====================
void reSet() { 
  lightningActive = true;
  stormActive = true;
  lightTime = 0;
  soundActive=false;
  ledState = LOW;
  previousMillis = currentMillis;
}

// =================== complete & send DFPlayer command line =====================
void send_cmd() {
  uint16_t sum = 0;      //calc checksum from bytes 1-6
  for (int i=1; i<7; i++) {
    sum += send_buf[i];
  }
  send_buf[7] = -sum>>8;  //place hi into byte 7
  send_buf[8] = -sum;     //place lo into byte   
  for (int i=0; i<10; i++) {
    mySerial.write(send_buf[i] );
  }
}
void playnext_cmd() {
  send_buf[3] = 0x01;
  send_buf[6] = 0x00;    // no parm
  send_cmd();
}
void setvol_cmd(int vol) {
  send_buf[3] = 0x06;    // cmd
  send_buf[6] = vol;     // specific vol
  send_cmd();
}

void rst_cmd() {
  send_buf[3] = 0x0C;    // cmd
  send_buf[6] = 0x00;    // no parm
  send_cmd();
}
