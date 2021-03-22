//==========================================================================
//  UNCOUPLE RAMP PULSE TIMING CIRCUIT  (ATTINY85)          F.Miller 9/17/20
//  An input  pulldown pulse of greater than 10 ms causes a 5 second output
//  positive pulse to hold uncoupling solenoid on (via an issolated 24VDC
//  MOSFET circuit.)  Input pulse is 'debounced' to remove noise.
//==========================================================================
#define DEBOUNCE 10             // ms to debounce, 5+ ms is usually plenty
#define PB 4                    //physical pin 3
#define outPulse 2              //physical pin 7
#define pulseSec 5              //length of output pulse in seconds
byte pressed;
byte justpressed;
byte justreleased;
//===========================================================================
void setup() {
  pinMode(PB, INPUT);
  pinMode(outPulse,OUTPUT);
  digitalWrite(PB, HIGH);
  justpressed=0;
  justreleased=0;
}
//=============================================================================
void loop() {
  check_switches();                  // check switches to get the current state
  if (justpressed==1) {
      justpressed=0;
      digitalWrite(outPulse,HIGH);
      delay(pulseSec*1000);          //make a 5 sec pulse
      digitalWrite(outPulse,LOW);      
    }
    if (justreleased==1) {
      justreleased=0;
//      Serial.println("Released");
     }
}
//=============================================================================
void check_switches() {
  static byte previousstate;
  static byte currentstate;
  static long lasttime;
  if (millis() < lasttime) {               // timer wrapped around, ignore this one
     lasttime = millis();
  }
  if ((lasttime + DEBOUNCE) > millis()) {  // still within debounce time
      return; 
  }
  lasttime = millis();                     //past debounce period, reset timer
  currentstate = digitalRead(PB);          // read the button
    if (currentstate == previousstate) {
      if ((pressed == LOW) && (currentstate == LOW)) {  // just pressed
            justpressed = 1;
      }
      else if ((pressed == HIGH) && (currentstate == HIGH)) {  // just released
          justreleased = 1;
       }
      pressed = !currentstate;
    }
    previousstate = currentstate;          // keep button status
}
//==============================================================================
