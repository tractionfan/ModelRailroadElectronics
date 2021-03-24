/*======================================================================================
IR Differential Detection - IR_DET_85V4-BEAM,  For ATTINY85 - F.Miller 08/15/20
  Version 4 - Reversed Output - ON if NO detect (ie beam interupted), OFF if IR detected
========================================================================================
This code removes ambience noise from sensor data by comparing received IR with and
without the IR LED on, making the circuit more sensative.  This provides
detection in very higher ambient light, sun, etc. The active circuit uses a 4N35 Optical Coupler to issolate sensing
circuit (powered by 6VAC BUS) 
------------------------ Connections ------------------------------------------
The IR Detector and IR LED are LTR-301 LTE-302 units.
IR LED connected to Digital pin: D4 thru 470 ohms
IR diode connected to analog input: A3 and also to
    +5 thru 10K ohms
Indicator LED connected from D0 and in series with a 1K resistor and the 4N35 optocoupler
=========================================================================================*/
int a,b,c;                        //used for analog readings and calculations
#define IRLed 4                   //chip 3, Digital 4
#define IRdet A3                  //chip 2, Analog 3   (NOTE-earlier version used A1
#define indLed 0                  //chip 5, Digital 0 

void setup() {
  pinMode(indLed,OUTPUT);         // led as indicator output
  pinMode(IRLed,OUTPUT);
}

void loop() {
  digitalWrite(IRLed,HIGH);       //Turning ON IR LED
  delay(1);                       //Microseconds(500);  //wait
  a=analogRead(IRdet);            //take reading from photodiode(pin A3) :noise+signal
  digitalWrite(IRLed,LOW);        //turn Off IR LED
  delay(1); //Microseconds(500);  //short wait
  b=analogRead(IRdet);            // again take reading from photodiode :noise
  c = abs(b - a);                 //taking differnce:[ (noise+signal)-(noise)] just signal

  if (c <= 50) {                  // 5 most sensitive, 50 least sensitive but works fine
    digitalWrite(indLed, HIGH);   // Turn on the LED
  }else{
    digitalWrite(indLed, LOW);    // Turn off the LED
    delay(100);                   //leave ON for a pulse  (3 sec may be useful at times)
  }
  delay(100);
}
