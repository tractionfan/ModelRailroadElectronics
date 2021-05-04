/****************************************************************************  
 *   Campfire Flicker - F.Miller 12/2018       (For ATTINY85)
 *   Random flickering of an LED to simulate a campfire
 ****************************************************************************/

const int firePin =  0;             // (CHIP-5) the number of the LED pin
int ledState = LOW;                 // keep track of LED/Lightning state
unsigned long previousMillis = 0;   // store last time LED was updated
unsigned long currentMillis;        // capture current time
int randNumber;                     // develop a random number


void setup() {
  pinMode(firePin, OUTPUT);         // (0) goes to LED 
}

void loop() { 
                                            // main running mode
       if (ledState == LOW){
          randNumber = random(1,20);        // off range
       } else {
          randNumber = random(1,100);       // On range
       }   
       doit();
}
void doit() {                               // Function to flip ON/OFF after delay
    delay(randNumber);    
    currentMillis = millis();
    if(currentMillis - previousMillis >= randNumber) {
      if (ledState == LOW){
         ledState = HIGH;
         digitalWrite(firePin, ledState);   // Turn LED on
      }else{
         ledState = LOW;
         digitalWrite(firePin, ledState);   // Turn LED off
      }
      previousMillis = currentMillis;       // reset for next time around     
   }
}
