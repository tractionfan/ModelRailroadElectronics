/***************************************************************************
 * Fading Blinking LEDS for Barricade or Crossing Signals
 * F. Miller 1/16/18         (for ATTINY85)
 ***************************************************************************/

int ledPin1 = 0;    //0 for ATTINY85 CHIP-5 PWM CAPABLE
int ledPin2 = 1;    //1 FOR ATTINY85 CHIP-6 PWM CAPABLE

void setup() {      //nothing to do in setup
}

void loop() {
  for (int fadeUP = 0 ; fadeUP <= 255; fadeUP += 15) {
    analogWrite(ledPin1, fadeUP);
    int fadeDW = 255-fadeUP;
    analogWrite(ledPin2, fadeDW);
    delay(20);
  }
  delay(800);
  
  for (int fadeDW = 255 ; fadeDW >= 0; fadeDW -= 15) {
    analogWrite(ledPin1, fadeDW);
    int fadeUP = 255 - fadeDW;
    analogWrite(ledPin2, fadeUP);
    delay(20);
  }
  delay(800);
}
