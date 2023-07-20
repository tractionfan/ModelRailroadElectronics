/**************************************************************
     VisitorDetector.ino     NANO                 F.Miller 7/9/23
     This sketch turns on a (relay) when an object (person) is
     sensed within the current Distance setting. That Distance
     setting can be adjusted from 10 to 100cm.  The relay is kept
     on for an ajustable time of 0 to 60 sec.
 **************************************************************/
#define timePot A6
#define distPot A7
#define ledPin 5
#define trigPin 3                   //Chip 2 to HC-SR04 Trigger
#define echoPin 4                   //Chip 3 to HC-SR04 Echo
unsigned long duration;
unsigned long distance;
int d, testSens, sens;
boolean act = false;
unsigned long previousMillis = 0;     // store last time clock ticked
unsigned long detPreviousMillis = 0;  // store last detection time
unsigned long lagTime;        // time between checks for det & nag playing
unsigned long testTime;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  Serial.println("BEGIN");
  getMaxDist();                       //get initial max distance
  getTimer();                         //get initial timer
}

void loop() {

  if (act) {          //if active, look for end of timer
    if (millis() > previousMillis + lagTime) {
      previousMillis = millis();
      act = false;
      digitalWrite(ledPin, LOW);
      Serial.println("stop timer");
    }
  } else {             //not active see if new activity
    getTimer();
    getMaxDist();                 //see if max dist adjusted
    getDist();
    if (distance < sens) {
      digitalWrite(ledPin, HIGH);
      Serial.print(distance);
      Serial.println("cm");
      act = true;
      previousMillis = millis();    //reset timer
      Serial.println("start timer");
    }
//    delay(1000);
  }
}

/* ----------------------------------------------------------------
       Get detection distance in CM (Average of 10 readings)
   ----------------------------------------------------------------*/
void getDist() {
  distance = 0;
  for (int i = 0; i < 10; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = distance + (duration * 0.034 / 2);
  }
  distance =  distance / 10;              //get avg
}
/* ----------------------------------------------------------------
       Get (new) max distance adj setting in CM
       (Ignore new setting if within +/- 2 cm of old)
   ----------------------------------------------------------------*/
void getMaxDist() {
  d = analogRead(distPot);         //take reading from adjust pot
  testSens = map(d, 0, 1023, 5, 100);
  if (testSens <= sens - 2 || testSens >= sens + 2) {
    sens = testSens;
    Serial.print("Adj max Dist:");
    Serial.print(testSens);
    Serial.println("cm");
  }
  sens = testSens;
}
/* ----------------------------------------------------------------
       Get (new) Timer setting
       (Ignore new setting if within +/- 1 sec of old)
   ----------------------------------------------------------------*/
void getTimer() {
  d = analogRead(timePot);
  testTime = map(d, 0, 1023, 0, 60000);
  if (testTime <= lagTime - 1000 || testTime >= lagTime + 1000) {
    Serial.print("Adj Timer:");
    Serial.print(testTime/1000);
    Serial.println("sec");
  }
  lagTime = testTime;
}
