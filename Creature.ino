#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <NewPing.h>
#include <AccelStepper.h>
#include <EightAccelStepper.h>
#include <CreatureMover.h>
#include <RingBuffer.h>
#include <Positioner.h>

/******************************
 * Creature
 *
 * The creature will move to the middle position at the start of the program
 * It should wait for 10s before moving anywhere else
 *
 *
 ******************************/
 
 
/******************************
 * Creature motor setup
 ******************************/
/*
int aSteps[3][3] = {
    {-1258, -572, 312} // LEFT
  , {-657, 0, 642}     // MID
  , {50, 446, 1104}    // RIGHT
};

int bSteps[3][3] = {
    {100, -637, -1135}  // LEFT
  , {738, 0, -760}    // MID
  , {1401, 508, -384}  // RIGHT
};
*/
/*
int aSteps[3][3] = {
    {-1501,   -519,   462} // LEFT
  , {-762,     0,     813}     // MID
  , {33,     494,     1445}    // RIGHT
};

int bSteps[3][3] = {
    {-60,     -590,   -1352}  // LEFT
  , {859,     0,      -953}    // MID
  , {1678,    561,    -554}  // RIGHT
};
*/

int aSteps[3][3] = {
    {-957,   -672,   -283} // LEFT
  , {-230,     0,     312}     // MID
//  , {726,     920,     1051}    // RIGHT
  , {576,     845,     1051}    // RIGHT
};

int bSteps[3][3] = {
    {-927,     -1086,   -1336}  // LEFT
  , {186,     0,      -380}    // MID
  , {827,    381,    35}  // RIGHT
};
/*
int aSteps[3][3] = {
    {-957,   -672,   -283} // LEFT
  , {-230,     0,     312}     // MID
  , {726,     920,     1051}    // RIGHT
};

int bSteps[3][3] = {
    {-927,     -1136,   -1436}  // LEFT
  , {186,     0,      -380}    // MID
  , {827,    381,    35}  // RIGHT
};
/*
int aSteps[3][3] = {
    {-320,   -230,   -95} // LEFT
  , {-70,     0,     106}     // MID
  , {240,     308,     350}    // RIGHT
};

int bSteps[3][3] = {
    {-310,     -380,   -450}  // LEFT
  , {62,     0,      -128}    // MID
  , {267,    128,    12}  // RIGHT
};
*/
int aMaxSpeed = 100; // 700 225
int bMaxSpeed = 100; // 700 275
int idleDistance = 0; // 25
int acceleration = 25;

EightAccelStepper stepperA(4, 28, 30, 32, 34);
EightAccelStepper stepperB(4, 23, 25, 27, 29);

CreatureMover creature(stepperA, aMaxSpeed, aSteps,
                       stepperB, bMaxSpeed, bSteps,
                       idleDistance, acceleration);
          
int creaturePosition = 0;
unsigned long idleTime = 0;
int creatureReady = 0;  // if the creature is ready to start moving
unsigned long creatureStartTime;
unsigned long creatureStartDelay = 10000;  // delay before moving the creature around

/******************************
 * QR Code setup
 ******************************/
bool qrEnabled = false;

int qrStepPin = 6;
int qrDirPin = 7;
AccelStepper stepperQr(1, qrStepPin, qrDirPin);

int qrDist = 900;
unsigned long qrIdleTime = 0;
unsigned long qrUpWaitTime = 20000;
unsigned long qrDownWaitTime = 15000;
unsigned long qrSaveInterval = 2000;
unsigned long qrLastSaved = 0;
int qrState = 1;  // 0: idle, 1: moving
bool qrUp = true;

/******************************
 * Positioner setup
 ******************************/

Positioner positioner(2); // specify how many times to visit each position
float aveDistances[3] = {0.0, 0.0, 0.0};
 
/******************************
 * Distance sensor setup
 ******************************/

#define SONAR_NUM     3 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 333 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

int currentSensor = 0;
int leftCurr = 0;

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(48, 49, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(50, 51, MAX_DISTANCE),
  NewPing(52, 53, MAX_DISTANCE)
};

RingBuffer sonarDistances[SONAR_NUM] = {     // Sensor object array.
  RingBuffer(10), // Each sensor's trigger pin, echo pin, and max distance to ping.
  RingBuffer(10),
  RingBuffer(10)
};
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.

unsigned long pingTimer[SONAR_NUM];


    
/*******************************
 * Switch sensor setup
 ******************************/

int buttonPin = 2;


/******************************
 * LED setup
 ******************************/       
int ledPin = 4; 
unsigned long lastBlink = 0;  
int minBrightness = 0;
int maxBrightness = 255;
int brightness = minBrightness;    // how bright the LED is
int fadeAmount = 10;    // how many points to fade the LED by  


/******************************
 * Green LED setup
 ******************************/       
int ledGreenPin = 5; 
unsigned long lastGreenBlink = 0;  
int minGreenBrightness = 50;
int maxGreenBrightness = 255;
int brightnessGreen = minGreenBrightness;    // how bright the LED is
int fadeGreenAmount = 2;    // how many points to fade the LED by


/******************************
 * Fortune setup
 ******************************/  
unsigned long lastFortune = 0;
unsigned long fortuneIdleTime = 18000;
unsigned long now = 0;
int showingFortune = 0;


/******************************
 * Arduino setup
 ******************************/  
void setup() {
   Serial.begin(9600);           // set up Serial library at 9600 bp
   Serial.println("setup");
   creature.moveToPosition(CreatureMover::XMID, CreatureMover::YMID);
   creatureStartTime = millis() + creatureStartDelay;
   
  pingTimer[0] = millis() + 100;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) { // Set the starting time for each sensor.
      pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
  currentSensor = 0; 
  
  pinMode(buttonPin, INPUT);
  pinMode(ledGreenPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  attachInterrupt(0, buttonClick, FALLING);  
//  creaturePosition = 4;
  if (qrEnabled) {
    stepperQr.setMaxSpeed(5000);
    stepperQr.setAcceleration(1000);
    getQrPosition();
  }
}


/******************************
 * Arduino loop
 ******************************/
void loop() {
  
  sensorRun();
  fortuneRun();
  creature.run();
  if (qrEnabled) {
    qrRun();
  }
//  greenLedRun();
  
  if (creatureReady) {
    if (creature.getState() == CreatureMover::STOPPED) {
      creatureIdle(3000);
    } else if (creature.getState() == CreatureMover::IDLE
               && millis() > idleTime
    ) {

      
      for (int i = 0; i < SONAR_NUM; i++) {
        aveDistances[i] = getSensorAverage(i);
      }
      /*
      Serial.print("Distances(");
      Serial.print(aveDistances[0]);
      Serial.print(", ");
      Serial.print(aveDistances[1]);
      Serial.print(", ");
      Serial.print(aveDistances[2]);
      Serial.println(")");
      */
      creaturePosition = positioner.getNextPosition(aveDistances);

      //Serial.print("Found position:");
      //Serial.println(creaturePosition);

      switch(creaturePosition) {
        case 0:
          creature.moveToPosition(CreatureMover::XLEFT, CreatureMover::YTOP);
          break;
        case 1:
          creature.moveToPosition(CreatureMover::XLEFT, CreatureMover::YMID);
          break;
        case 2:
          creature.moveToPosition(CreatureMover::XLEFT, CreatureMover::YBOTTOM);
          break;
        case 3:
          creature.moveToPosition(CreatureMover::XMID, CreatureMover::YTOP);
          break;
        case 4:
          creature.moveToPosition(CreatureMover::XMID, CreatureMover::YMID);
          break;
        case 5:
          creature.moveToPosition(CreatureMover::XMID, CreatureMover::YBOTTOM);
          break;
        case 6:
          creature.moveToPosition(CreatureMover::XRIGHT, CreatureMover::YTOP);
          break;
        case 7:
          creature.moveToPosition(CreatureMover::XRIGHT, CreatureMover::YMID);
          break;
        case 8:
          creature.moveToPosition(CreatureMover::XRIGHT, CreatureMover::YBOTTOM);
          break;
      }
/*      
      creaturePosition++;
      if (creaturePosition > 8) {
        creaturePosition = 0;
      }
*/
/*
      if (creaturePosition == 4 || creaturePosition == 2) {
        creaturePosition = 1;
      } else if (creaturePosition == 1) {
        creaturePosition = 0;
      } else {
        creaturePosition = 2;
      }   
*/
    }
  } else if (millis() > creatureStartTime) {
    creatureReady = 1;
    Serial.println("Creature is ready");
  }
}

void creatureIdle(unsigned long creatureIdleTime) {
    idleTime = millis() + creatureIdleTime;
    creature.idle();
}

void sensorRun() {
  
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
}


void fortuneRun() {
  
  if (showingFortune == 1) {
    now = millis();
    if (now > lastBlink + 50) {
      // set the brightness of pin 9:
      analogWrite(ledPin, brightness);    
    
      // change the brightness for next time through the loop:
      brightness = brightness + fadeAmount;
    
      // reverse the direction of the fading at the ends of the fade: 
      if (brightness <= minBrightness || brightness >= maxBrightness) {
        fadeAmount = -fadeAmount ; 
      }
      lastBlink = now;
    }
    if (now > lastFortune + fortuneIdleTime) {
      showingFortune = 0;
      analogWrite(ledPin, 0); // turn off LED   
      // set creature moving again
    }
  }
}


void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer()) {
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
  }
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
 
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    sonarDistances[i].push(cm[i]);
  }

  Serial.print("Left Average:");
  Serial.println(getSensorAverage(0));
  Serial.print("Right Average:");
  Serial.println(getSensorAverage(1));
  Serial.print("Center Average:");
  Serial.println(getSensorAverage(2));

}

float getSensorAverage(int sensor) {
    float ave = sonarDistances[sensor].getAverage();
    if (ave == 0.0) {
      ave = MAX_DISTANCE;
    }
    return ave;
}
int getSensorMin(int sensor) {
    return sonarDistances[sensor].getMinimum();
}
int getSensorMax(int sensor) {
    return sonarDistances[sensor].getMaximum();
}
int getSensorCurrent(int sensor) {
    return sonarDistances[sensor].peek();    
}

void buttonClick() {
  if (!showingFortune) {
    Serial.println("Show fortune");
    showingFortune = 1;
    lastFortune = millis();
    brightness = minBrightness;
    fadeAmount = 5;
    // set creature to idle
    creatureIdle(fortuneIdleTime);
  }
}


void qrRun()
{
  if (qrState) {
    if (stepperQr.distanceToGo() != 0) {
      stepperQr.run();
    } else {
      qrIdleTime = millis() + (qrUp ? qrUpWaitTime : qrDownWaitTime);
      qrUp = !qrUp;
      qrState = 0;      
    }    
    saveQrPosition();
  } else if (millis() > qrIdleTime) {
    stepperQr.move(qrUp ? qrDist : -qrDist); 
    qrState = 1;
  }    
}

void getQrPosition() {
    int posQr;
    EEPROM_readAnything(4, posQr);    
    stepperQr.setCurrentPosition(posQr);
    stepperQr.move(qrDist - stepperQr.currentPosition());
}

void saveQrPosition() {
  if (millis() > qrLastSaved + qrSaveInterval) {
      qrLastSaved = millis();
      EEPROM_writeAnything(4, (int) stepperQr.currentPosition());
  }
}

void greenLedRun() {
 
  now = millis();
  if (now > lastGreenBlink + 50) {
    // set the brightness of pin 9:
    analogWrite(ledGreenPin, brightnessGreen);    
  
    // change the brightness for next time through the loop:
    brightnessGreen = brightnessGreen + fadeGreenAmount;
  
    // reverse the direction of the fading at the ends of the fade: 
    if (brightnessGreen <= minGreenBrightness || brightnessGreen >= maxGreenBrightness) {
      fadeGreenAmount = -fadeGreenAmount ; 
    }
    lastGreenBlink = now;
  }
}
