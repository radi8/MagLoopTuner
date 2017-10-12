//////////////////////////////////////////////////////////////////
//©2014 Graeme Jury ZL2APV
//Released under the lgpl License - Please alter and share.
//Using the Btran Schmaltz easy stepper with an arduino
//rotate() steps a specific number of steps.
//rotateDeg() steps a specific number of degrees
//speed is controlled by a number from .01 -> 1 (1 = fastest) 
//Slower Speed == Stronger movement
//Written to control 32 step stepper (32*8 = 256 microsteps)
// Stepper used = 28BYJ-48 5 volt
// Gear ratio   = 1/64
/////////////////////////////////////////////////////////////////

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

#define DIR_PIN 2
#define STEP_PIN 3

//#include <Stepper.h>

const int stepsPerMotorRev = 32;    // Change this to fit the number of steps per revolution
                                    // for your motor.
const int gearboxRatio     = 64;     // Assuming gear down ratio
const int stepsPerShaftRev = (stepsPerMotorRev * gearboxRatio);  // 2048 for 28BYJ-48 motor.
const int microStepsPerStep = 8;     // Change according to EasyDriver settings

                         
void setup() {
// initialize the digital pins as outputs.
  pinMode(led, OUTPUT);
  pinMode(DIR_PIN, OUTPUT); 
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(led, LOW);   // turn the LED off for initial state
  //Initialize serial and wait for port to open:
  Serial.begin(9600); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

}

void loop() {
/*  
  Serial.println("Starting series");
  //rotate a specific number of degrees 
  Serial.println("Rotating 360 deg. at 0.1");
//  rotateDeg(360, .1);
  delay(1000);

  Serial.println("Rotating -360 deg. at 0.01");
//  rotateDeg(-180, .01);  //reverse
  delay(1000); 
*/

  //rotate a specific number of microsteps (8 microsteps per step)
  //a 32 step stepper would take 32 * 8 * 64 = 16384 micro steps for one full shaft revolution
  Serial.println("Rotating 360 deg. at 0.5");
  rotate(16384, .5); 
  delay(1000); 

  Serial.println("Rotating -360 deg. at 0.01");
  rotate(-16384, .05); //reverse
  delay(1000);

}

void rotate(int steps, float speed){ 
  //rotate a specific number of microsteps (8 microsteps per step) - (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (steps > 0)? HIGH:LOW;
  steps = abs(steps);
  digitalWrite(led, HIGH);   // turn the LED on while rotating
  digitalWrite(DIR_PIN,dir); 

  float usDelay = (1/speed) * 140;

  for(int i=0; i < steps; i++){ 
    digitalWrite(STEP_PIN, HIGH); 
    delayMicroseconds(usDelay); 

    digitalWrite(STEP_PIN, LOW); 
    delayMicroseconds(usDelay); 
  } 
  digitalWrite(led, LOW);   // turn the LED off now rotation finished
} 

void rotateDeg(float deg, float speed){ 
  //rotate a specific number of degrees (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (deg > 0)? HIGH:LOW;
  digitalWrite(led, HIGH);   // turn the LED on while rotating
  digitalWrite(DIR_PIN,dir); 

  int steps = abs(deg)*(1/0.9375);// 360/384=.9375
  float usDelay = (1/speed) * 140;

  for(int i=0; i < steps; i++){ 
    digitalWrite(STEP_PIN, HIGH); 
    delayMicroseconds(usDelay); 

    digitalWrite(STEP_PIN, LOW); 
    delayMicroseconds(usDelay); 
  } 
  digitalWrite(led, LOW);   // turn the LED off now rotation finished
}
