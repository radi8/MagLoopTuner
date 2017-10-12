//////////////////////////////////////////////////////////////////
//©2017 Graeme Jury ZL2APV
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
const uint8_t pinA = 4;    // Connected to CLK on KY-040
int pinB = 5;    // Connected to DT on KY-040
int pinBtn = 6;  // Connected to Push Button on KY-040
int encoderPosCount = 0;
boolean bCW;

#define MASK   0b11000111

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
  static uint8_t Push_button_history = 0;
  static uint8_t Rotary_Encoder_history = 0;
  static boolean ledState = false;

//  update_button(&Rotary_Encoder_history, pinA);
  update_button(&Rotary_Encoder_history, PIND, PD3);
  if (is_button_pressed(&Rotary_Encoder_history)) {
    // if the knob is rotating, we need to determine direction We do that by reading pin B state
    // and comparing to pinA's (both pins are equal when encoder is stationary).
    // We know pinA has gone from 1->0 so see if pinB is also 0
    if ((digitalRead(pinB) == LOW)){
      // pinB is already low so we are moving anticlockwise
      encoderPosCount --;
      bCW = false;
    }else{
      //  Going clockwise
      encoderPosCount ++;
      bCW = true;
    }
    Serial.print ("Encoder position count = ");
    Serial.println(encoderPosCount);    
/***********************************************************************************************/
  //rotate a specific number of microsteps (8 microsteps per step)
  //a 32 step stepper would take 32 * 8 * 64 = 16384 micro steps for one full shaft revolution
  Serial.println("Rotating 360 deg. at 0.5");
  rotate(16384, .5); 
  delay(1000); 

  Serial.println("Rotating -360 deg. at 0.01");
  rotate(-16384, .05); //reverse
  delay(1000);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutines start here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void update_button(uint8_t *button_history,int portNum, int pinNum) {
  *button_history = *button_history << 1;
//  *button_history |= (digitalRead(pinNum) == 0);
  *button_history |= ((portNum & (1<<pinNum)) == 0); // Normally pulled up so goes to 0 with button press
}
/**********************************************************************************************************/
uint8_t is_button_pressed(uint8_t *button_history) {
  uint8_t pressed = 0;
  if ((*button_history & MASK) == 0b00000111) {
    pressed = 1;
    *button_history = 0b11111111;
  }
  return pressed;
}
/**********************************************************************************************************/
uint8_t is_button_released(uint8_t *button_history) {
  uint8_t released = 0;
  if ((*button_history & MASK) == 0b11000000) { // mask_bits removed from here
    released = 1;
    *button_history = 0b00000000;
  }
  return released;
}
/**********************************************************************************************************/
/*
uint8_t is_button_down(uint8_t *button_history) {
  return (*button_history == 0b11111111);
}
uint8_t is_button_up(uint8_t *button_history) {
  return (*button_history == 0b00000000);
}
*/

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
/**********************************************************************************************************/
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
/**********************************************************************************************************/
