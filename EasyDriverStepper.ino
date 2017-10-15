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

#define DIR_PIN 2
#define STEP_PIN 3
const uint8_t pinA = 4;    // Connected to CLK on KY-040
const uint8_t pinB = 5;    // Connected to DT on KY-040
const uint8_t pinBtn = 6;  // Connected to Push Button on KY-040
const uint8_t endStopPin = 8;

const int stepsPerMotorRev = 32;    // Change this to fit the number of steps per revolution
// for your motor.
const int gearboxRatio     = 64;     // Assuming gear down ratio
const int stepsPerShaftRev = (stepsPerMotorRev * gearboxRatio);  // 2048 for 28BYJ-48 motor.
const int microStepsPerStep = 1;     // Change according to EasyDriver settings

int encoderPosCount = 0;

int maxPosn = stepsPerShaftRev/2 * microStepsPerStep * 0.998;// Don't drive it right to the end
int currentPosn = (maxPosn - 1);
boolean bCW;
//boolean pinA_Last;

const uint8_t MASK = 0b11000111;

void setup() {
  // initialize the digital output pins.
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  // initialize the digital input pins.
  pinMode(pinA, INPUT); digitalWrite(pinA, HIGH); // Pullups are part of the encoder component
  pinMode(pinB, INPUT); digitalWrite(pinB, HIGH);
  pinMode(pinBtn, INPUT); digitalWrite(pinBtn, HIGH); // Enable pullup for this one
  pinMode(endStopPin, INPUT); digitalWrite(endStopPin, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Turn off the LED
  //  pinA_Last = digitalRead(pinA);
  //Initialize serial and wait for port to open:
  Serial.begin(112500);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  calibrate(); // Calibrate the zero position
/*
  // Calibrate the zero position
  int endStatus = digitalRead(endStopPin); //High when interrupted
  uint16_t targetPosn = 0;
  Serial.print ("currentPosn start = ");
  Serial.println (currentPosn); 
  while (!endStatus) {
    rotate(-1, .1);
    targetPosn++;
    endStatus = digitalRead(endStopPin);
    //    delay(1); // Add delay here to slow down switch rotation speed
  }
  currentPosn = 0;
  Serial.print ("currentPosnA = ");
  Serial.print (currentPosn);
  Serial.print (":  targetPosnA = ");
  Serial.println(targetPosn);
  if (targetPosn != 0) { // Settings are OK if already at zero position
    rotate(targetPosn, .1);
  }
  Serial.print ("currentPosn = ");
  Serial.print (currentPosn);
  Serial.print (":  targetPosn = ");
  Serial.println(targetPosn);
*/
}

void loop() {
  static uint8_t Push_button_history = 0;
  static uint8_t Rotary_Encoder_history = 0;
  static boolean ledState = false;

  boolean b = digitalRead(pinB);
  update_button(&Rotary_Encoder_history, pinA);

  if (is_button_pressed(&Rotary_Encoder_history)) {
    // if the knob is rotating, we need to determine direction We do that by reading pin B state
    // and comparing to pinA's (both pins are equal when encoder is stationary).
    // We know pinA has gone from 1 -> 0 so see if pinB is also 0 yet
    if ((digitalRead(pinB) == HIGH)) {
      //   if (b == HIGH) {
//      Serial.print(digitalRead(pinA)); Serial.print(" <A .... B> "); Serial.println(b);
      // Means pin A Changed first - We're Rotating Clockwise
      encoderPosCount ++;
      bCW = true;
      if(ledState){
        rotate(5, .1);
      } else {
        rotate(1, .1);
      }
    } else {
//      Serial.print(digitalRead(pinA)); Serial.print(" <A else B> "); Serial.println(b);
      // Otherwise B changed first and we're moving CCW
      encoderPosCount --;
      bCW = false;
      if(ledState){
        rotate(-5, .1);
      } else {
        rotate(-1, .1);
      }
    }
    //    pinA_Last = !pinA_Last;
    Serial.print ("Encoder position count = ");
    Serial.print(encoderPosCount);
    Serial.print (" and Current position = ");
    Serial.println(currentPosn);
  }

  if (is_button_released(&Rotary_Encoder_history)) {
    // if the knob is rotating, we need to determine direction We do that by reading pin B state
    // and comparing to pinA's (both pins are equal when encoder is stationary).
    // We know pinA has gone from 1-> 0 so see if pinB is also 0
    if ((digitalRead(pinB) == LOW)) {
      //    if (b == LOW) {
//      Serial.print(digitalRead(pinA)); Serial.print(" <A release B> "); Serial.println(b);
      // pinB is already low so we are moving clockwise
      encoderPosCount ++;
      bCW = true;
      if(ledState){
        rotate(5, .1);
      } else {
        rotate(1, .1);
      }
    } else {
//      Serial.print(digitalRead(pinA)); Serial.print(" <A release else B> "); Serial.println(b);
      //  Going anticlockwise
      encoderPosCount --;
      bCW = false;
      if(ledState){
        rotate(-5, .1);
      } else {
        rotate(-1, .1);
      }
    }
    Serial.print ("Encoder position count = ");
    Serial.print(encoderPosCount);
    Serial.print (" and Current position = ");
    Serial.println(currentPosn);
  }

  update_button(&Push_button_history, pinBtn);
  if (is_button_pressed(&Push_button_history)) {
    Serial.println ("Button press detected");
    // Toggle LED
    if (ledState) {
      digitalWrite(LED_BUILTIN, LOW); // Turn off LED
      ledState = false;
    } else {
      digitalWrite(LED_BUILTIN, HIGH);   // Turn on LED
      ledState = true;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutines start here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void update_button(uint8_t *button_history, int pinNum) {
  //  ProfileTimer t ("update_button");
  //  delay(2);
  *button_history = *button_history << 1;
  *button_history |= (digitalRead(pinNum) == 0);  // Normally pulled up so goes to 0 with button press
}
/**********************************************************************************************************/
// Returns true if debounced transition from HIGH to LOW
uint8_t is_button_pressed(uint8_t *button_history) {
  //  ProfileTimer t ("is_button_pressed");
  uint8_t pressed = 0;
  if ((*button_history & MASK) == 0b00000111) {
    pressed = 1;
    *button_history = 0b11111111;
  }
  return pressed;
}
/**********************************************************************************************************/

uint8_t is_button_released(uint8_t *button_history) {
  //  ProfileTimer t ("is_button_released");
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
/**********************************************************************************************************/
void calibrate()
{
  // Calibrate the zero position
  int endStatus = digitalRead(endStopPin); //High when interrupted
  uint16_t targetPosn = 0;
  Serial.print ("currentPosn start = ");
  Serial.println (currentPosn); 
  while (!endStatus) {
    rotate(-1, .1);
    targetPosn++;
    endStatus = digitalRead(endStopPin);
    //    delay(1); // Add delay here to slow down switch rotation speed
  }
  currentPosn = 0;
  Serial.print ("currentPosnA = ");
  Serial.print (currentPosn);
  Serial.print (":  targetPosnA = ");
  Serial.println(targetPosn);
  if (targetPosn != 0) { // Settings are OK if already at zero position
    rotate(targetPosn, .1);
  }
  Serial.print ("currentPosn = ");
  Serial.print (currentPosn);
  Serial.print (":  targetPosn = ");
  Serial.println(targetPosn);
}


void rotate(int steps, float speed) {
  //rotate a specific number of microsteps (8 microsteps per step) - (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (steps > 0) ? HIGH : LOW;
  steps = abs(steps);

//  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on while rotating
  digitalWrite(DIR_PIN, dir);

  float usDelay = (1 / speed) * 140;

  for (int i = 0; i < steps; i++) {
    boolean y = digitalRead(endStopPin);
    
    if(y && !dir) break; // Only let it step clockwise if at end stop
    if((currentPosn >= maxPosn) && dir)break; // Only let it step anticlockwise if at maximum
    
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
    if(dir){
      currentPosn++;
    } else {
      currentPosn--;
    }
  }
//  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off now rotation finished
}
/**********************************************************************************************************/
void rotateDeg(float deg, float speed) {
  //rotate a specific number of degrees (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (deg > 0) ? HIGH : LOW;
//  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on while rotating
  digitalWrite(DIR_PIN, dir);

  int steps = abs(deg) * (1 / 0.9375); // 360/384=.9375
  float usDelay = (1 / speed) * 140;

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
  }
//  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off now rotation finished
}
/**********************************************************************************************************/
