//////////////////////////////////////////////////////////////////
//©2017 Graeme Jury ZL2APV
//Released under the lgpl License - Please alter and share.
//Using the Brian Schmaltz easy stepper with an arduino
//rotate() steps a specific number of steps.
//rotateDeg() steps a specific number of degrees
//speed is controlled by a number from .01 -> 1 (1 = fastest)
//Slower Speed == Stronger movement
//Written to control 32 step stepper (32*8 = 256 microsteps)
// Stepper used = 28BYJ-48 5 volt
// Gear ratio   = 1/64
//The capacitor rotates anticlockwise to traverse from maximum to
//minimum capacitance. We rotate clockwise to increase frequency.
/////////////////////////////////////////////////////////////////

#include <EEPROM.h>

#define DIR_PIN 2
#define STEP_PIN 3

#define NEMA17
//#define A28BYJ48

//const uint8_t pinA = 4;       // Connected to CLK on KY-040
//const uint8_t pinB = 5;       // Connected to DT on KY-040
//const uint8_t encoderBtn = 6; // Connected to Push Button on KY-040
// const uint8_t calBtn = 7;      // Connected to the calibrate button
const uint8_t endStopPin = 8;
const uint8_t maxCendstop = 10;
const uint8_t minCendstop = 12;

#define BTN1_PIN 7 // Connected to the calibrate button
#define BTN2_PIN 6 // Connected to Push Button on KY-040
#define BTN3_PIN 4 // Connected to CLK on KY-040
#define BTN4_PIN 5 // Connected to DT on KY-040

#ifdef A28BYJ48
// Change this to fit the number of steps per revolution for your motor.
const int stepsPerMotorRev = 32;
//
const int gearboxRatio     = 64;     // Assuming gear down ratio
const int stepsPerShaftRev = (stepsPerMotorRev * gearboxRatio);  // 2048 for 28BYJ-48 motor.
const int microStepsPerStep = 1;     // Change according to EasyDriver settings
#endif

#ifdef NEMA17
// Change this to fit the number of steps per revolution for your motor.
const int stepsPerMotorRev = 200;
//
const int gearboxRatio     = 27;     // Assuming gear down ratio
const int stepsPerShaftRev = (stepsPerMotorRev * gearboxRatio);  // 5400 for Nema_17 motor.
const int microStepsPerStep = 1;     // Change according to EasyDriver settings
#endif


#define pollTime 100
#define MASK1 0b1100000000011111  // ( C01F )
#define pressPattern   0b1100000000000000
#define releasePattern 0b0000000000011111

struct btns {
  uint16_t btn1_history = 0xFFFF;
#ifdef BTN2_PIN
  uint16_t btn2_history = 0xFFFF;
#endif
#ifdef BTN3_PIN
  uint16_t btn3_history = 0xFFFF;
#endif
#ifdef BTN4_PIN
  uint16_t btn4_history = 0xFFFF;
#endif
#ifdef FEATURE_DEBUG_BTNS
  int count = 0xFF;
#endif
} key_I; //Key state information

int encoderPosCount = 0;
int mode = -1; // -1 = initialize; 0 = normal tuning; 1 to 4 = calibration modes.

uint16_t interrupt2maxC = 0;
uint16_t backlash = 0;
uint16_t minimumC = stepsPerShaftRev / 2 * microStepsPerStep * 1; // Drive it right to the end
uint16_t maximumC = 0; // Position 0 is plates fully meshed

// As we are stepping anticlockwise if not already at the interrupter location or less, starting
// with current posn = 2700 prevents us from ever reaching maximum capacitance and trying to go
// less than currentPosn = 0. We treat being at interrupter or less separately
int currentPosn = 2700;
boolean bCW;
unsigned int eepromTermAddr = 0;

const uint8_t MASK = 0b11000111;

void setup() {
  // initialize the digital output pins.
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(maxCendstop, OUTPUT);
  pinMode(minCendstop, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(maxCendstop, LOW);
  digitalWrite(minCendstop, LOW);
  digitalWrite(LED_BUILTIN, LOW); // Turn off the LED
  // initialize the digital input pins.
  //  pinMode(pinA, INPUT); digitalWrite(pinA, HIGH); // Pullups are part of the encoder component
  //  pinMode(pinB, INPUT); digitalWrite(pinB, HIGH);
  //  pinMode(encoderBtn, INPUT); digitalWrite(encoderBtn, HIGH); // Enable pullup for this one
  //  pinMode(calBtn, INPUT); digitalWrite(calBtn, HIGH); // Enable pullup for this one
  pinMode(BTN1_PIN, INPUT_PULLUP); // calBtn
  pinMode(BTN2_PIN, INPUT_PULLUP); // Encoder pushbutton
  pinMode(BTN3_PIN, INPUT_PULLUP); // CLK on KY-040 (pin4)
  pinMode(BTN4_PIN, INPUT_PULLUP); // DT on KY-040 (pin 5)
  pinMode(endStopPin, INPUT); digitalWrite(endStopPin, HIGH);

  //  pinA_Last = digitalRead(pinA);
  //Initialize serial and wait for port to open:
  Serial.begin(112500);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  // Check to see if calibrated
  uint8_t eeAddress = 0;
  uint8_t magicNum;
  EEPROM.get(eeAddress, magicNum);
  if (magicNum == 0xAA) {
    eeAddress += sizeof(uint8_t);
    EEPROM.get(eeAddress, interrupt2maxC);
    eeAddress += sizeof(interrupt2maxC);
    EEPROM.get(eeAddress, backlash);
    eeAddress += sizeof(backlash);
    EEPROM.get(eeAddress, minimumC);
    eeAddress += sizeof(minimumC);
    EEPROM.get(eeAddress, maximumC);
    eeAddress += sizeof(maximumC);
    Serial.println (F("\n-------------------\n  EEPROM VALUES"));
    Serial.print (F("magicNum = "));
    Serial.println (magicNum);
    Serial.print (F("interrupt2maxC = "));
    Serial.println (interrupt2maxC);
    Serial.print (F("backlash = "));
    Serial.println (backlash);
    Serial.print (F("minimumC = "));
    Serial.println (minimumC);
    Serial.print (F("maximumC = "));
    Serial.println (maximumC);
    Serial.println (F("-------------------"));

  } else { // No valid calibration data so flash the LED's to warn of non-calibration
    for (int x = 0; x < 10; x++) {
      digitalWrite(maxCendstop, HIGH);
      digitalWrite(minCendstop, HIGH);
      digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED's
      delay(500);
      digitalWrite(maxCendstop, LOW);
      digitalWrite(minCendstop, LOW);
      digitalWrite(LED_BUILTIN, LOW); // Turn off the LED's
      delay(500);
    }
  }


  setPosition(); // set the zero position
}

void loop() {
  static uint16_t KY_040_pBtn_Hist = 0;
  static uint16_t Cal_button_history = 0xFFFF;
  static uint16_t KY_040_DT_Hist = 0xFFFF;
  static uint16_t KY_040_CLK_Hist = 0xFFFF;
  static uint8_t Rotary_Encoder_history = 0;
  static boolean ledState = false;

  boolean b = digitalRead(BTN4_PIN);

  updateButton1();

  // *** Process Rotary Encoder ***
  /*
    if (key_I.btn3_history == 0x0000) {
      Serial.print("key_I.btn3_history = ");Serial.print(key_I.btn3_history);
      Serial.print(" & KY_040_CLK_Hist = ");Serial.println(KY_040_CLK_Hist);
      Serial.print("key_I.btn4_history = ");Serial.print(key_I.btn4_history);
      Serial.print(" & KY_040_DT_Hist = ");Serial.println(KY_040_DT_Hist); Serial.println();
      delay(1000);
    }

    if (key_I.btn3_history == 0xFFFF) {
      Serial.print("key_I.btn3_history = ");Serial.print(key_I.btn3_history);
      Serial.print(" & KY_040_CLK_Hist = ");Serial.println(KY_040_CLK_Hist);
      Serial.print("key_I.btn4_history = ");Serial.print(key_I.btn4_history);
      Serial.print(" & KY_040_DT_Hist = ");Serial.println(KY_040_DT_Hist); Serial.println();
      delay(1000);
    }
  */
  if ((key_I.btn3_history == 0x0000) && (key_I.btn3_history != KY_040_CLK_Hist)) { // HIGH to LOW transition
    KY_040_CLK_Hist = key_I.btn3_history;
    Serial.println("HIGH to LOW transition");
    // if the knob is rotating, we need to determine direction We do that by reading pin B state
    // and comparing to pinA's (both pins are equal when encoder is stationary).
    // We know pinA has gone from 1 -> 0 so see if pinB is also 0 yet
//    if ((digitalRead(BTN4_PIN) == HIGH)) {

    if(key_I.btn4_history == 0xFFFF) {
      // Means pin A Changed first - We're Rotating Clockwise
      encoderPosCount ++;
      bCW = true;
      if (ledState) {
        rotate(-5, .1);
      } else {
        rotate(-1, .1);
      }
    } else {
      // Otherwise B changed first and we're moving CCW
      encoderPosCount --;
      bCW = false;
      if (ledState) {
        rotate(5, .1);
      } else {
        rotate(1, .1);
      }
    }
    Serial.print ("1 Encoder position count = ");
    Serial.print(encoderPosCount);
    Serial.print (" and Current position = ");
    Serial.println(currentPosn);
  }
  
  if ((key_I.btn3_history == 0xFFFF) && (key_I.btn3_history != KY_040_CLK_Hist)) { // LOW to HIGH transition
    KY_040_CLK_Hist = key_I.btn3_history;
    Serial.println("LOW to HIGH transition");
        if(key_I.btn4_history == 0x0000) {
      // Means pin A Changed first - We're Rotating Clockwise
      encoderPosCount ++;
      bCW = true;
      if (ledState) {
        rotate(-5, .1);
      } else {
        rotate(-1, .1);
      }
    } else {
      // Otherwise B changed first and we're moving CCW
      encoderPosCount --;
      bCW = false;
      if (ledState) {
        rotate(5, .1);
      } else {
        rotate(1, .1);
      }
    }
    Serial.print ("1 Encoder position count = ");
    Serial.print(encoderPosCount);
    Serial.print (" and Current position = ");
    Serial.println(currentPosn);
  }

  // *** Process Calibrate button ***
  if ((key_I.btn1_history == 0x0000) && (key_I.btn1_history != Cal_button_history)) { // Button has changed state
    Cal_button_history = 0x0000;
    Serial.println ("Calibrate button press detected");
    calibrate();
  }
  if ((key_I.btn1_history == 0xFFFF) && (key_I.btn1_history != Cal_button_history)) { // Button has gone to released
    Cal_button_history = 0xFFFF;
  }

  // *** Process Rotary Encoder Pushbutton ***
  if ((key_I.btn2_history == 0x0000) && (key_I.btn2_history != KY_040_pBtn_Hist)) { // Button was pressed
    KY_040_pBtn_Hist = 0x0000;
    Serial.print ("key_I.btn1_history = ");
    Serial.print(key_I.btn1_history);
    Serial.print (" and KY_040_CLK_Hist = ");
    Serial.println(KY_040_CLK_Hist);
    // Toggle LED
    if (ledState) {
      digitalWrite(LED_BUILTIN, LOW); // Turn off LED
      ledState = false;
    } else {
      digitalWrite(LED_BUILTIN, HIGH);   // Turn on LED
      ledState = true;
    }
  }
  if ((key_I.btn2_history == 0xFFFF) && (key_I.btn2_history != KY_040_pBtn_Hist)) { // Button was released
    KY_040_pBtn_Hist = 0xFFFF;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutines start here
////////////////////////////////////////////////////////////////////////////////////////////////////////////

void calibrate()
// Calibrate the zero position
{
  mode++;
  int endStatus = 0;

  int clockwise[10];
  int anticlock[10];

  uint16_t counter = 0;
  uint16_t Aclock = 0;
  uint16_t clockW = 0;

  switch (mode) {
    case 1:
      {
        if (digitalRead(endStopPin)) { //High when interrupted
          rotate(-500, .1); // Step clockwise
        }
        //        stepFromEndstop();
        Serial.print (F("case 1, currentPosn start = "));
        Serial.print (currentPosn);
        Serial.print (F("; endStopPin = "));
        Serial.println (digitalRead(endStopPin));
        while (!endStatus) {
          rotate(1, .1); // Step clockwise
          counter++;
          endStatus = digitalRead(endStopPin);
          //    delay(1); // Add delay here to slow down switch rotation speed
        }
        currentPosn = 0;
        Serial.print (F("case 1, currentPosnA = "));
        Serial.print (currentPosn);
        Serial.print (F(":  counterA = "));
        Serial.println(counter);
        break;
      }
    case 2:
      {
        // We are stationary it the interrupt stop so count how many steps in the reverse direction
        // are required to take up the backlash and step off the interrupter then check how many
        // steps it takes to come back onto the interrupter. Average this over 5 steps
        counter = 0;
        endStatus = digitalRead(endStopPin);
        for (int x = 0; x < 10; x++) {
          if (endStatus) digitalWrite(LED_BUILTIN, HIGH);
          while (endStatus) {
            rotate(-1, .01); // Step anticlockwise slowly
            counter++;
            endStatus = digitalRead(endStopPin);
          }

          rotate(-1, .01); // Take an extra step to be sure we have activated the interrupter
          counter++;

          digitalWrite(LED_BUILTIN, LOW);
          anticlock[x] = counter - 1; // anticlock holds steps out of interrupter

          Serial.print (F("case 2, anticlock backlash = "));
          Serial.println (counter);
          counter = 0;

          delay(100); // Wait for things to mechanically settle

          while (!endStatus) {
            rotate(1, .01); // Step clockwise into the interrupter
            counter++;
            endStatus = digitalRead(endStopPin);
          }
          rotate(1, .01); // Take an extra step to be sure we have activated the interrupter
          counter++;
          clockwise[x] = counter - 1;

          delay(100); // Wait for things to mechanically settle

          Serial.print (F("case 2, clockwise backlash = "));
          Serial.println (counter); // Counter holds steps to go back
          counter = 0;
        }
        for (int x = 0; x < 10; x++) {
          Aclock += anticlock[x];
          Serial.print (F("anticlock[x] = "));
          Serial.print (anticlock[x]); Serial.print("; "); Serial.println(Aclock);
          clockW += clockwise[x];
          Serial.print (F("clockwise[x] = "));
          Serial.print (clockwise[x]); Serial.print("; "); Serial.println(clockW);
        }
        backlash = (Aclock + clockW) / 10 / 2;
        Serial.println (backlash);
        counter = 0;
        currentPosn = 0;
        Serial.println (F("\nPLEASE MANUALLY TUNE TO MAX C AND PRESS CAL BUTTON\n"));
        break;
      }
    case 3:
      {
        interrupt2maxC = abs(currentPosn);
        Serial.print (F("case 3, interrupt2maxC = "));
        Serial.println (interrupt2maxC);
        counter = 0;
        currentPosn = 0;
        Serial.println (F("\nPLEASE MANUALLY TUNE TO MIN C AND PRESS CAL BUTTON\n"));
        break;
      }
    case 4:
      {
        Serial.print (F("case 4, currentPosnA = "));
        Serial.println (currentPosn);
        mode = 0;

        Serial.print (F("case 4, interrupt2maxC = "));
        Serial.println (interrupt2maxC);
        Serial.print (F("case 4, backlash = "));
        Serial.println (backlash);
        Serial.print (F("case 4, minimumC = "));
        Serial.println (minimumC);
        Serial.print (F("case 4, maximumC = "));
        Serial.println (maximumC);

        // Store the calibration values
        int  eeAddress = 0;
        uint8_t magicNum = 0xAA;
        EEPROM.put(eeAddress, magicNum); //The magic number that shows valid info in eeprom
        eeAddress += sizeof(uint8_t);
        EEPROM.put(eeAddress, interrupt2maxC);
        eeAddress += sizeof(interrupt2maxC);
        EEPROM.put(eeAddress, backlash);
        eeAddress += sizeof(backlash);
        EEPROM.put(eeAddress, minimumC);
        eeAddress += sizeof(minimumC);
        EEPROM.put(eeAddress, maximumC);
        eeAddress += sizeof(maximumC);

        break;
      }
  } // End of "switch(mode)"

}

/**********************************************************************************************************/

void stepFromEndstop()
//  Check if we are sitting with the end indicator operated i.e. at max C or close to it and step towards
// minimum C enough to be clear of backlash for calibrating or initial setting.
{
  if (digitalRead(maxCendstop)) {
    rotate(-500, .1); // Step clockwise
  }
}

/**********************************************************************************************************/

void update_button(uint8_t *button_history, int pinNum) {
  //  ProfileTimer t ("update_button");
  //  delay(2);
  *button_history = *button_history << 1;
  *button_history |= (digitalRead(pinNum) == 0);  // Normally pulled up so goes to 0 with button press
}
/**********************************************************************************************************/

uint8_t is_button_pressed(uint8_t *button_history)
// Returns true if debounced transition from HIGH to LOW
{
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

void setPosition()
// Called by setup

// There are two possible states here on powerup. (1) the capacitor position is such that we are activating
// the interrupter or (2) we are stepped beyond the interrupter. In both cases we need to be stepped beyond
// the interrupter and step back to it where we mark the position using this as a reference from the
// interrupt2maxC calibration data stored in EEprom.

// (1) We will rotate 500 steps anticlockwise to step out of the interrupter, counting the steps. We then
// rotate clockwise back to the interrupter and set the position reference then continue counting back the
// rest of the 500 steps to restore the original position

// (2) We are rotating clockwise (facing the capacitor shaft) to increase the capacity until it reaches the
// interrupter. We count each step taken to do this. When the interrupter is reached the current position is set
// to interrupt2maxC and we command the stepper to step anticlockwise the number of steps counted to restore
// the original position.
{
  int endStatus = digitalRead(endStopPin); //High when interrupted
  uint16_t counter = 0;

  Serial.print ("Counter initial value = ");
  Serial.println (counter);

  // Test to see if we have operated the interrupter

  if (endStatus) { // At interrupter so step beyond and back. (High when interrupted)
    mode = -1;
    rotate(-1, .01); // Set the backlash to be at a constant position
    delay(500);
    rotate(1, .01);
    delay(500);
    rotate(-500, .1); // Step anticlockwise out of the interrupter
    counter = 500; // set counter to match steps taken
    Serial.print ("Counter maximum value = ");
    Serial.println (counter);
    endStatus = digitalRead(endStopPin);
    while (!endStatus) {
      rotate(1, .1); // Step clockwise
      counter--;
      //      Serial.print (F("Counter value = ")); Serial.println (counter);
      endStatus = digitalRead(endStopPin);
    }
    currentPosn = interrupt2maxC; // Set our reference value at interrupter change to high
    Serial.print (F("currentPosn = "));
    Serial.print (currentPosn);
    Serial.print (F("; -- Counter value at reference = "));
    Serial.println (counter);
    while (counter) {
      rotate(1, .1); // Step clockwise to original position
      counter--; // currentPosn is adjusted in the rotate routine
    }
  } else { // We were not at interrupter
    mode = -2;
    while (!endStatus) {
      rotate(1, .1); // Step clockwise
      counter++;
      endStatus = digitalRead(endStopPin);
    }
    currentPosn = interrupt2maxC;
    Serial.print (F("currentPosn = "));
    Serial.print (currentPosn);
    Serial.print (F("; -- Counter value at reference = "));
    Serial.println (counter);
    while (counter) {
      rotate(-1, .1); // Step anticlockwise to original position
      counter--; // currentPosn is adjusted in the rotate routine
    }
  }
  mode = 0; // Set to normal stepper operation
  Serial.print (F("\ncurrentPosn = "));
  Serial.print (currentPosn);
  Serial.print (F(":  counter = "));
  Serial.println(counter);
}
/**********************************************************************************************************/
void rotate(int steps, float speed) {
  // Rotate a specific number of microsteps (8 microsteps per step) - (negitive for reverse movement)
  // speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  // This routine checks for a change of direction and applies the backlash correction.

  static int oldRotationDirection = HIGH;
  int rotationDirection = (steps > 0) ? HIGH : LOW;
  boolean y = false;
  float usDelay = (1 / speed) * 140;

  if (mode == -1) oldRotationDirection = LOW;
  steps = abs(steps); // Convert to positive number if negative
  if (mode == 0) {
    if (rotationDirection != oldRotationDirection) {
      steps += backlash;
      oldRotationDirection = rotationDirection;
    }
  }
  digitalWrite(DIR_PIN, rotationDirection); // Set the rotation direction on the Easy Stepper

  for (int i = 0; i < steps; i++) {
    // If not in calibration mode, detect the position light interrupter status
    if (mode == 0) {
      y = digitalRead(endStopPin);
    }
    //
    if ((currentPosn <= maximumC) && rotationDirection) { // maximumC is at currentPosn = 0
      digitalWrite(maxCendstop, HIGH);
      if (!mode) { // Don't honour endstops if in "Calibrate mode"
        break; // Only let it step clockwise if at end stop
      }
    } else {
      digitalWrite(maxCendstop, LOW);
    }
    if ((currentPosn >= minimumC) && !rotationDirection) { // minimumC is at currentPosn = 2700
      digitalWrite(minCendstop, HIGH);
      if (!mode) { // Don't honour endstops if in "Calibrate mode"
        break; // Only let it step anticlockwise if at maximum
      }
    } else {
      digitalWrite(minCendstop, LOW);
    }

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
    if (rotationDirection) {
      currentPosn--;
    } else {
      currentPosn++;
    }
  } // End of for loop
}
/**********************************************************************************************************/
void updateButton1(void) {

  // Get the current button state.

  static uint32_t startTime = micros() - pollTime; //Force immediate execution 1st time around

  if ((startTime + pollTime) > micros()) {
    return false; // We hadn't reached timeout so didn't process a button
  }

  startTime = micros();

#ifdef FEATURE_DEBUG_BTNS
  if (key_I.count == 0) {
    Serial.print(key_I.count); Serial.print("\t");
    Serial.println(key_I.btn1_history, BIN);
    key_I.count++;
  }
#endif

  key_I.btn1_history = (key_I.btn1_history << 1) | digitalRead(BTN1_PIN);

#ifdef FEATURE_DEBUG_BTNS
  if ((key_I.count <= 25) && (key_I.count != 0xFF)) {
    Serial.print(key_I.count); Serial.print("\t");
    Serial.println(key_I.btn1_history, BIN);
    key_I.count++;
  } else {
    if (key_I.count != 0xFF) Serial.println("-----------------------------------------");
    key_I.count = 0xFF;
  }
#endif
  if ((key_I.btn1_history & MASK1) == releasePattern) {
    key_I.btn1_history = 0x5555;
#ifdef FEATURE_DEBUG_BTNS
    key_I.count = 0;
    Serial.println("Released");
#endif
  }
  if ((key_I.btn1_history & MASK1) == pressPattern) {
    //    pressed = true;
    key_I.btn1_history = 0xAAAA;
#ifdef FEATURE_DEBUG_BTNS
    key_I.count = 0;
    Serial.println("Pressed");
#endif
  }

#ifdef BTN2_PIN
  key_I.btn2_history = (key_I.btn2_history << 1) | digitalRead(BTN2_PIN);
  if ((key_I.btn2_history & MASK1) == releasePattern) {
    key_I.btn2_history = 0x5555;
  }
  if ((key_I.btn2_history & MASK1) == pressPattern) {
    key_I.btn2_history = 0xAAAA;
  }
#endif
#ifdef BTN3_PIN
  key_I.btn3_history = (key_I.btn3_history << 1) | digitalRead(BTN3_PIN);
  if ((key_I.btn3_history & MASK1) == releasePattern) {
    key_I.btn3_history = 0x5555;
  }
  if ((key_I.btn3_history & MASK1) == pressPattern) {
    key_I.btn3_history = 0xAAAA;
  }
#endif
#ifdef BTN4_PIN
  key_I.btn4_history = (key_I.btn4_history << 1) | digitalRead(BTN4_PIN);
  if ((key_I.btn4_history & MASK1) == releasePattern) {
    key_I.btn4_history = 0x5555;
  }
  if ((key_I.btn4_history & MASK1) == pressPattern) {
    key_I.btn4_history = 0xAAAA;
  }
#endif
}
