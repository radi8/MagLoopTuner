//////////////////////////////////////////////////////////////////
// ©2017 Graeme Jury ZL2APV
// Released under the lgpl License - Please alter and share.
// Using the Brian Schmaltz easy stepper with an arduino
// rotate() steps a specific number of steps.
// speed is controlled by a number from .01 -> 1 (1 = fastest)
// Slower Speed == Stronger movement
// The capacitor rotates anticlockwise to traverse from maximum to
// minimum capacitance. We rotate clockwise to increase frequency.
/////////////////////////////////////////////////////////////////

#include <EEPROM.h>
#include <U8x8lib.h>
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

// Choose the stepper motor type (Only one)
#define NEMA17
//#define A28BYJ48

// Arduino pin assignments
const uint8_t DIR_PIN     = 2;  // Connected to the EasyDriver board
const uint8_t STEP_PIN    = 3;  // Connected to the EasyDriver board
// These assignments need to be #defines as they are referenced in the struct.
#define BTN3_PIN 4              // Connected to CLK on KY-040
#define BTN4_PIN 5              // Connected to DT on KY-040
#define BTN2_PIN 6              // Connected to Push Button on KY-040
#define BTN1_PIN 7              // Connected to the calibrate button
const uint8_t endStopPin  = 8;  // Connected to the interrupter
const uint8_t enablePin  = 9;  // Connected to the !ENABLE Pin on the EasyDriver board
const uint8_t maxCendstop = 10; // Connected to LED indicator
const uint8_t minCendstop = 12; // Connected to LED indicator

// Set stepper motor properties

#ifdef A28BYJ48 // 28BYJ-48 5 volt
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

// Program GLOBAL debounce definitions
#define pollTime 100
#define MASK1 0b1100000000011111  // ( C01F )
#define pressPattern   0b1100000000000000
#define releasePattern 0b0000000000011111

// Structure for holding the debounce history
struct btns {
  uint16_t btn1_State = 0xFFFF;
  uint16_t Cal_button_history = 0xFFFF;
#ifdef BTN2_PIN
  uint16_t btn2_State = 0xFFFF;
  uint16_t KY_040_pBtn_Hist = 0;
#endif
#ifdef BTN3_PIN
  uint16_t btn3_State = 0xFFFF;
  uint16_t KY_040_CLK_Hist = 0xFFFF;
#endif
#ifdef BTN4_PIN
  uint16_t btn4_State = 0xFFFF;
  uint16_t KY_040_DT_Hist = 0xFFFF;
#endif
#ifdef FEATURE_DEBUG_BTNS
  int count = 0xFF;
#endif
} KI; //Key state information

#define MENU_ITEMS 4
const char *menu_main[MENU_ITEMS] = { " *** STATUS *** ", "Restoring Last C", "Tune Rate     X1", "Current Band 20M" };
const char *menu_top[MENU_ITEMS] = { "Get Band", "Set Band", "Calibrate", "EXIT" };
const char *menu_band[MENU_ITEMS] = { "Band = 30M", "Band = 20M", "Band = 17M", "Band = 15M"};


int encoderPosCount = 0;
char encoderPosString[5]; // The count needs to be a string for the display
int mode = -1; // -1 = initialize; 0 = normal tuning; 1 to 4 = calibration modes.

// Stepper motor state variables loaded from EEprom at startup
uint16_t interrupt2maxC = 0;
uint16_t backlash = 0;
uint16_t minimumC = stepsPerShaftRev / 2 * microStepsPerStep * 1; // Drive it right to the end
uint16_t maximumC = 0; // Position 0 is plates fully meshed

// As we are stepping anticlockwise if not already at the interrupter location or less, starting
// with current posn = 2700 prevents us from ever reaching maximum capacitance and trying to go
// less than currentPosn = 0. We treat being at interrupter or less separately
int currentPosn = 2700;

unsigned int eepromTermAddr = 0;

void setup() {
  // initialize the digital output pins.
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(maxCendstop, OUTPUT);
  pinMode(minCendstop, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Turn off the LED
  digitalWrite(enablePin, HIGH); // Disable EasyDriver stepper driver stage
  digitalWrite(maxCendstop, LOW);
  digitalWrite(minCendstop, LOW);

  // initialize the digital input pins.
  pinMode(BTN1_PIN, INPUT_PULLUP); // calBtn
  pinMode(BTN2_PIN, INPUT_PULLUP); // Encoder pushbutton
  pinMode(BTN3_PIN, INPUT_PULLUP); // CLK on KY-040 (pin4)
  pinMode(BTN4_PIN, INPUT_PULLUP); // DT on KY-040 (pin 5)
  pinMode(endStopPin, INPUT_PULLUP);

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

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_7x14_1x2_r); // 4 rows of 16 chars (We stay with this font for all printing)
/*
  u8x8.drawString(0, 0, " *** STATUS *** ");
  u8x8.setInverseFont(1);
  u8x8.drawString(0, 2, "Restoring Last C");
  u8x8.setInverseFont(0);
  u8x8.drawString(0, 4, "Tune Rate     X1");
  //
  u8x8.drawString(0, 6, "Current Band 20M");
  //
  //  u8x8.drawString(0,6,"1234567890123456");
*/
  drawMenu(1, 0);
  setPosition(); // set the zero position
}

void loop()
{
  static uint32_t buttonTime = micros(); //Measure encoder pushbutton held time
  static uint8_t Rotary_Encoder_history = 0;
  static boolean ledState = true; // 1st time through loop LED will toggle to false
  static boolean tmp = true;
  static boolean menuMode = false;
  boolean bCW;

  updateButton(); // Get the current state of all the buttons

  // To avoid the default values for the KY_040 historys causing a false step from the
  // rotary encoder, they need to be adjusted to match the detent (Hi or Lo) values. This
  // is achieved by reading the CLK detent. Both CLK & DT have same output when stationary.
  if (tmp) { // Do this first time through the loop only
    if (digitalRead(BTN3_PIN) == 0) {
      KI.KY_040_CLK_Hist = 0;
      KI.KY_040_DT_Hist = 0;
    }
    tmp = false;
  }
  // *** Process Rotary Encoder ***

  // Test for a rotary encoder CLK HIGH to LOW transition
  if ((KI.btn3_State == 0x0000) && (KI.btn3_State != KI.KY_040_CLK_Hist)) {
    KI.KY_040_CLK_Hist = KI.btn3_State;
    Serial.println("HIGH to LOW transition");
    // if the knob is rotating, we need to determine direction We do that by reading pin B state
    // and comparing to pinA's (both pins are equal when encoder is stationary).
    // We know pinA has gone from 1 -> 0 so see if pinB is also 0 yet

    if (KI.btn4_State == 0xFFFF) {
      // Means pin A Changed first - We're Rotating Clockwise
      bCW = true;
    } else {
      bCW = false;
    }
    rotaryEncoderStep(bCW, ledState);
  }
  // Test for a rotary encoder CLK LOW to HIGH transition
  if ((KI.btn3_State == 0xFFFF) && (KI.btn3_State != KI.KY_040_CLK_Hist)) {
    KI.KY_040_CLK_Hist = KI.btn3_State;
    Serial.println("LOW to HIGH transition");
    if (KI.btn4_State == 0x0000) {
      // Means pin A Changed first - We're Rotating Clockwise
      bCW = true;
    } else {
      bCW = false;
    }
    rotaryEncoderStep(bCW, ledState);
  }

  // *** Process Calibrate button ***

  if ((KI.btn1_State == 0x0000) && (KI.btn1_State != KI.Cal_button_history)) { // Button has changed state
    KI.Cal_button_history = 0x0000;
    Serial.println ("Calibrate button press detected");
    calibrate();
  }
  if ((KI.btn1_State == 0xFFFF) && (KI.btn1_State != KI.Cal_button_history)) { // Button has gone to released
    KI.Cal_button_history = 0xFFFF;
  }

  // *** Process Rotary Encoder Pushbutton ***

  if ((KI.btn2_State == 0x0000) && (KI.btn2_State != KI.KY_040_pBtn_Hist)) { // Button was pressed
    KI.KY_040_pBtn_Hist = 0x0000;
    buttonTime = micros();
    Serial.print(F("KI.btn1_State = "));
    Serial.print(KI.btn1_State);
    Serial.print(F(" and KI.KY_040_CLK_Hist = "));
    Serial.println(KI.KY_040_CLK_Hist);

  }
  if ((KI.btn2_State == 0xFFFF) && (KI.btn2_State != KI.KY_040_pBtn_Hist)) { // Button was released
    KI.KY_040_pBtn_Hist = 0xFFFF;
    Serial.print("buttonTime + = "); Serial.print(buttonTime + 1000000);
    Serial.print(" & micros() = "); Serial.println(micros());
    if (menuMode) {
      loopMenu();
    } else {
      if ((buttonTime + 1000000) < micros()) { // check for a long press
        // We are going into menu mode
        Serial.println(F("We are going into menu mode"));
        menuMode = true;
      } else {
        // We are staying in stepper mode
        Serial.println(F("We are staying in stepper mode"));
        if (ledState) {
          digitalWrite(LED_BUILTIN, LOW); // Turn off LED
          ledState = false;
          u8x8.drawString(15, 4, "1");
        } else {
          digitalWrite(LED_BUILTIN, HIGH);   // Turn on LED
          ledState = true;
          u8x8.drawString(15, 4, "5");
        }
      }
    }
    buttonTime = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subroutines start here
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void updateMenu(void) {
/*
  if ( uiKeyCode != KEY_NONE && last_key_code == uiKeyCode ) {
    return;
  }
  last_key_code = uiKeyCode;

  switch ( uiKeyCode ) {
    case KEY_NEXT:
      menu_current_row++;
      if ( menu_current_row >= MENU_ITEMS )
        menu_current_row = 0;
      menu_redraw_required = 1;
      break;
    case KEY_PREV:
      if ( menu_current_row == 0 )
        menu_current_row = MENU_ITEMS;
      menu_current_row--;
      menu_redraw_required = 1;
      break;
  }
  */
}
//*************************************************************************************
void drawMenu(uint8_t CURRENT_MENU, uint8_t highlightRow) {

  uint8_t i, j;
  uint8_t s, w, h, d;

  for ( i = 0; i < MENU_ITEMS; i++ ) {
    if (CURRENT_MENU == 1) {
      u8x8.drawString(0, i * 2, menu_main[i*2]);
    } else {
      u8x8.drawString(d, i * h, menu_top[i*2]);
    }
    u8x8.setInverseFont(0);
  }

}

void loopMenu()
{
  Serial.println(F("Processing menu pushButton"));
  u8x8.clear();
  u8x8.setInverseFont(1);
  u8x8.drawString(0, 0, "Get Band");
  u8x8.setInverseFont(0);
  u8x8.drawString(0, 2, "Set Band");
  u8x8.drawString(0, 4, "Calibrate");
  u8x8.drawString(0, 6, "EXIT");
}

/**********************************************************************************************************/
void printPosition(void)
{
  int cnt;

  itoa(currentPosn, encoderPosString, 10);
  if (currentPosn < 10) {
    // Shift right 3 places with blanks in 1st 3 places
    encoderPosString[3] = encoderPosString[0];
    encoderPosString[0] = " ";
    encoderPosString[1] = " ";
    encoderPosString[2] = " ";
  } else if (currentPosn < 100) {
    // Shift right 2 places with blanks in 1st 2 places
    encoderPosString[3] = encoderPosString[1];
    encoderPosString[2] = encoderPosString[0];
    encoderPosString[0] = " ";
    encoderPosString[1] = " ";
  } else if (currentPosn < 1000) {
    // Shift right 1 place with blank in 1st place
    encoderPosString[3] = encoderPosString[2];
    encoderPosString[2] = encoderPosString[1];
    encoderPosString[1] = encoderPosString[0];
    encoderPosString[0] = ' ';
  }

  u8x8.drawString(12, 2, encoderPosString);
}

/**********************************************************************************************************/
void rotaryEncoderStep(boolean bCW, boolean &ledState)
{
  if (bCW == true) {
    encoderPosCount ++;
    if (ledState) {
      rotate(-5, .1);
    } else {
      rotate(-1, .1);
    }
  } else {
    // Otherwise B changed first and we're moving CCW
    encoderPosCount --;
    if (ledState) {
      rotate(5, .1);
    } else {
      rotate(1, .1);
    }
  }
  Serial.print(F("1 Encoder position count = "));
  Serial.print(encoderPosCount);
  Serial.print(F(" and Current position = "));
  Serial.println(currentPosn);

  printPosition();
}

/**********************************************************************************************************/
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
  //  itoa(currentPosn, encoderPosString, 10);
  Serial.print (F(":  counter = "));
  Serial.println(counter);
  //  itoa(currentPosn, encoderPosString, 10);
  //  u8x8.drawString(12, 0, encoderPosString);
  u8x8.drawString(0, 2, "Step Posn = ");
  printPosition();
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

  digitalWrite(enablePin, LOW); // Enable EasyDriver stepper driver stage
  delayMicroseconds(50); // Some settling time before stepping

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
  digitalWrite(enablePin, HIGH); // Disable EasyDriver stepper driver stage
}
/**********************************************************************************************************/
void updateButton(void) {

  // Get the current button state.

  static uint32_t startTime = micros() - pollTime; //Force immediate execution 1st time around

  if ((startTime + pollTime) > micros()) {
    return false; // We hadn't reached timeout so didn't process a button
  }

  startTime = micros();

#ifdef FEATURE_DEBUG_BTNS
  if (KI.count == 0) {
    Serial.print(KI.count); Serial.print("\t");
    Serial.println(KI.btn1_State, BIN);
    KI.count++;
  }
#endif

  KI.btn1_State = (KI.btn1_State << 1) | digitalRead(BTN1_PIN);

#ifdef FEATURE_DEBUG_BTNS
  if ((KI.count <= 25) && (KI.count != 0xFF)) {
    Serial.print(KI.count); Serial.print("\t");
    Serial.println(KI.btn1_State, BIN);
    KI.count++;
  } else {
    if (KI.count != 0xFF) Serial.println("-----------------------------------------");
    KI.count = 0xFF;
  }
#endif
  if ((KI.btn1_State & MASK1) == releasePattern) {
    KI.btn1_State = 0x5555;
#ifdef FEATURE_DEBUG_BTNS
    KI.count = 0;
    Serial.println("Released");
#endif
  }
  if ((KI.btn1_State & MASK1) == pressPattern) {
    //    pressed = true;
    KI.btn1_State = 0xAAAA;
#ifdef FEATURE_DEBUG_BTNS
    KI.count = 0;
    Serial.println("Pressed");
#endif
  }

#ifdef BTN2_PIN
  KI.btn2_State = (KI.btn2_State << 1) | digitalRead(BTN2_PIN);
  if ((KI.btn2_State & MASK1) == releasePattern) {
    KI.btn2_State = 0x5555;
  }
  if ((KI.btn2_State & MASK1) == pressPattern) {
    KI.btn2_State = 0xAAAA;
  }
#endif
#ifdef BTN3_PIN
  KI.btn3_State = (KI.btn3_State << 1) | digitalRead(BTN3_PIN);
  if ((KI.btn3_State & MASK1) == releasePattern) {
    KI.btn3_State = 0x5555;
  }
  if ((KI.btn3_State & MASK1) == pressPattern) {
    KI.btn3_State = 0xAAAA;
  }
#endif
#ifdef BTN4_PIN
  KI.btn4_State = (KI.btn4_State << 1) | digitalRead(BTN4_PIN);
  if ((KI.btn4_State & MASK1) == releasePattern) {
    KI.btn4_State = 0x5555;
  }
  if ((KI.btn4_State & MASK1) == pressPattern) {
    KI.btn4_State = 0xAAAA;
  }
#endif
}
