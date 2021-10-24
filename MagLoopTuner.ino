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
const uint8_t pinA = 4;    // Connected to CLK on KY-040
const uint8_t pinB = 5;    // Connected to DT on KY-040
const uint8_t pinBtn = 6;  // Connected to Push Button on KY-040
const uint8_t calBtn = 7;  // Connected to the calibrate button
const uint8_t endStopPin = 8;
const uint8_t maxCendstop = 10;
const uint8_t minCendstop = 12;

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

int encoderPosCount = 0;
int mode = 0;

uint16_t interrupt2maxC = 0;
uint16_t backlash = 0;
uint16_t minimumC = stepsPerShaftRev / 2 * microStepsPerStep * 1; // Drive it right to the end
uint16_t maximumC = 0; // Position 0 is plates fully meshed

int currentPosn = 1;
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
  pinMode(pinA, INPUT); digitalWrite(pinA, HIGH); // Pullups are part of the encoder component
  pinMode(pinB, INPUT); digitalWrite(pinB, HIGH);
  pinMode(pinBtn, INPUT); digitalWrite(pinBtn, HIGH); // Enable pullup for this one
  pinMode(calBtn, INPUT); digitalWrite(calBtn, HIGH); // Enable pullup for this one
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
    Serial.println (F("  EEPROM VALUES"));
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
  } else { // No valid calibration data so flash the LED's
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
  static uint8_t Push_button_history = 0;
  static uint8_t Cal_button_history = 0;
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
      if (ledState) {
        rotate(-5, .1);
      } else {
        rotate(-1, .1);
      }
    } else {
      //      Serial.print(digitalRead(pinA)); Serial.print(" <A else B> "); Serial.println(b);
      // Otherwise B changed first and we're moving CCW
      encoderPosCount --;
      bCW = false;
      if (ledState) {
        rotate(5, .1);
      } else {
        rotate(1, .1);
      }
    }

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
      if (ledState) {
        rotate(-5, .1);
      } else {
        rotate(-1, .1);
      }
    } else {
      //      Serial.print(digitalRead(pinA)); Serial.print(" <A release else B> "); Serial.println(b);
      //  Going anticlockwise
      encoderPosCount --;
      bCW = false;
      if (ledState) {
        rotate(5, .1);
      } else {
        rotate(1, .1);
      }
    }
    Serial.print ("Encoder position count = ");
    Serial.print(encoderPosCount);
    Serial.print (" and Current position = ");
    Serial.println(currentPosn);
  }

  update_button(&Push_button_history, pinBtn);
  if (is_button_pressed(&Push_button_history)) {
    Serial.println ("Encoder button press detected");
    // Toggle LED
    if (ledState) {
      digitalWrite(LED_BUILTIN, LOW); // Turn off LED
      ledState = false;
    } else {
      digitalWrite(LED_BUILTIN, HIGH);   // Turn on LED
      ledState = true;
    }
  }
  update_button(&Cal_button_history, calBtn);
  if (is_button_pressed(&Cal_button_history)) {
    Serial.println ("Calibrate button press detected");
    calibrate();
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
void setPosition()
// We are rotating clockwise (facing the capacitor shaft) to increase the capacity until it reaches the
// end stop. We count each step taken to do this. When the end stop is reached the current position is set
// to interrupt2maxC and we command the stepper to step anticlockwise the number of steps counted to restore
// the original position.
{
  // Set to the Interrupter position
  int endStatus = digitalRead(endStopPin); //High when interrupted
  uint16_t counter = 0;
  Serial.print ("currentPosn start = ");
  Serial.println (currentPosn);
  while (!endStatus) {
    rotate(1, .1); // Step clockwise
    counter++;
    endStatus = digitalRead(endStopPin);
    //    delay(1); // Add delay here to slow down switch rotation speed
  }
  currentPosn = interrupt2maxC;
  Serial.print ("currentPosnA = ");
  Serial.print (currentPosn);
  Serial.print (":  counterA = ");
  Serial.println(counter);

  // If we were at Interrupter position when setPosition called no steps would have been taken so we don't
  // need to do anything else otherwise we restore the original position of the capacitor.
  if (counter != 0) {
    rotate(-counter, .1);
  }
  Serial.print ("currentPosn = ");
  Serial.print (currentPosn);
  Serial.print (":  counter = ");
  Serial.println(counter);
}
/**********************************************************************************************************/
void rotate(int steps, float speed) {
  // Rotate a specific number of microsteps (8 microsteps per step) - (negitive for reverse movement)
  // speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  // This routine checks for a change of direction and applies the backlash correction.

  static int oldDir = LOW;
  int dir = (steps > 0) ? HIGH : LOW;
  steps = abs(steps); // Convert to positive number if negative
  if (mode == 0) {
    if (dir != oldDir) {
      steps += backlash;
      oldDir = dir;
    }
  }
  boolean y = false;
  float usDelay = (1 / speed) * 140;

  digitalWrite(DIR_PIN, dir); // Set the rotation direction on the Easy Stepper

  for (int i = 0; i < steps; i++) {
    if (mode == 0) {    // In calibration mode, prevent detecting the end stop condition
      y = digitalRead(endStopPin);
    }
    if (y && dir) {
      digitalWrite(maxCendstop, HIGH);
      if (!mode) { // Don't honour endstops if in "Calibrate mode"
        break; // Only let it step clockwise if at end stop
      }
    } else {
      digitalWrite(maxCendstop, LOW);
    }
    if ((currentPosn >= minimumC) && !dir) {
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
    if (dir) {
      currentPosn--;
    } else {
      currentPosn++;
    }
  } // End of for loop
}
/**********************************************************************************************************/
/*
  void rotateDeg(float deg, float speed) {
  //rotate a specific number of degrees (negative for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (deg > 0) ? HIGH : LOW;

  digitalWrite(DIR_PIN, dir);

  int steps = abs(deg) * (1 / 0.9375); // 360/384=.9375
  float usDelay = (1 / speed) * 140;

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
  }
  }
*/
/**********************************************************************************************************/
void eeprom_Load(unsigned int freq)
{
  // Loads the passed frequency into the eeprom table of presets
  // If freq = 0 then the table is cleared and reset to address 0 as start/end of table
  // If frequency is higher than any other in the table it will be appended
  // If frequency is lower than any table entry it will be pre-pended
  // Otherwise the frequency will be inserted so as to keep the table in ascending frequency.

  int eeAddress = 0;
  int eeAddrHi = 0;
  boolean endFlag = false;

  if (freq == 0) {
    eepromTermAddr = 0;
    EEPROM.put(eepromTermAddr, 0);
    Serial.println(F("Relay table has been zeroed"));
  } else {
    /*
        EEPROM.get(eeAddress, val.freq);
        if (val.freq == 0) {
          //    endFlag = true; // If 1st addr = 0 (terminator) then we have a non loaded eeprom
          EEPROM.put(eeAddress, freq);
          eepromTermAddr += sizeof(MyValues);
          EEPROM.put(eepromTermAddr, 0);
          Serial.print(F("eeAddress = ")); Serial.print(eeAddress);
          Serial.print(F(", eepromTermAddr = ")); Serial.println(eeAddress);
        }
        while ((val.freq) || (endFlag)) {
          // We only get here if val.freq is a non zero number or we want to append the frequency
          if (val.freq == freq) {
            // The frequency we are entering matches a frequency already in the presets
            // so simply overwrite it.
            val.L = _status.L_relays;
            val.C = _status.C_relays;
            val.Z = _status.outputZ;
            EEPROM.put(eeAddress, val);
            Serial.print(F("freq ")); Serial.print(val.freq); Serial.print(F(" written to presets at address "));
            Serial.println(eeAddress);
            break;
          } else {
            Serial.print(F("val.freq = ")); Serial.print(val.freq);
            Serial.print(F(";  freq = ")); Serial.print(freq);
            Serial.print(F(";  eeAddress = ")); Serial.println(eeAddress);
            // -------------------------------------------------------
            if ((val.freq > freq) || (endFlag)) { // Don't process for frequencies less than the entry frequency
              // val.freq must be greater than freq or we have reached the end to get here.

              // We are going to insert the tune values in the address before this one
              // (which is held in eeAddrHi). All the values above will need to be
              // shifted up one to allow the tune data to be inserted.
              Serial.println();

              eeAddrHi = eeAddress;
              eeAddress = eepromTermAddr;;
              eepromTermAddr += sizeof(MyValues);//Move address to one struct item beyond
              EEPROM.put(eepromTermAddr, 0);
              Serial.print(F("Entering while loop, eeAddrHi = ")); Serial.print(eeAddrHi);
              Serial.print(F(";  eeAddress = ")); Serial.print(eeAddress);
              Serial.print(F(";  eepromTermAddr = ")); Serial.println(eepromTermAddr);

              while ((eeAddress >= eeAddrHi) || (eeAddress == 0)) {
                EEPROM.get(eeAddress, val.freq);
                Serial.print(F("Start address = ")); Serial.print(eeAddress); Serial.print(F("  val.freq = ")); Serial.print(val.freq);
                eeAddress += sizeof(unsigned int);  // Step off freq to L
                EEPROM.get(eeAddress, val.L);
                eeAddress += sizeof(byte);          // Step off L to C
                EEPROM.get(eeAddress, val.C);
                eeAddress += sizeof(byte);          // Step off C to Z
                EEPROM.get(eeAddress, val.Z);
                eeAddress += sizeof(byte);
                EEPROM.put(eeAddress, val);
                Serial.print(F("  Finish address = ")); Serial.println(eeAddress);
                eeAddress -= (sizeof(MyValues) * 2);
              }

              val.freq = freq;
              val.L = _status.L_relays;
              val.C = _status.C_relays;
              val.Z = _status.outputZ;
              eeAddress = eeAddrHi;
              EEPROM.put(eeAddress, val);
              Serial.print(F("insert freq ")); Serial.print(val.freq); Serial.print(F(" written to presets at address "));
              Serial.println(eeAddress);
              eeAddress = (eepromTermAddr - sizeof(MyValues));
              break;

            }
            // -------------------------------------------------------

            eeAddress += sizeof(MyValues);//Move address to the next struct item
            //      Serial.print(F("eeAddress = ")); Serial.println(eeAddress);
            EEPROM.get(eeAddress, val.freq); //Get next frequency or 0000 if at end
            if (val.freq == 0) endFlag = true;
            //      Serial.print(F("Exiting else, val.freq = ")); Serial.println(val.freq);
          }
        }
        EEPROM.put(eepromTermAddr, 0);
        Serial.print(F("eeprom_Load() exit freq ")); Serial.print(val.freq);
        Serial.print(F(", eepromTermAddr = ")); Serial.println(eepromTermAddr);
    */
    eeprom_Print();
  }
}

//--------------------------------------------------------------------------------------------------------/

void eeprom_Print()
{
  int eeAddress = 0;

  Serial.println(F("-------------------"));
  Serial.println(F("Freq, L_relays, C_Relays, outputZ, Address"));
  /*
    EEPROM.get(eeAddress, 0);

    while (val.freq) {
      eeAddress += sizeof(unsigned int);  // Step off freq to L
      EEPROM.get(eeAddress, val.L);
      eeAddress += sizeof(byte);          // Step off L to C
      EEPROM.get(eeAddress, val.C);
      eeAddress += sizeof(byte);          // Step off C to Z
      EEPROM.get(eeAddress, val.Z);
      eeAddress += sizeof(byte);

      Serial.print(val.freq);
      Serial.print(F("\t")); Serial.print(val.L);
      Serial.print(F("\t  ")); Serial.print(val.C);
      Serial.print(F("\t   ")); Serial.print(val.Z);
      Serial.print(F("\t    ")); Serial.println(eeAddress - sizeof(MyValues));

      EEPROM.get(eeAddress, val.freq); //Get next frequency or 0000 if at end
    }
  */
  Serial.print(0);
  Serial.print(F("  eepromTermAddr = ")); Serial.println(eepromTermAddr);
  Serial.println(F("-------------------")); Serial.println();
}
/**********************************************************************************************************/
void eeprom_initialise()
{
  // this routine loads some preset values into eeprom for fast tuning. In the final version this will become
  // redundant as the values will be loaded by frequency with the counter installation. A magic number is
  // loaded into the last byte of the eeprom to indicate that it is already loaded with tune values and we
  // don't duplicate the data at each switch on.

  int eeAddress = 0;

  if (EEPROM[EEPROM.length() - 1] != 120) {
    EEPROM.put(eeAddress, 100);
    EEPROM[EEPROM.length() - 1] = 120; //Put a marker to show that data has been loaded into the eeprom
    Serial.println(F("EEPROM initialised"));
  } else {
    Serial.println(F("EEPROM was already initialised"));
  }
}
