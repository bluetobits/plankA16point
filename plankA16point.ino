//PlankA16 v10.3 Points Steve Lomax June 14/06/2025 condensed user settings
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Encoder.h>
#include <EEPROM.h>
#include <PCF8575.h>            //https://github.com/RobTillaart/PCF8575
#include <FastLED.h>            //https://github.com/FastLED/FastLED
#include <LiquidCrystal_I2C.h>  //https://github.com/johnrickman/LiquidCrystal_I2C


#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugln2(x, y) Serial.println(x, y)
#else
#define debug(x)
#define debugln(x)
#define debugln2(x, y)
#endif

#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

//////////// USER CONGIGURABLE VARIABLES //////////
///////////////////////////////////////////////////

//CHANGE THE VALUES FOR THE i2c ADDRESSES IDENTIFIED ON THE SERIAL MONITOR AT STARTUP. REMEMBER TO SET ADDRESS LINKS
constexpr int PCF1_ADDRESS = 0X20;  // FIRST EXPANDER
//constexpr int PCF2_ADDRESS = 0X22;// SECOND EXPANDER
constexpr int PCA1_ADDRESS = 0X40;  //FIRST SERVO DRIVER
//constexpr int PCF2_ADDRESS = 0X43;//SECOND SERVO DRIVER
constexpr int LCD_ADDRESS = 0X27;   //LCD DISPLAY

constexpr int NO_OF_SERVOS = 16;    // ENTER THE NUMBER OF SERVOS UP TO 16 this can be 16 even if not all implemebted 
const uint8_t NO_OF_LEDS = 16;      // ENTER THE NUMBER OF NEO PIXEL LEDs (edit setLeds() function to customise LED logic)

// POINT_PAIRS
// This determines which points are opreated together my one switch. 
// the default numerical order infers that each switch operates its own point
// the switch is represented by the number, the point is represented by the position in the list
// a negative number denotes that point thrown / closed position is reversed
// the lowest switch position in a pair or group is the operating switch for the pair or group
// every point must have a switch allocated to it.
// the lowest switch in a group must operate it's own point 
// point paiting is disabled when calibrating or disabled in the menu.

constexpr int8_t POINT_PAIRS[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

// switches are numbers            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 
// points are positions            |  |  -  |  -  |  |  -  |  -  |   -   |   |   |   |                               
// example point pairs             0, 1, 1, 3,-3, 5, 5, 3, 8, 8, 10,-10, 12, 13, 14, 15
// in this example                    !..!  |  |  !..!  |  !..!  !...!
//                                          !..!........!
// switch 0 -> point 0, switch 1 -> points 1 & 2, switch 2 -> not used, switch 3-> points 3 & 4(reversed) & 7,
// switch 4 not used, switch 5 -> points 5 & 6 , switch 7 not used, switch 8 -> points 8 & 9, ...

//LEDS_MIMIC this is the order of the leds for each point. default is 1 to 1 - led 1 indicates point 1. 
// if editing ensure each led is exclusively allocated a point number. by its position in the list. 
const uint8_t LEDS_MIMIC[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

const uint8_t DATA_PIN = 11;  // neopixels data connect to pin via 50 ohm resistor
uint8_t onSat = 255;          // Global. Neopixel saturation level (0-255)  255 is pure colour, 122 is pale colour, 0 is white
uint8_t onLev = 80;           // Global. Neopixel brightness level (0-255) 0 is not lit. 255 is max brightness (and current)
uint8_t onHue = 255;          // Global. Neopixel colour ROYGBIV from 0 to 255

constexpr int ENCODER_PUSH = 8;// pin connection (input pullup switch to gnd) 
const uint8_t ENCA_PIN = 3;  // encoder pin A pinmode set by library pin 2 or 3 recommended
const uint8_t ENCB_PIN = 2;  // encoder pin B pinmode set by library pin 2 or 3 recommended

int moveSpeed = 60; //* initial move speed. increase to make faster. this is adjusted through menu.
int pointPairing = 0;  //* flag if selected. int because memstruct must be the same tyoes 

const int MIN_MOVE_SPEED = 1; // must not be zero
const int MAX_MOVE_SPEED = 100; // as fast as the servos can travel
const int MID_POINT = 1500;//for setting servos
const int TOP_PULSE_LEN = 2400;    // *setting the maximum cw servo position(actual = 2500 but not all servos are the same)
const int BOTTOM_PULSE_LEN = 600;  // *setting the minimum ccw servo position

//* these values are overwritten by EEPROM and will only exist until saved from the menu option.
const unsigned long flashSpeed = 400;  // flash rate in ms for LED indicators.

//////////// END OF USER CONGIGURABLE VARIABLES //////////




constexpr int EEPROM_ADDRESS = 0;  // Define EEPROM base address
int cal = 0;
int localAutomation = 0;  //flag if selected  int because memstruct must be the same tyoes
bool centreServoFlag = 0;
int lastPointMoved = 0;

int encoderPos = 0;

CRGB leds[NO_OF_LEDS];  //LED neopixel strip
//colour Hues:
const uint8_t Hred = 0;
const uint8_t Hora = 32;
const uint8_t Hyel = 64;
const uint8_t Hgre = 96;
const uint8_t Hcya = 128;
const uint8_t Hblu = 160;
const uint8_t Hpur = 192;
const uint8_t Hpin = 224;
bool moving = 0;



unsigned long timeNow;
unsigned long flashTimeNow;
unsigned long flashTimeout = 0;        //Global.  Neopixel flash timer
bool flash = 0;                        //Global.  Neopixel flash status

Adafruit_PWMServoDriver PCA1 = Adafruit_PWMServoDriver(PCA1_ADDRESS);
//Adafruit_PWMServoDriver PCA2 = Adafruit_PWMServoDriver(PCA2_ADDRESS);
PCF8575 PCF1(PCF1_ADDRESS);  // Set the PCF1 I2C address (default is 0x20)
//PCF8575 PCF2(PCF2_ADDRESS);  // Set the PCF2 I2C address (default is 0x20)
Encoder encoder(ENCA_PIN, ENCB_PIN);
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display


const int MENU_COUNT = 6;
const char* menuItems[MENU_COUNT] = {
  "Exit", "Calibrate Positions", "Set Throw Speed",
  "Point Pairing", "Local Automation", "Centre Servo"
};
int currentMenuIndex = 0;
long lastEncoderPos = 0;
int subMenuIndex = 0;
enum SubMenuType {
  CALIBRATION,
  POINT_PAIRING,
  LOCAL_AUTOMATION,
  SET_THROW_SPEED,
  CENTRE_SERVO
};



struct MemStruct {
  int mclosedPos[NO_OF_SERVOS];
  int mthrownPos[NO_OF_SERVOS];
  int mmoveSpeedMem;
  int mpointPairing = 1;     // save point pairing enabled status
  int mlocalAutomation = 0;  //save localAutomation enabled status

  void
  readEEPROM() {
    EEPROM.get(EEPROM_ADDRESS, *this);
  }
  void writeEEPROM() {
    EEPROM.put(EEPROM_ADDRESS, *this);
  }
};

MemStruct memData;


struct Points {
  int closedPos = 700;
  int thrownPos = 2300;
  int curPos = 700;
  bool target = 0;
  void MovePoint(int index) {
    // Determine desired position
    int targetPos = target ? thrownPos : closedPos;
    int delta = targetPos - curPos;  //DTG from current pos.
    if (delta != 0) {
      // Move by at most moveSpeed each call
      int step;                               // distance to move in this call
      if (delta > 0) {                        //clockwise movement
        if (delta < moveSpeed) step = delta;  //if the DTG is less than movespeed set the step to the DTG
        else step = moveSpeed;                // otherwise set step to the movespeed
      } else {
        if (delta > -moveSpeed) step = delta;  // same for anticlockwise.
        else step = -moveSpeed;
      }
      curPos += step;
      moving = true;
      // if (index < 16) {
      PCA1.writeMicroseconds(index, curPos);
      // } else {
      //   PCA2.writeMicroseconds(index - 16, curPos);
      // }
    }
  }
};

Points point[NO_OF_SERVOS];


void savePointValues() {

  for (int i = 0; i < NO_OF_SERVOS; i++) {
    memData.mthrownPos[i] = point[i].thrownPos;

    memData.mclosedPos[i] = point[i].closedPos;
  }
  memData.mmoveSpeedMem = moveSpeed;
  memData.mpointPairing = pointPairing;
  memData.mlocalAutomation = localAutomation;
  memData.writeEEPROM();
  debug("written pointPairing = ");
  debug(pointPairing);
  debug(" Written moveSpeed = ");
  debugln(moveSpeed);
  loadPointValues();
}
void loadPointValues() {

  memData.readEEPROM();
  for (int i = 0; i < NO_OF_SERVOS; i++) {  // transfer memory values to point values
    point[i].thrownPos = memData.mthrownPos[i];
    point[i].closedPos = memData.mclosedPos[i];
    if (point[i].thrownPos < BOTTOM_PULSE_LEN || point[i].thrownPos > TOP_PULSE_LEN) {
      point[i].thrownPos = 1700;
    }
    if (point[i].closedPos < BOTTOM_PULSE_LEN || point[i].closedPos > TOP_PULSE_LEN) {
      point[i].closedPos = 1300;
    }
  }
  moveSpeed = memData.mmoveSpeedMem;
  pointPairing = memData.mpointPairing;
  localAutomation = memData.mlocalAutomation;
  if (pointPairing > 1) pointPairing = 1;
  if (localAutomation > 1) localAutomation = 1;
  if (moveSpeed < 3) moveSpeed = 3;
  if (moveSpeed > 200) moveSpeed = 200;
  debug("loaded pointPairing = ");
  debug(pointPairing);
  debug("  loaded moveSpeed = ");
  debugln(moveSpeed);
}

void startPos() {
  int tempspeed = moveSpeed;
  moveSpeed = MID_POINT;
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    point[i].target = 0;
    point[i].MovePoint(i);
  }
  moveSpeed = tempspeed;
}
void scanButtons() {
  // unsigned long nowMillis = millis();
  // if (nowMillis - pollTimeOut >= pollFreq) {
  //   pollTimeOut = millis();
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    if (i < 16) {

      if (!PCF1.read(i)) {
        while (!(PCF1.read(i))) {}
        delay(10);
        // debug("PCF1 pressed for servo ");
        // debug(i);

        point[i].target = !point[i].target;  //flip the target
        point[i].MovePoint(i);
        pointPairs(i);
        lastPointMoved = i;
        lcdPos();  //writes the position character
      }
    }
    // else {
    //   if (!PCF2.read(i - 16)) {
    //     while (!PCF2.read(i - 16)) {}
    //     delay(10);
    //     // debug("PCF2 pressed for servo ");
    //     // debug(i);

    //     point[i].target = !point[i].target;
    //     point[i].MovePoint(i);
    //     pointPairs(i);
    //     lastPointMoved = i;
    //     lcdPos();  //writes the position character
    //   }
    // }
  }
}
void pointPairs(int idx) {  //idx = the current switched point
  if (pointPairing) {
    //
    for (int k = idx; k < NO_OF_SERVOS; k++) {  // from current position through to all higher
      if (k != POINT_PAIRS[idx]) {
        if (abs(POINT_PAIRS[k]) == idx) {  // if the value of point Paris  == the current position
          POINT_PAIRS[k] < 0 ? (point[k].target = !point[idx].target) : (point[k].target = point[idx].target);
          point[k].MovePoint(k);
        }
      }
    }
  }
}


void offLeds() {
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    leds[LEDS_MIMIC[i]] = CHSV(Hred, onSat, 0);  // set all to red off
    FastLED.show();
  }
}
void setLeds() {
  flashTimeNow = millis();
  if (flashTimeNow > (flashTimeout)) {         // every 400ms change flash on to flash off or vice versa
    flash = !flash;                            // whatever flash state is (on or off), make it the opposite
    flashTimeout = flashTimeNow + flashSpeed;  //reset the timer
  }
  for (int i = 0; i < NO_OF_SERVOS; i++) {
    // loop through all points
    leds[LEDS_MIMIC[i]] = CHSV(Hred, onSat, onLev);  // set all to red
    if (!point[i].target) {
      leds[LEDS_MIMIC[i]] = CHSV(Hgre, onSat, onLev);  //set closed points to green
    }
    if (pointPairing) {
      if (POINT_PAIRS[i] != i) {
        leds[LEDS_MIMIC[i]] = CHSV(Hora, onSat, onLev);  // set all paired to orange
        if (!point[i].target) {
          leds[LEDS_MIMIC[i]] = CHSV(Hcya, onSat, onLev);  //set closed paired points to cyan
        }
      }
    }

    if (moving) {
      if (!point[i].target) {
        if (point[i].closedPos != point[i].curPos) {
          leds[LEDS_MIMIC[i]] = CHSV(Hblu, onSat, onLev);  //set points not in their correct position to blue (moving points)
        }
      } else {
        if (point[i].thrownPos != point[i].curPos) {
          leds[LEDS_MIMIC[i]] = CHSV(Hblu, onSat, onLev);  //set points not in their correct position to blue (moving points)
        }
      }
    }


    if (flash) {
      if (cal && i == lastPointMoved) {
        leds[LEDS_MIMIC[i]] = CHSV(Hpur, onSat, onLev);  //set points being calibrated to purple
      }
    }

    if (centreServoFlag) {
      if (point[i].thrownPos == MID_POINT && point[i].closedPos == MID_POINT) {
        leds[LEDS_MIMIC[i]] = CHSV(Hyel, onSat, onLev);
      }
    }
  }
  FastLED.show();
}
void lcdGrid() {
  lcd.setCursor(0, 0);
  lcd.print(F("Pnt 0123456789012345"));
  lcd.setCursor(0, 1);
  lcd.print(F("Pos "));
  lcd.setCursor(0, 2);
  lcd.print(F("Pnt 6789012345678901"));
  lcd.setCursor(0, 3);
  lcd.print(F("Pos "));
}

void lcdPos() {
  if (currentMenuIndex == 0) {
    lcd.setCursor(4, 1);
    for (int i = 0; i < 16; i++) {
      lcd.print(
        (pointPairing && POINT_PAIRS[i] != i) ? (point[i].target ? F("I") : F("\"")) : (point[i].target ? F("|") : F("-")));
    }

    lcd.setCursor(4, 3);
    for (int i = 0; i < 16; i++) {
      lcd.print(
        (pointPairing && POINT_PAIRS[i + 16] != i + 16) ? (point[i + 16].target ? F("I") : F("\"")) : (point[i + 16].target ? F("|") : F("-")));
    }
  }
}
void lcdPrint() {

  if (cal) {

    lcd.setCursor(0, 1);
    lcd.print(F("Point "));
    lcd.print(lastPointMoved);
    lcd.print(F("  "));
    lcd.setCursor(9, 1);

    if (point[lastPointMoved].target) {
      lcd.print(F("THROWN "));
    } else {
      lcd.print(F("CLOSED "));
    }
    // lcd.setCursor(14, 1);
    lcd.print(point[lastPointMoved].curPos);
    lcd.print(" ");
    // lcd.setCursor(1, 2);
    // lcd.print("Long Push to save");
  }
}
void centreServo() {
  int oldmoveSpeed = moveSpeed;
  moveSpeed = 1500;
  int curPoint = lastPointMoved;
  centreServoFlag = 1;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Select point to"));
  lcd.setCursor(0, 1);
  lcd.print(F("Centre Servo"));
  lcd.setCursor(0, 2);


  while (digitalRead(ENCODER_PUSH) == 1) {
    scanButtons();
    if (curPoint != lastPointMoved) {

      curPoint = lastPointMoved;
      point[curPoint].closedPos = MID_POINT;
      point[curPoint].thrownPos = MID_POINT;
      // point[curPoint].curPos = 0;
      point[curPoint].MovePoint(curPoint);

      lcd.print(curPoint);
      lcd.print(F(" "));

      setLeds();
    }
  }
  while (digitalRead(ENCODER_PUSH) == 0) {}
  delay(100);
  moveSpeed = oldmoveSpeed;
}


//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[ HANDLESUBMENU ]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
void handleSubMenu(SubMenuType submenuType, bool hasSaveOption = true) {
  // 1) Determine title, firstOption, and disable Save for Centre Servo
  const char* title;
  const char* firstOption;
  switch (submenuType) {
    case CALIBRATION:
      title = "Calibration";
      firstOption = "Calibrate";
      break;
    case SET_THROW_SPEED:
      title = "Throw Speed";
      firstOption = "Adjust Speed";
      break;
    case POINT_PAIRING:
      title = "Point Pairs";
      firstOption = (pointPairing ? "Disable" : "Enable");
      break;
    case LOCAL_AUTOMATION:
      title = "Local Automation";
      firstOption = (localAutomation ? "Disable" : "Enable");
      break;
    case CENTRE_SERVO:
      title = "Centre Servo";
      firstOption = "Centre Point";
      hasSaveOption = false;
      break;
  }

  // 2) Build the menuStrings array
  const char* menuStrings[3];
  menuStrings[0] = firstOption;
  menuStrings[1] = hasSaveOption ? "Save and Exit" : "Undo and Exit";
  menuStrings[2] = hasSaveOption ? "Undo and Exit" : "";

  int menuItems = hasSaveOption ? 3 : 2;
  int selection = 0;
  int lastSelection = -1;
  long lastEncPos;

  // 3) Initialize encoder
  encoder.write(0);
  lastEncPos = encoder.read() / 4;

  // 4) Initial draw of title + all lines (no arrows yet)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(title);
  for (int i = 0; i < menuItems; i++) {
    lcd.setCursor(0, i + 1);
    lcd.print("  ");
    lcd.print(menuStrings[i]);
  }

  // 5) Menu loop
  while (true) {
    // a) read encoder and update selection
    long pos = encoder.read() / 4;
    if (pos != lastEncPos) {
      int diff = pos - lastEncPos;
      lastEncPos = pos;

      selection = (selection + diff + menuItems) % menuItems;
    }


    // b) redraw arrows and text if needed
    if (selection != lastSelection) {
      for (int i = 0; i < menuItems; i++) {
        lcd.setCursor(0, i + 1);
        lcd.print(i == selection ? "> " : "  ");
        lcd.print(menuStrings[i]);
      }
      lastSelection = selection;
    }

    // c) handle button press
    if (digitalRead(ENCODER_PUSH) == LOW) {
      while (digitalRead(ENCODER_PUSH) == LOW)
        ;
      delay(100);

      switch (selection) {
        case 0:
          if (submenuType == CALIBRATION) {
            calibrate();
            lastSelection = -1;
          } else if (submenuType == SET_THROW_SPEED) {
            pointMoveSpeed();
            lastSelection = -1;
          } else if (submenuType == POINT_PAIRING) {
            pointPairing = !pointPairing;
            setLeds();
            menuStrings[0] = pointPairing ? "Disable" : "Enable";
            lastSelection = -1;
          } else if (submenuType == LOCAL_AUTOMATION) {
            localAutomation = !localAutomation;
            menuStrings[0] = localAutomation ? "Disable" : "Enable";
            lastSelection = -1;
          } else if (submenuType == CENTRE_SERVO) {
            centreServo();
            lastSelection = -1;
          }
          break;

        case 1:
          if (hasSaveOption) {
            savePointValues();
          } else {
            loadPointValues();
          }
          return;

        case 2:
          // Only valid if hasSaveOption == true
          loadPointValues();
          return;
      }
    }

    delay(50);
  }
}
//[[[[[[[[[[[[[[[[[[[[[[[ POINT MOVE SPEED ]]]]]]]]]]]]]]]]]]]]]]]

void pointMoveSpeed() {
  // 1) Clear & draw static UI
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Adjusting Speed"));
  lcd.setCursor(0, 3);
  lcd.print(F("Press to exit"));
  encoder.write(0);
  int lastSpeedShown = -1;

  while (digitalRead(ENCODER_PUSH) == HIGH) {
    int delta = encoder.read() / 4;
    if (delta != 0) {

      encoder.write(0);
      moveSpeed = constrain(moveSpeed + delta, MIN_MOVE_SPEED, MAX_MOVE_SPEED);
    }
    if (moveSpeed != lastSpeedShown) {
      lastSpeedShown = moveSpeed;
      lcd.setCursor(0, 2);
      lcd.print(F("Speed: "));
      lcd.print(moveSpeed);
      lcd.print(F("    "));
    }

    int tempPointPairing = pointPairing;
    pointPairing = 0;
    scanButtons();

    if (moving) {
      moving = 0;
      for (int i = 0; i < NO_OF_SERVOS; i++) {
        point[i].MovePoint(i);
      }
    }
  }
  while (digitalRead(ENCODER_PUSH) == LOW) {}
  delay(100);
}

//[[[[[[[[[[[[[[[[[[[[[[[[[[[[ CALIBRATE ]]]]]]]]]]]]]]]]]]]]]]]]]]]]
void calibrate() {
  cal = 1;  // set cal flag
  // Backup current state
  int originalPointPairing = pointPairing;
  pointPairing = false;

  int originalMoveSpeed = moveSpeed;
  moveSpeed = 2500;

  // Initialize encoder
  encoder.write(0);
  lastEncoderPos = encoder.read() / 4;

  lcd.clear();

  // Draw static elements once
  lcd.setCursor(0, 0);
  lcd.print(F("Adjusting Point"));
  lcd.setCursor(0, 3);
  lcd.print(F("Turn/Change/Press  "));  // 🔹 Bug 2 fix


  int lastPosShown = -1;
  int lastPoint = -1;
  int lastTarget = -1;

  // Calibration loop
  while (digitalRead(ENCODER_PUSH) == HIGH) {
    scanButtons();  // Updates lastPointMoved and toggles its target
    setLeds();

    int& targetPos = (point[lastPointMoved].target == 0)
                       ? point[lastPointMoved].closedPos
                       : point[lastPointMoved].thrownPos;

    // Encoder adjustment
    int pos = encoder.read();
    if (pos != lastEncoderPos) {
      int diff = pos - lastEncoderPos;
      lastEncoderPos = pos;

      targetPos += diff;
      targetPos = constrain(targetPos, BOTTOM_PULSE_LEN, TOP_PULSE_LEN);
      point[lastPointMoved].curPos = targetPos;
      //point[lastPointMoved].MovePoint(lastPointMoved);  // ✅ this line is essential
      // if (lastPointMoved < 16) {
      PCA1.writeMicroseconds(lastPointMoved, targetPos);
      // } else {
      //   PCA2.writeMicroseconds(lastPointMoved - 16, targetPos);
      // }
      setLeds();
    }

    // Only update display if values change
    if (lastPoint != lastPointMoved || lastTarget != point[lastPointMoved].target || lastPosShown != point[lastPointMoved].curPos) {
      lastPoint = lastPointMoved;
      lastTarget = point[lastPointMoved].target;
      lastPosShown = point[lastPointMoved].curPos;

      lcd.setCursor(0, 1);
      lcd.print(F("Point: "));
      if (lastPointMoved < 10) lcd.print(F("0"));
      lcd.print(lastPointMoved);
      lcd.print(F(" "));
      lcd.print(lastTarget == 0 ? F("CLOSED") : F("THROWN"));
      lcd.print(F("   "));  // clear residual text

      lcd.setCursor(0, 2);
      lcd.print(F("Pos: "));
      lcd.print(lastPosShown);
      lcd.print(F("  "));
    }

    delay(50);  // debounce and flicker protection
  }


  while (digitalRead(ENCODER_PUSH) == LOW) {}  // Wait for button release
  delay(100);
  encoder.write(0);
  lcd.clear();  //
  // Redraw calibration menu
  lcd.setCursor(0, 0);


  pointPairing = originalPointPairing;
  moveSpeed = originalMoveSpeed;
  cal = 0;  //
}

//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[ MAIN MENU ]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
void lcdControlMenu() {

  if (digitalRead(ENCODER_PUSH) == HIGH) return;

  while (digitalRead(ENCODER_PUSH) == LOW) {}  // Wait for release

  currentMenuIndex = 0;
  encoder.write(currentMenuIndex * 4);
  lastEncoderPos = encoder.read() / 4;
  int lastMenuIndex = -1;

  displayMenu(currentMenuIndex);  // Initial draw

  while (true) {
    long pos = encoder.read() / 4;

    if (pos != lastEncoderPos) {
      int diff = pos - lastEncoderPos;
      lastEncoderPos = pos;
      currentMenuIndex = constrain((currentMenuIndex + diff), 0, MENU_COUNT - 1);
    }

    if (currentMenuIndex != lastMenuIndex) {
      displayMenu(currentMenuIndex);
      lastMenuIndex = currentMenuIndex;
    }

    if (digitalRead(ENCODER_PUSH) == LOW) {
      while (digitalRead(ENCODER_PUSH) == LOW)
        ;  // Wait for release
      lcd.clear();
      switch (currentMenuIndex) {
        case 0:

          lcdGrid();
          lcdPos();
          return;

        case 1:
          handleSubMenu(CALIBRATION);
          break;

        case 2:
          handleSubMenu(SET_THROW_SPEED);
          break;

        case 3:
          handleSubMenu(POINT_PAIRING);
          break;

        case 4:
          handleSubMenu(LOCAL_AUTOMATION);
          break;

        case 5:
          handleSubMenu(CENTRE_SERVO, false);  // No Save option
          break;
      }

      // Restore menu index after submenu
      encoder.write(currentMenuIndex * 4);
      lastEncoderPos = encoder.read() / 4;
      lastMenuIndex = -1;  // Force redraw
    }

    delay(50);  // debounce
  }
}

void displayMenu(int selectedIndex) {
  lcd.clear();


  int topIndex = selectedIndex < 2 ? 0 : selectedIndex > MENU_COUNT - 3 ? MENU_COUNT - 4
                                                                        : selectedIndex - 1;

  for (int i = 0; i < 4; i++) {
    int menuIndex = topIndex + i;
    lcd.setCursor(0, i);
    lcd.print((menuIndex == selectedIndex) ? "->" : "  ");
    lcd.print(menuItems[menuIndex]);
  }
}

void centreServoPos(int pointNo) {


  int tempMoveSpeed = moveSpeed;
  moveSpeed = MID_POINT;

  point[pointNo].thrownPos = MID_POINT;
  point[pointNo].closedPos = MID_POINT;
  point[pointNo].curPos = 0;
  point[pointNo].target = 0;
  point[pointNo].MovePoint(pointNo);
  point[pointNo].target = 1;
  point[pointNo].MovePoint(pointNo);

  moveSpeed = tempMoveSpeed;
}

//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[ SETUP ]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]
void setup() {
  Wire.begin();
  Serial.begin(9600);
  PCF1.begin();
  //PCF2.begin();

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NO_OF_LEDS);

  Serial.println("plankA16 12/06/2025 RS485 included not tested");
  int nDevices = 0;
  offLeds();
  Serial.println("Scanning I2C ");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("error at 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  lcd.init();  // initialize the lcd
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Point control v10.3");
  lcd.setCursor(2, 1);
  lcd.print("Steve Lomax 2025");
  delay(1000);

  encoder.write(0);

  pinMode(ENCODER_PUSH, INPUT_PULLUP);
  loadPointValues();

  PCA1.begin();
  PCA1.setPWMFreq(50);
  PCA1.setOscillatorFrequency(25000000);
  // PCA2.begin();
  // PCA2.setPWMFreq(50);
  // PCA2.setOscillatorFrequency(25000000);



  PCF1.write16(0Xffff);
  debug("PCF1 = ");
  //debugln2(PCF1.read16(), BIN);
  // PCF2.write16(0Xffff);
  // debug("PCF2 = ");
  // debugln2(PCF2.read16(), BIN);


  lcd.setCursor(3, 3);
  lcd.print(F("Loading..."));
  if (digitalRead(ENCODER_PUSH) == 0) {
    lcd.setCursor(0, 2);
    lcd.print(F("Hold = Centre Servo"));
    delay(2000);
  }
  if (digitalRead(ENCODER_PUSH) == 0) {
    lcd.setCursor(0, 3);
    lcd.print(F("Reposition points"));

    for (int i = 0; i < NO_OF_SERVOS; i++) {
      centreServoPos(i);
      centreServoFlag = 1;
    }
  }
  delay(2000);


  startPos();

  lcd.clear();
  lcdGrid();
  lcdPos();
}


void loop() {

  scanButtons();

  if (moving) {

    // lcdPos();
    moving = 0;
    for (int i = 0; i < NO_OF_SERVOS; i++) {
      point[i].MovePoint(i);
    }
  } else {
    lcdPos();
    lcdControlMenu();
  }
  setLeds();
}
