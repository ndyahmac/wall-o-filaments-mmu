// to do
//stepper drivers! ugh...
//

//FRAM storage stuff
#include <Adafruit_FRAM_I2C.h>
//colour sensor
#include <Adafruit_TCS34725.h>
//neopixel library
#include <Adafruit_NeoPixel.h>
//motor libraries
#include <SpeedyStepper.h>
#include <Servo.h>
//oled display stuff
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
//software reset!! (FOR AVR ONLY)
#ifdef ARDUINO_ARCH_AVR
#include <SoftwareReset.hpp>
#endif
//i2c GPIO expanders
#include <Adafruit_MCP23X17.h>
//Wire!
#include <Wire.h>

/*
custom gcode commands :]
G0   move x,e,s
G28  home x
G38  auto test max accel and speed (S for speed else accel) 
  these values are where it is no longer stable, so use something smaller than this
  only tests the selector, manual tuning is required for extruder.

G48  test X/E at speed

M701 Tn	Load filament n
M702 Tn	Unload filament n
M704 Preload
M705 report tool
M706 MMU self-test
M707 Sn	Set selector position directly
M708 Kill MMU
M709 Reset MMU
M710 scan i2c
M711 test colour (optional S fo repeat delay)
M712 get raw colour snesor values.
M713 Detect curent color
M714 DUMP FRAM (T to clear)
*/

//definitions, DO NOT TOUCH unless you know what you are doing.

//things to include or not
#define useserial true              //enable the Serial interface
#define useprefeederswitches false  //enable the prefeeder switches
#define includedebugcommands true   //enable certain debug gcode commands

//port expander i2c addresses
#define mcpaaddr 32
#define mcpbaddr 33

//oled display adress
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32 (typically)

//config
#define serialspeed 115200  //baud rate of serial interface, default is 9600
//  allowable baud rates
//  300,600,750,1200,2400,4800,9600,19200,31250,38400,57600,74880,115200,230400,250000,460800,500000,921600,1000000,2000000
//  typical baud rates
//  9600,115200

#define WIRE Wire     //for the i2c scanning function ( M710 over serial )
#define LINE_BUF 128  //buffer size for gcode commands, should be greater than 8.

//pin definitions
#define seldirPin 7        //direction pin of selector motor driver
#define selstepPin 8       //step pin of the driver
#define extdirPin 4        //direction pin of extruder motor driver
#define extstepPin 5       //step pin of the driver
#define endstop 6          //pin of the endstop for the selector
#define servopin 3         //selector servo pin
#define enable_pin 2       //pin to enable/disable the stepper drivers
#define inductiveprobe 12  //pin for the inductive probe for sensing filament
#define LED_STRIP_PIN 11   //pin for the led strip across the front

//button input pins
#define buttona A0
#define buttonb A1
#define buttonc A2

//motor characteristis
//24v ( needs calibrating)
// #define Selspeed 9000   //max speed in steps/s of the selector motor
// #define Extspeed 12000  //max speed in steps/s of the extruder motor
// #define accel 75000     //acceleration of both motors in steps/s2
//12v
#define Selspeed 4000      //max speed in steps/s of the selector motor
#define Extspeed 8000      //max speed in steps/s of the extruder motor
#define Extslowspeed 2000  //max speed in steps/s of the extruder motor
#define accel 120000       //acceleration of both motors in steps/s/s
#define homespeed 12       //mm/s for homing speed of the selector

//physical hardware details
#define firstpos 6.5       //mm from 0 where the first filament resides
#define increment 10       //increment in mm in between each consecutive filament slot
#define tools 23           //one less than the total filament slots
#define maxpos 236         //maximum position of the selector in mm
#define mindistfromhome 5  //safe position to move to after homing

//selector definitions
#define distancetoprobe 40         //the distance at which the filament has to move from the probe to not collide with the selector.
#define distancetocoloursensor 50  //the distance past the prob activation that the filament has to be pushed for the colour sensor to detect it. (preferable to let a little go further)
#define bowdentubelength 350       //length in mm of the bowden tube to the extruder on the tool head.
#define servoengagedpos 90         //position for the servo motor to engage the filament
#define servodisengagedpos 140     //position away from the filament for the servo motor to spin to.
#define servosafehomepos 180       //safe position for the servo moto to go to when homing.
#define stepspermili 14.72

//colour offsets
#define redoffset 55
#define greenoffset 91
#define blueoffset 104

#define tcsdelay 625

#define highlightcolour strip.Color(127, 0, 255)
#define basecolour strip.Color(255, 64, 0)
#define startupcolour strip.Color(0, 255, 0)
#define brightness 16

//init fram object
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
//gcode parser stuff (entirely so you can send a command and have it work)
struct GCode {
  bool hasG, hasM, hasT, hasS, hasX, hasE;
  int G, M, T, S, X, E;
};
GCode gc;


#if useprefeederswitches
Adafruit_MCP23X17 mcpa;
Adafruit_MCP23X17 mcpb;
#endif

// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
SSD1306AsciiWire display;

//defining motors
SpeedyStepper selector;
SpeedyStepper extruder;
Servo servo;

//defning the colour stuff (oohh shiny)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_4X);  //colour sensor
Adafruit_NeoPixel strip(tools + 1, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);
byte colours[3][tools];  //array to store the colour values(rgb for each tool)

//global variable definitions
bool engaged = 0;
int curtool = 0;
bool startchecksran = 0;
bool LOADED = 0;
bool enablecolour = 0;
bool FRAMENABLED = 0;
bool prefeederswitches = 0;
bool filamentrunout = 0;

int statled = 0;

#if useprefeederswitches
uint32_t prefeedbuttons = 0;
uint32_t oldprefeedbuttons = 0;
#endif

//button stuff
bool buttons[2][3];  //button list for storing current and previous states

//menu configs!!! and what not, yay...

//main menu
enum MenuState {
  MENU_MAIN,
  MENU_SELECT_TOOL,
  MENU_PRELOAD,
  MENU_LOAD,
  MENU_PRELOADALL,
  MENU_PRELOADALL2
};

MenuState menu = MENU_MAIN;
byte cursor = 1;
bool redraw = 1;
const byte menumaxval[] = { 4, tools, 1, 1, tools + 1, 1 };
const byte menuminval[] = { 1, 0, 1, 1, 0, 1 };

unsigned long long poweroncount = 0;
//start of actual code
void setup() {
#if useserial
  Serial.begin(serialspeed);
#endif

  // display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.begin(&Adafruit128x32, SCREEN_ADDRESS);

  display.clear();
  display.setFont(System5x7);
  display.setCursor(0, 0);
  display.println(F("Initializing"));


  if (fram.begin()) {  //init the FRAM, if it's not detected, the program will keep going.
    printStatus(F("Found FRAM"));
    fram.readObject(32759, poweroncount);
    if (poweroncount == 0xFFFFFFFFFFFFFFFF) {
      poweroncount = 0;
    }
    fram.writeObject(32759, poweroncount + 1);
    FRAMENABLED = 1;
  } else {
    printStatus(F("No FRAM"));
  }

  strip.begin();  // INITIALIZE NeoPixel strip object
  strip.clear();
  strip.setBrightness(brightness);
  strip.fill(startupcolour);
  strip.show();
  loadingleds(statled++, basecolour, highlightcolour);

#if useprefeederswitches
  if (mcpa.begin_I2C() && mcpb.begin_I2C()) {
    prefeederswitches = 1;
    for (int i = 0; i < 15; i++) {
      mcpa.pinMode(i, INPUT_PULLUP);
      mcpb.pinMode(i, INPUT_PULLUP);
    }
  } else {
    printStatus(F("no prefeed switches"));
  }
  loadingleds(statled++, basecolour, highlightcolour);
#endif

  pinMode(buttona, INPUT_PULLUP);
  pinMode(buttonb, INPUT_PULLUP);
  pinMode(buttonc, INPUT_PULLUP);

  pinMode(enable_pin, OUTPUT);
  pinMode(endstop, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(inductiveprobe, INPUT);

  disableMotors();

  loadingleds(statled++, basecolour, highlightcolour);

  servo.attach(servopin);
  servo.write(180);

  selector.connectToPins(selstepPin, seldirPin);

  selector.setSpeedInStepsPerSecond(Selspeed);
  selector.setAccelerationInStepsPerSecondPerSecond(accel);
  selector.setStepsPerMillimeter(50);

  extruder.connectToPins(extstepPin, extdirPin);

  extruder.setSpeedInStepsPerSecond(Extspeed);
  extruder.setAccelerationInStepsPerSecondPerSecond(accel);
  extruder.setStepsPerMillimeter(stepspermili);  //change and calibrate later, this is a temporary value and needs to be measured

  loadingleds(statled++, basecolour, highlightcolour);

  if (tcs.begin()) {
    printStatus(F("Found colour\nsensor!"));
    enablecolour = 1;
  } else {
    printStatus(F("no colour sensor"));
    enablecolour = 0;
  }
  loadingleds(statled++, basecolour, highlightcolour);
  if (FRAMENABLED) {
    restoreFromFRAM();
  }
  printStatus(F("initialized"));
  loadingleds(statled++, basecolour, highlightcolour);
}

//main loop
void loop() {
  if (startchecksran == 0) {
    if (digitalRead(inductiveprobe)) {
      runstartupchecks();
      restoreFromFRAM();
      updateleds();
    }
  }
  updatebuttonstate();
  PROCESSMENU();
  drawMenu();

#if useserial
  processSerial();
#endif

#if useprefeederswitches
  processprefeedswitches();
#endif
}
void processSerial() {
  //chatgpt code! love it, just wanted a gcode parser
  static char line[LINE_BUF];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) {
        line[len] = '\0';
        process_line(line);
        len = 0;
      }
    } else if (len < LINE_BUF - 1) {
      line[len++] = c;
    }
  }
}

void run_command(GCode& gc) {
  if (gc.hasG) {
    switch (gc.G) {
      case 48:
        servo.write(180);
        enableMotors();
        if (gc.hasE) {
          extruder.setSpeedInStepsPerSecond(gc.E);
          extruder.moveRelativeInMillimeters(1000);
          extruder.moveRelativeInMillimeters(-1000);
        }
        if (gc.hasX) {
          selector.setSpeedInStepsPerSecond(gc.X);
          selector.moveToPositionInMillimeters(maxpos);
          selector.moveToPositionInMillimeters(mindistfromhome);
        }
        disableMotors();
        return;
      case 38:
        if (gc.hasS) {
          getmaxspeed(250);
        } else {
          getmaxaccel(1000);
        }
        homeSelector();
        return;
      case 0:
        if (gc.hasE) {
          moveExtruder(gc.E);
        }
        if (gc.hasX) {
          enableMotors();
          selector.moveToPositionInMillimeters(gc.X);
          disableMotors();
        }
        if (gc.hasS) {
          servo.write(gc.S);
        }
        return;
      case 28:
        homeSelector();
        return;
    }
  }
  if (gc.hasM) {
    switch (gc.M) {
      case 0:
        STOP();
        return;
      case 701:
        if (gc.hasT) {
          select(gc.T);
          Loadfilament();
        } else missingT();
        return;

      case 702:
        Unloadfilament();
        return;
      case 704:
        if (gc.hasT) {
          select(gc.T);
          Preloadfilament();
        } else missingT();
        return;
      case 705:
        Serial.print(F("T"));
        Serial.println(curtool);
        return;

      case 707:
        if (gc.hasT) {
          select(gc.T);
        } else missingT();
        return;

      case 713:
        detectcolour(curtool);
        return;
#if includedebugcommands
      case 706:
        startchecksran = 0;
        runstartupchecks();
        return;

      case 708:
        STOP();
        return;
      case 709:
        printStatus(F("resetting..."));
#ifdef ARDUINO_ARCH_AVR
        softwareReset::standard();
#endif
#ifndef ARDUINO_ARCH_AVR
        NVIC_SystemReset();
#endif
        return;
      case 710:
        scani2c();
        return;
      case 711:
        testcolour();
        if (gc.hasS) {
          while (true) {
            testcolour();
            delay(gc.S);
          }
        }
        return;

      case 714:
        if (gc.hasX) {
          for (long i = 0; i < 32768; i++) {
            fram.write(i, gc.X);
          }
        }
        Serial.print(F("\nFRAM DUMP"));
        for (long i = 0; i < (gc.hasT ? gc.T : 32768); i++) {
          if (i % 256 == 0) {
            Serial.println();
            Serial.print(i, HEX);
            Serial.print(F(" : "));
          }
          Serial.print(fram.read(i), HEX);
        }
        return;

      case 712:
        strip.clear();
        strip.show();
        delay(10);
        float red, green, blue;
        tcs.setInterrupt(false);  // turn on LED
        delay(tcsdelay);          // takes 50ms to read
        tcs.getRGB(&red, &green, &blue);
        tcs.setInterrupt(true);  // turn off LED

        Serial.print(F("R: "));
        Serial.print(red);
        Serial.print(F(" G: "));
        Serial.print(green);
        Serial.print(F(" B: "));
        Serial.println(blue);
        return;
#endif
    }
  }
  Serial.println(F("error: unknown command"));
}

void missingT() {
  Serial.println(F("error: T missing"));
}

void process_line(char* line) {
  strip_comments(line);
  parse_gcode(line);
  run_command(gc);
}
void strip_comments(char* s) {
  for (uint8_t i = 0; s[i]; i++) {
    if (s[i] == ';' || s[i] == '(') {
      s[i] = '\0';
      return;
    }
  }
}
void parse_gcode(char* s) {
  memset(&gc, 0, sizeof(gc));
  while (*s) {
    if (*s == ' ' || *s == '\t') {
      s++;
      continue;
    }
    char code = *s++;
    int value = strtol(s, &s, 10);
    switch (code) {
      case 'G':
        gc.hasG = true;
        gc.G = value;
        break;
      case 'M':
        gc.hasM = true;
        gc.M = value;
        break;
      case 'T':
        gc.hasT = true;
        gc.T = value;
        break;
      case 'X':
        gc.hasX = true;
        gc.X = value;
        break;
      case 'E':
        gc.hasE = true;
        gc.E = value;
        break;
      case 'S':
        gc.hasS = true;
        gc.S = value;
        break;
    }
  }
}

void PROCESSMENU() {
  if (getbutton(0)) {
    cursor += 1;
    cursor = min(cursor, menumaxval[menu]);
    redraw = 1;
  }
  if (getbutton(1)) {
    if (menu == MENU_SELECT_TOOL) {
      menu = MENU_MAIN;
    } else if (menu == MENU_PRELOADALL) {
      menu = MENU_PRELOADALL2;
      redraw = 1;
    } else if (menu == MENU_PRELOADALL2) {
    } else {
      menu = MenuState(cursor);
      redraw = 1;
    }
  }
  if (getbutton(2)) {
    cursor -= 1;
    cursor = max(cursor, menuminval[menu]);
    redraw = 1;
  }
}

void drawMenu() {
  if (!redraw) return;
  redraw = false;

  display.clear();
  display.setCursor(0, 0);

  switch (menu) {
    case MENU_MAIN:
      display.println(cursor == 1 ? F("> Select Tool") : F("  Select Tool"));
      display.println(cursor == 2 ? F("> Preload Current") : F("  Preload Current"));
      if (LOADED == 0) {
        display.println(cursor == 3 ? F("> Load") : F("  Load"));
      } else {
        display.println(cursor == 3 ? F("> Unload") : F("  Unload"));
      }
      display.println(cursor == 4 ? F("> Preload ALL") : F("  Preload ALL"));
      break;
    case MENU_SELECT_TOOL:
      display.println(F("selecting filament!"));
      select(cursor);
      break;
    case MENU_PRELOAD:
      Preloadfilament();
      menu = MENU_MAIN;
      redraw = 1;
      break;
    case MENU_LOAD:
      if (LOADED) {
        Unloadfilament();
        redraw = 1;
        menu = MENU_MAIN;
      } else {
        Loadfilament();
        redraw = 1;
        menu = MENU_MAIN;
      }
      break;
    case MENU_PRELOADALL:
      display.print(F("starting From T0,\nPreloadall tools to\nT"));
      display.print(cursor);
      break;
    case MENU_PRELOADALL2:
      PreloadALL(cursor);
      break;
  }
}

void disableMotors() {
  digitalWrite(enable_pin, 1);
}

void enableMotors() {
  digitalWrite(enable_pin, 0);
}

void STOP() {
  disableMotors();
  servo.write(180);
  strip.fill(strip.Color(255, 0, 0), 0, tools + 1);
  strip.show();
  printStatus(F("MMU STOPPED"));
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
}

void printStatus(const __FlashStringHelper* status) {
  Serial.println(status);
  display.clear();
  display.setCursor(0, 0);
  display.println(status);
}

void disengage() {
  engaged = false;
  servo.write(servodisengagedpos);
  delay(250);
}

void engage() {
  engaged = true;
  servo.write(servoengagedpos);
  delay(250);
}
void select(int tool) {
  if (startchecksran == 0) {
    printStatus(F("run startup\nchecks first"));
    delay(1000);
    menu = MENU_MAIN;
    redraw = 1;
    return;
  }
  display.clear();
  display.setCursor(0, 0);
  enableMotors();
  if ((tool > tools) + (tool < 0)) {
    display.println(F("Tool Selection\nOut Of Range"));

  } else {
    display.print(F("switching tool\nT"));
    display.print(curtool);
    display.print(F(" -> T"));
    display.print(tool);

    if (engaged == false) {
      selector.moveToPositionInMillimeters(firstpos + (increment * min(max(tool, 0), tools)));
    }
    curtool = tool;
  }
  disableMotors();
}

void homeSelector() {
  printStatus(F("Homing Selector"));
  servo.write(servosafehomepos);
  enableMotors();
  if (selector.moveToHomeInMillimeters(1, 10, maxpos * 1.2, endstop) != true) {
    STOP();
  }
  selector.setSpeedInStepsPerSecond(1600);
  selector.moveRelativeInMillimeters(0.5);
  printStatus(F("moving away\nfrom switch"));
  selector.setSpeedInStepsPerSecond(100);
  printStatus(F("Homing Slow"));
  if (selector.moveToHomeInMillimeters(1, 2.5, 2.5, endstop) != true) {
    STOP();
  }
  selector.setSpeedInStepsPerSecond(Selspeed);
  selector.moveToPositionInMillimeters(mindistfromhome);
  printStatus(F("Homed Selector"));
  disableMotors();
  disengage();
}

void moveExtruder(float MM) {
  enableMotors();
  extruder.moveRelativeInMillimeters(MM);
  disableMotors();
}

void runstartupchecks() {
  loadingleds(statled++, basecolour, highlightcolour);
  engage();
  extruder.setSpeedInStepsPerSecond(Extslowspeed);
  moveExtruder(distancetoprobe + 10);
  if (!digitalRead(inductiveprobe)) {
    enableMotors();

    while (digitalRead(inductiveprobe)) {
      extruder.moveRelativeInSteps(1);
    }
    while (!digitalRead(inductiveprobe)) {
      extruder.moveRelativeInSteps(-1);
    }

    printStatus(F("Retracting from\nSelector"));
    moveExtruder(-distancetoprobe);
  }

  disengage();

  homeSelector();

  loadingleds(statled++, basecolour, highlightcolour);

  printStatus(F("Testing Selector"));
  enableMotors();

  loadingleds(statled++, basecolour, highlightcolour);

  selector.moveToPositionInMillimeters(maxpos);
  printStatus(F("Testing Extruder"));

  loadingleds(statled++, basecolour, highlightcolour);

  moveExtruder(200);
  moveExtruder(-200);
  printStatus(F("Testing Servo"));

  loadingleds(statled++, basecolour, highlightcolour);

  engage();
  delay(250);
  disengage();
  startchecksran = 1;

  loadingleds(statled++, basecolour, highlightcolour);

  select(0);
  disableMotors();

  loadingleds(statled++, basecolour, highlightcolour);

  testcolour();
  printStatus(F("Done!"));
  redraw = 1;
}

void updatebuttonstate() {
  bool buttonstatea = !digitalRead(buttona);
  bool buttonstateb = !digitalRead(buttonb);
  bool buttonstatec = !digitalRead(buttonc);
  buttons[0][0] = (buttons[1][0] < buttonstatea);
  buttons[1][0] = buttonstatea;
  buttons[0][1] = (buttons[1][1] < buttonstateb);
  buttons[1][1] = buttonstateb;
  buttons[0][2] = (buttons[1][2] < buttonstatec);
  buttons[1][2] = buttonstatec;
}

bool getbutton(byte buttonnum) {
  return buttons[0][buttonnum];
}

void Unloadfilament() {
  engage();
  printStatus(F("Retracting"));
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(-bowdentubelength);
  enableMotors();
  extruder.setSpeedInStepsPerSecond(Extslowspeed);

  while (digitalRead(inductiveprobe)) {
    extruder.moveRelativeInSteps(1);
  }
  while (!digitalRead(inductiveprobe)) {
    extruder.moveRelativeInSteps(-1);
  }

  printStatus(F("Retracting from\nSelector"));
  moveExtruder(-distancetoprobe);
  disengage();
  disableMotors();
  LOADED = 0;
}
void Loadfilament() {
  engage();
  printStatus(F("Loading into\nSelector"));
  extruder.setSpeedInStepsPerSecond(Extspeed);
  enableMotors();
  feedtoprobe();
  printStatus(F("Loading to\nToolhead"));
  extruder.setSpeedInStepsPerSecond(Extslowspeed);
  moveExtruder(distancetocoloursensor);
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(bowdentubelength);
  disengage();
  disableMotors();
  LOADED = 1;
}

void Preloadfilament() {
  engage();
  enableMotors();
  printStatus(F("Feeding to Probe"));
  feedtoprobe();
  extruder.setSpeedInStepsPerSecond(Extslowspeed);
  moveExtruder(-distancetoprobe);
  enableMotors();
  feedtoprobe();
  extruder.setSpeedInStepsPerSecond(Extslowspeed);
  moveExtruder(-distancetoprobe);
  disableMotors();
  disengage();
  if (enablecolour) {
    detectcolour(curtool);
    updateleds();
  }
}

void feedtoprobe() {
  while (digitalRead(inductiveprobe)) {
    extruder.moveRelativeInSteps(1);
  }
  while (!digitalRead(inductiveprobe)) {
    extruder.moveRelativeInSteps(-1);
  }
}
void detectcolour(int selectedtool) {
  if (curtool != selectedtool) {
    select(selectedtool);
  }

  engage();
  enableMotors();
  printStatus(F("Loading into\nSelector"));
  extruder.setSpeedInStepsPerSecond(Extslowspeed);
  feedtoprobe();
  printStatus(F("Loading to\ncolour sensor"));
  extruder.setSpeedInStepsPerSecond(Extslowspeed);
  moveExtruder(distancetocoloursensor);
  disableMotors();

  //do colour stuff
  strip.clear();
  strip.show();
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED
  delay(tcsdelay);          // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED

  red = max(red - redoffset, 0);
  green = max(green - greenoffset, 0);
  blue = max(blue - blueoffset, 0);

  colours[0][selectedtool] = byte(red);
  colours[1][selectedtool] = byte(green);
  colours[2][selectedtool] = byte(blue);
  //print colour info to display
  display.clear();
  display.setCursor(0, 0);
  display.print(F("colour Read!\nR:"));
  display.print(colours[0][selectedtool]);
  display.print(F(" G:"));
  display.print(colours[1][selectedtool]);
  display.print(F("\nB:"));
  display.print(colours[2][selectedtool]);
  display.print(F("\nRetracting"));

  //pull filament back out of the selector
  engage();
  enableMotors();
  extruder.moveToHomeInMillimeters(1, 10, distancetocoloursensor + 10, inductiveprobe);
  extruder.setSpeedInStepsPerSecond(Extslowspeed);
  moveExtruder(-distancetoprobe);
  disengage();
  disableMotors();
}

void updateleds() {
  for (int i = 0; i < tools; i++) {
    strip.setPixelColor(i, strip.Color(colours[0][i], colours[1][i], colours[2][i]));
  }
  strip.show();
  updateFRAM();
}
void PreloadALL(int toolcount) {
  for (int i = 0; i < toolcount; i++) {
    select(i);
    Preloadfilament();
  }
  if (FRAMENABLED) {
    updateFRAM();
  }
  menu = MENU_MAIN;
  redraw = 1;
}

void updateFRAM() {
  printStatus(F("updating FRAM\nValues"));
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < tools; i++) {
      fram.write(i + (j * tools), colours[j][i]);
    }
  }
}
void restoreFromFRAM() {
  printStatus(F("restoring FRAM\nValues"));
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < tools; i++) {
      colours[j][i] = fram.read(i + (j * tools));
    }
  }
}

void scani2c() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge at the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done");
  display.clear();
  display.setCursor(0, 0);
  display.print(F("found "));
  display.print(nDevices);
  display.print(F(" device"));
  if (nDevices != 1) {
    display.print(F("s"));
  }
}
void testcolour() {  //do colour stuff
  if (enablecolour) {
    strip.fill(strip.Color(0, 0, 0));
    strip.show();
    float red, green, blue;
    tcs.setInterrupt(false);  // turn on LED
    delay(tcsdelay);          // takes 50ms to illuminate
    tcs.getRGB(&red, &green, &blue);
    tcs.setInterrupt(true);  // turn off LED
    red = max(red - redoffset, 0);
    green = max(green - greenoffset, 0);
    blue = max(blue - blueoffset, 0);
    Serial.println(red);
    Serial.println(green);
    Serial.println(blue);
    //print colour info to display
    display.clear();
    display.setCursor(0, 0);
    display.print(F("colour Read!\nR:"));
    display.print(round(red));
    display.print(F(" G:"));
    display.print(byte(green));
    display.print(F("\nB:"));
    display.print(byte(blue));

    strip.clear();
    strip.fill(strip.Color(byte(red), byte(green), byte(blue)), 0, tools + 1);
    strip.show();
  } else {
    printStatus(F("colour sensor\nnot initialized."));
  }
}

#if useprefeederswitches
void updateswitches() {
  oldprefeedbuttons = prefeedbuttons;
  prefeedbuttons = 0;
  for (int i = 0; i < 15; i++) {
    prefeedbuttons += mcpa.digitalRead(i) << i;
    prefeedbuttons += mcpb.digitalRead(i) << (i + 16);
  }
}

void processprefeedswitches() {
  updateswitches();
  byte detectedtool = detectswitches();
  if (detectedtool == 255) { return; }
  if (LOADED && (curtool == detectedtool)) {
    filamentrunout = 1;
    return;
  }
  if (LOADED == false) {
    select(detectedtool);
    Preloadfilament();
  }
}

byte detectswitches() {
  if (oldprefeedbuttons != prefeedbuttons) {
    uint32_t temp = oldprefeedbuttons ^ prefeedbuttons;
    for (int i = 0; i < 31; i++) {
      if ((temp >> i) & 1) {
        return i;
      }
    }
  }
  return 255;
}
#endif

void processrunout() {
  /*
  1: tell printer we're out of filament
  2:wait for it to finish moving
  3:wait for it to say unload.
  4:unload
  5:detect if spool join

  if spool join
  6:load next filament

  if no spool join
  6:notify somehow? (flash lights on front maybe?)
  */
}

void loadingleds(int lednum, uint32_t lbasecolour, uint32_t colour) {
  strip.fill(lbasecolour);
  strip.setPixelColor(lednum, colour);
  strip.show();
}

bool tryaccel(long acceleration) {
  delay(15);
  Serial.print(F("\ntrying accel:"));
  Serial.print(acceleration);
  selector.setAccelerationInStepsPerSecondPerSecond(acceleration);
  selector.moveToPositionInMillimeters((maxpos / 2) - 10);
  selector.setAccelerationInStepsPerSecondPerSecond(accel);
  if (selector.moveToHomeInMillimeters(1, homespeed, (maxpos / 2) - 11, endstop)) {
    return false;
  } else {
    if (!selector.moveToHomeInMillimeters(1, homespeed, maxpos / 2, endstop)) { STOP(); }
    selector.setAccelerationInStepsPerSecondPerSecond(acceleration);
    selector.moveToPositionInMillimeters((maxpos / 2) - 10);
    selector.setAccelerationInStepsPerSecondPerSecond(accel);
    if (selector.moveToHomeInMillimeters(1, homespeed, (maxpos / 2) - 11, endstop)) {
      return false;
    } else {
      if (!selector.moveToHomeInMillimeters(1, homespeed, maxpos / 2, endstop)) { STOP(); }
      return true;
    }
  }
}

long getmaxaccel(long inc) {
  homeSelector();
  long low = inc;
  long high = inc;

  enableMotors();

  while (tryaccel(high)) {
    low = high;
    high <<= 1;
  }

  while ((high - low) > 2) {
    long mid = low + ((high - low) >> 1);

    if (tryaccel(mid)) {
      low = mid;
    } else {
      high = mid;
    }
  }
  disableMotors();

  Serial.print(F("\nmax acceleration found:"));
  Serial.print(low);
  Serial.println(F("steps/s/s"));
  return low;
}

bool tryspeed(long speed) {
  delay(15);
  Serial.print(F("\ntrying speed:"));
  Serial.print(speed);
  selector.setSpeedInStepsPerSecond(speed);
  selector.moveToPositionInMillimeters((maxpos / 2) - 10);
  selector.setSpeedInStepsPerSecond(Selspeed);
  if (selector.moveToHomeInMillimeters(1, homespeed, (maxpos / 2) - 11, endstop)) {
    return false;
  } else {
    if (!selector.moveToHomeInMillimeters(1, homespeed, maxpos / 2, endstop)) { STOP(); }
    selector.setSpeedInStepsPerSecond(speed);
    selector.moveToPositionInMillimeters((maxpos / 2) - 10);
    selector.setSpeedInStepsPerSecond(Selspeed);
    if (selector.moveToHomeInMillimeters(1, homespeed, (maxpos / 2) - 11, endstop)) {
      return false;
    } else {
      if (!selector.moveToHomeInMillimeters(1, homespeed, maxpos / 2, endstop)) { STOP(); }
      return true;
    }
  }
}

long getmaxspeed(long inc) {
  homeSelector();
  long low = inc;
  long high = inc;

  enableMotors();

  while (tryspeed(high)) {
    low = high;
    high <<= 1;
  }

  while ((high - low) > 2) {
    long mid = low + ((high - low) >> 1);

    if (tryspeed(mid)) {
      low = mid;
    } else {
      high = mid;
    }
  }
  disableMotors();

  Serial.print(F("\nmax speed found:"));
  Serial.print(low);
  Serial.println(F("steps/s"));
  return low;
}
