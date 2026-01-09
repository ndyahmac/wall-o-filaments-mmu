//FRAM storage stuff
#include <Adafruit_EEPROM_I2C.h>
#include <Adafruit_FRAM_I2C.h>
//colour sensor
#include <Adafruit_TCS34725.h>
//neopixel library
#include <Adafruit_NeoPixel.h>
//motor libraries
#include <SpeedyStepper.h>
#include <Servo.h>
//oled display stuff
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//software reset!!
#include <SoftwareReset.hpp>

#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

//config
#define useserial true    //enable the Serial interface
#define serialspeed 9600  //baud rate of serial interface, default is 9600
//allowable baud rates
//300,600,750,1200,2400,4800,9600,19200,31250,38400,57600,74880,115200,230400,250000,460800,500000,921600,1000000,2000000
//typical baud rates
//9600,19200,115200

#define WIRE Wire  //for the i2c scanning function

#define LINE_BUF 96
//pin definitions
#define seldirPin 4        //direction pin of selector motor driver
#define selstepPin 5       //step pin of the driver
#define extdirPin 7        //direction pin of extruder motor driver
#define extstepPin 8       //step pin of the driver
#define endstop 6          //pin of the endstop for the selector
#define servopin 3         //selector servo pin
#define enable_pin 2       //pin to enable/disable the stepper drivers
#define inductiveprobe 12  //pin for the inductive probe for sensing filament
#define LED_STRIP_PIN 11   //pin for the led strip across the front
#define psuEnablePin 10    //pin used the enable the power supply for motors/probe

//button input pins
#define buttona A0
#define buttonb A1
#define buttonc A2

//motor characteristis
//24v
// #define Selspeed 9000   //max speed in steps/s of the selector motor
// #define Extspeed 12000  //max speed in steps/s of the extrudor motor
// #define accel 75000     //acceleration of both motors in steps/s2
//12v
#define Selspeed 4000   //max speed in steps/s of the selector motor
#define Extspeed 8000  //max speed in steps/s of the extrudor motor
#define accel 50000     //acceleration of both motors in steps/s2

//physical hardware details
#define firstpos 7   //mm from 0 where the first filament resides
#define increment 8  //increment in mm in between each consecutive filament slot
#define tools 29     //one less than the total filament slots
#define maxpos 240   //maximum position of the selector in mm
#define mindistfromhome 5

//selector definitions
#define distancetoprobe 25         //the distance at which the filament has to move from the probe to not collide with the selector.
#define distancetocoloursensor 10  //the distance past the prob activation that the filament has to be pushed for the colour snesor to detect it.
#define bowdentubelength 550       //length in mm of the bowden tube to the extruder on the tool head.
#define servoengagedpos 50         //position for the servo motor to engage the filament
#define servodisengagedpos 80      //safe position away from the filament for the servo motor to spin to.

//init fram object
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
//gcode parser stuff (entirely so you can send a command and have it work)
struct GCode {
  bool hasG, hasM, hasT, hasS, hasX, hasE;
  int G, M, T, S, X, E;
};
GCode gc;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
//defining motors
SpeedyStepper selector;
SpeedyStepper extruder;
Servo servo;

//defning the colour stuff (oohh shiny)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);  //colour sensor
Adafruit_NeoPixel strip(tools + 1, LED_STRIP_PIN, NEO_RGB + NEO_KHZ800);
bool enablecolour = 1;
byte colours[tools][3];  //array to store the colour values(rgb for each tool)

//global variable definitions
bool engaged = 0;
int curtool = 0;
bool startchecksran = 0;
bool LOADED = 0;
bool FRAMENABLED = 0;

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

//start of actual code
void setup() {
  //enable the psu
  pinMode(psuEnablePin, OUTPUT);
  digitalWrite(psuEnablePin, HIGH);

  if (fram.begin()) {  //init the FRAM, if it's installed, if it's not detected, the program will keep going.
    printStatus(F("Found FRAM\nWill save values"));
    FRAMENABLED = 1;
  } else {
    printStatus(F("Could NOT find FRAM\nvalues will not\nbe saved"));
  }
  strip.begin();  // INITIALIZE NeoPixel strip object
  strip.show();   // Turn OFF all pixels ASAP
  strip.setBrightness(50);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, 1, 1);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing"));
  display.display();

  pinMode(buttona, INPUT_PULLUP);
  pinMode(buttonb, INPUT_PULLUP);
  pinMode(buttonc, INPUT_PULLUP);

  pinMode(enable_pin, OUTPUT);
  pinMode(endstop, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(inductiveprobe, INPUT_PULLUP);

  disableMotors();

  servo.attach(servopin);
  servo.write(180);

  selector.connectToPins(selstepPin, seldirPin);

  selector.setSpeedInStepsPerSecond(Selspeed);
  selector.setAccelerationInStepsPerSecondPerSecond(accel);
  selector.setStepsPerMillimeter(50);

  extruder.connectToPins(extstepPin, extdirPin);

  extruder.setSpeedInStepsPerSecond(Extspeed);
  extruder.setAccelerationInStepsPerSecondPerSecond(accel);
  extruder.setStepsPerMillimeter(17.3);  //change and calibrate later, this is a temporary value and needs to be measured

#if useserial
  Serial.begin(serialspeed);
#endif
  if (tcs.begin()) {
    printStatus(F("Found colour\nsensor!"));
    enablecolour = 1;
  } else {
    printStatus(F("Colour sensor\nnot found\ndisabling colour."));
    enablecolour = 0;
    delay(5000);
  }
  if (FRAMENABLED) {
    restoreFromFRAM();
    if (enablecolour) {
      updateleds();
    }
  }
  printStatus(F("initialized"));
}

//main loop
void loop() {
  if (startchecksran == 0) {
    if (!digitalRead(inductiveprobe)) {
      runstartupchecks();
    }
  }
  updatebuttonstate();
  PROCESSMENU();
  drawMenu();

#if useserial
  processSerial();
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
/*
G48  test X/E at speed
G0   move x,e
G28  home x
M701 Tn	Load filament n
M702 Tn	Unload filament n
M704 Preload
M709 Reset MMU
M705 report tool
M706 MMU self-test
M707 Sn	Set selector position directly
M708 Kill MMU
M710 scan i2c
M711 drain PSU
*/
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
      case 0:
        if (gc.hasE) {
          moveExtruder(gc.E);
        }
        if (gc.hasX) {
          enableMotors();
          selector.moveToPositionInMillimeters(gc.X);
          disableMotors();
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
        } else Serial.println(F("error: T missing"));
        return;

      case 702:
        if (gc.hasT) {
          Unloadfilament();
        } else Serial.println(F("error: T missing"));
        return;
      case 704:
        if (gc.hasT) {
          select(gc.T);
          Preloadfilament();
        } else Serial.println(F("error: T missing"));
        return;
      case 705:
        Serial.print(F("T"));
        Serial.println(curtool);
        return;
      case 706:
        startchecksran = 0;
        runstartupchecks();
        return;
      case 707:
        if (gc.hasT) {
          select(gc.T);
        } else Serial.println(F("error: T missing"));
        return;
      case 708:
        STOP();
        return;
      case 709:
        Serial.println(F("resetting..."));
        printStatus(F("resetting..."));
        softwareReset::standard();
        return;
      case 710:
        scani2c();
        return;
      case 711:
        printStatus(F("Draining residual\npower from PSU"));\
        enableMotors();
        while (selector.moveToHomeInMillimeters(1, 10, 248, endstop) == true) {}
        disableMotors();
        return;
    }
  }
  Serial.println(F("error: unknown command"));
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
      menu = cursor;
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

  display.clearDisplay();
  display.setCursor(0, 0);

  switch (menu) {
    case MENU_MAIN:
      display.println(cursor == 1 ? F("> Select Tool") : F("  Select Tool"));
      display.println(cursor == 2 ? F("> Preload Current") : F("  Preload Current"));
      if (LOADED == 1) {
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
  display.display();
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
  printStatus(F("MMU STOPPED"));
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
}

void printStatus(const __FlashStringHelper status[]) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(status);
  display.display();
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
  display.clearDisplay();
  display.setCursor(0, 0);
  enableMotors();
  if ((tool > tools) + (tool < 0)) {
    display.println(F("Tool Selection\nOut Of Range"));
    display.display();
  } else {
    display.print(F("switching tool\nT"));
    display.print(curtool);
    display.print(F(" -> T"));
    display.print(tool);
    display.display();
    if (engaged == false) {
      selector.moveToPositionInMillimeters(firstpos + (increment * min(max(tool, 0), tools)));
    }
    curtool = tool;
  }
  disableMotors();
}

void homeSelector() {
  printStatus(F("Homing Selector"));
  servo.write(110);
  enableMotors();
  if (selector.moveToHomeInMillimeters(1, 10, 248, endstop) != true) {
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
  homeSelector();
  printStatus(F("Testing Selector"));
  enableMotors();
  selector.moveToPositionInMillimeters(maxpos);
  printStatus(F("Testing Extruder"));
  moveExtruder(200);
  moveExtruder(-200);
  printStatus(F("Testing Servo"));
  engage();
  delay(250);
  disengage();
  startchecksran = 1;
  select(0);
  disableMotors();
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
  enableMotors();
  extruder.moveToHomeInMillimeters(-1, 10, bowdentubelength + 50, inductiveprobe);
  printStatus(F("Retracting from\nSelector"));
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(-distancetoprobe);
  disengage();
}
void Loadfilament() {
  engage();
  printStatus(F("Loading into\nSelector"));
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(distancetoprobe);
  enableMotors();
  extruder.moveToHomeInMillimeters(1, 10, 25, inductiveprobe);
  printStatus(F("Loading to\nToolhead"));
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(bowdentubelength);
  disengage();
}

void Preloadfilament() {
  engage();
  enableMotors();
  printStatus(F("Feeding to Probe"));
  extruder.moveToHomeInMillimeters(-1, 5, 32000, inductiveprobe);
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(-distancetoprobe);
  enableMotors();
  extruder.moveToHomeInMillimeters(-1, 5, 40, inductiveprobe);
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(-distancetoprobe);
  disableMotors();
  disengage();
  if (enablecolour) {
    detectcolour(curtool);
    updateleds();
  }
}

void detectcolour(int selectedtool) {
  if (curtool != selectedtool) {
    select(selectedtool);
  }

  engage();
  printStatus(F("Loading into\nSelector"));
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(distancetoprobe);
  extruder.moveToHomeInMillimeters(1, 10, 25, inductiveprobe);
  printStatus(F("Loading to\ncolour sensor"));
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(distancetocoloursensor);

  //do colour stuff
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED
  delay(60);                // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED
  colours[0][selectedtool] = byte(red);
  colours[1][selectedtool] = byte(green);
  colours[2][selectedtool] = byte(blue);
  //print colour info to display
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(F("colour Read!\nR:"));
  display.print(colours[0][selectedtool]);
  display.print(F(" G:"));
  display.print(colours[1][selectedtool]);
  display.print(F("\nB:"));
  display.print(colours[2][selectedtool]);
  display.print(F("\nRetracting"));
  display.display();
  //pull filament back out of the selector
  engage();
  enableMotors();
  extruder.moveToHomeInMillimeters(-1, 10, distancetocoloursensor + 10, inductiveprobe);
  extruder.setSpeedInStepsPerSecond(Extspeed);
  moveExtruder(-distancetoprobe);
  disengage();
}

void updateleds() {
  for (int i = 0; i < tools; i++) {
    strip.setPixelColor(i, strip.gamma32(strip.Color(colours[0][i], colours[1][i], colours[2][i])));
  }
}
void PreloadALL(int toolcount) {
  for (int i = 0; i < toolcount; i++) {
    select(i);
    Preloadfilament();
    if (enablecolour) {
      detectcolour(i);
      updateleds();
    }
  }
  if (FRAMENABLED) {
    updateFRAM();
  }
  menu = MENU_MAIN;
  redraw = 1;
}

void updateFRAM() {
  printStatus(F("updating FRAM\nValues"));
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < tools; i++) {
      fram.write(i + (j * tools), colours[j][i]);
    }
  }
}
void restoreFromFRAM() {
  printStatus(F("restoring FRAM\nValues"));
  for (int j = 0; j < 2; j++) {
    for (int i = 0; i < tools; i++) {
      colours[j][i] = fram.read(i + (j * tools));
    }
  }
}

void scani2c() {
  byte error, address;
  int nDevices;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("scanning at address"));
  display.drawRect(0, 20, 127, 6, 1);
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge at the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();
    display.setCursor(0, 8);
    display.drawRect(0, 8, 16, 8, 0);
    display.drawRect(1, 9, 14, 6, 0);
    display.drawRect(2, 10, 12, 4, 0);
    display.drawRect(3, 11, 10, 2, 0);
    display.print(address);
    display.drawRect(0, 21, address, 4, 1);
    display.drawRect(0, 22, address, 2, 1);
    display.display();
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
    Serial.println("done\n");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(F("found "));
  display.print(nDevices);
  if (nDevices == 1) {
    display.print(F(" device"));
  } else {
    display.print(F(" devices"));
  }
  display.display();
}
