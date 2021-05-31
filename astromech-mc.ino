// =======================================================================================
//       Sketch: SyRenSabertoothToBTS7960MotorController
//       Author: Kenneth Ripple (KenRip on Astromech.net)
//         Date: May 31 2021
// =======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it for
//         your personal use and the personal use of other astromech club members.  
//
//         This program is distributed in the hope that it will be useful 
//         as a courtesy to fellow astromech club members wanting to develop
//         their own droid control system.
//
//         IT IS OFFERED WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//         You are using this software at your own risk and, as a fellow club member, it is
//         expected you will have the proper experience / background to handle and manage that 
//         risk appropriately.  It is completely up to you to insure the safe operation of
//         your droid and to validate and test all aspects of your droid control system.
//
// =======================================================================================
//
//  Description: This sketch is designed to emulate the Dimension Engineering SyRen 10 and Sabertooth 2x32 motor
//               controllers.  It is designed for an Arduino Uno (or compatible) with at least 6 pins capable for 
//               PWM.  It utilizes three inexpensive BTS7960 motor controllers each theoretically capable of 43A 12VDC 
//               loads to drive the Dome, Left and Right foot motors in an Astromech droid.  It responds to the 
//               packet serial commands sent from the Dimension Engineering Sabertooth library used in the SHADOW,
//               SHADOW MD or Padawan controller sketches.  This version of the code has been tested with the SHADOW MD
//               sketch but should be compatible with the SHADOW and Padawan based sketches as these all appear to
//               utilize the same Sabertooth motor controller logic.  The sketch uses no external libraries to drive
//               the BTS7960 motor drivers as these are simple to control with PWM signals.  Signal from the SHADOW
//               sketch running on the Arduino Mega is carried from serial TX2 pin (pin 16) to the serial RX0 (pin 0)
//               on the Arduino Uno running this sketch.
// 
//               These are the BTS7960 modules I used...
//               https://www.amazon.com/BTS7960-Driver-Module-Arduino-Current/dp/B091TPZ9R1
//               US price for three (3) was $18.39.
//               Other compatible units can be found on Amazon or on AliExpress...
//               https://www.aliexpress.com/wholesale?SearchText=BTS7960
//
// =======================================================================================

//#define SERIAL_DEBUG    // Uncomment to display Serial debugging messages
// Note: Serial debugging significantly degrades the actual serial communications between the SHADOW/Padawn Arduino Mega
//       and this controller.  You should compile your final version of this sketch with SERIAL_DEBUG commented out to
//       avoid this performance impact.
//       CAUTION: If you run your Astromech with SERIAL_DEBUG enabled the droid may respond very sluggishly with very
//                delayed response times.  This could case your droid to run it's foot motors for unexpected movements
//                and not stop in a timely fashion.

// Variables
int turnleftVal;
int turnrightVal;
int driveleftVal;
int driverightVal;

// Serial Communications values for receiving SyRen and Sabertooth Packet Serial Commands
uint8_t serialdata[64]; // Initialized variable to store recieved data
int datareceived;
int i;
int d;
byte CMD;
int VAL;
int CHK;
byte ChkSum;
int serialtimeout = 10; // Number of loop iterations that if no serial data received then drive motors will stop
int loopdelay = 50;     // Millisecond delay per loop iteration
int timeout;

// Constants to define the SyRen and Sabertooth Packet Serial Command values
// Note: Based on the legacy packet serial protocol documented in the Dimension Engineering
//       Sabertooth 2x10 datasheet found here -> https://www.dimensionengineering.com/datasheets/Sabertooth2x10.pdf
const byte autobaud = 0xAA;       // bauding character decimal 170 (0xAA). Sent before any other commands are sent
const byte addr128 = 0x80;        // Addr 128 = Sabertooth  (as set in the Shadow.ino source)
const byte addr129 = 0x81;        // Addr 129 = SyRen       (as set in the Shadow.ino source)
const byte motor1forward = 0x00;  // 0: Drive forward motor 1. Valid data value 0-127 for off to full power
const byte motor1reverse = 0x01;  // 1: Drive backwards motor 1. Valid data value 0-127 for off to full power
const byte minvoltage = 0x02;     // 2: Min voltage. Not implemented in this sketch
const byte maxvoltage = 0x03;     // 3: Max voltage. Not implemented in this sketch
const byte motor2forward = 0x04;  // 4: Drive forward motor 2. Valid data value 0-127 for off to full power
const byte motor2reverse = 0x05;  // 5: Drive backwards motor 2. Valid data value 0-127 for off to full power
const byte motor17bit = 0x06;     // 6: Drive motor 1 7 bit. Not implemented in this sketch
const byte motor27bit = 0x07;     // 7: Drive motor 2 7 bit. Not implemented in this sketch
const byte drivereverse = 0x08;   // 8: Drive forward mixed mode. Valid data value 0-127 for off to full drive forward
const byte driveforward = 0x09;   // 9: Drive backwards mixed mode. Valid data value 0-127 for off to full drive backwards
const byte turnleft = 0x0A;       // 10: Turn right mixed mode. Valid data value 0-127 for zero to maximum turning speed
const byte turnright = 0x0B;      // 11: Turn leftt mixed mode. Valid data value 0-127 for zero to maximum turning speed
const byte settimeout = 0x0E;     // No documentation found for this command.  Generated by ST->setTimeout statement in Shadow.ino.  Not implemented in this sketch
const byte setdeadband = 0x11;    // No documentation found for this command.  Generated by ST->setDeadband statement in Shadow.ino.  Not implemented in this sketch

// Values to monitor motor state
boolean isSTMotorsStopped = true;
boolean isSyRMotorStopped = true;

// Setup variables used for BTS7960 Motor Control
int RightFoot_R_IS = 2;
int RightFoot_R_EN = 4;
int RightFoot_R_PWM = 3;
int RightFoot_L_IS = 7;
int RightFoot_L_EN = 8;
int RightFoot_L_PWM = 5;
int LeftFoot_R_IS = 12;
int LeftFoot_R_EN = 13;
int LeftFoot_R_PWM = 6;
int LeftFoot_L_IS = 14; // A0
int LeftFoot_L_EN = 15; // A1
int LeftFoot_L_PWM = 9;
int Dome_R_IS = 16; // A2
int Dome_R_EN = 17; // A3
int Dome_R_PWM = 10;
int Dome_L_IS = 18; // A4
int Dome_L_EN = 19; // A5
int Dome_L_PWM = 11;

void setup() {
  Serial.begin(9600);
  timeout = 0;
#ifdef SERIAL_DEBUG
  Serial.println("Setup BTS7960 Output Pins");
#endif

  // Setup BTS7960 output pins (RightFoot)
  pinMode(RightFoot_R_IS, OUTPUT);
  pinMode(RightFoot_R_EN, OUTPUT);
  pinMode(RightFoot_R_PWM, OUTPUT);
  pinMode(RightFoot_L_IS, OUTPUT);
  pinMode(RightFoot_L_EN, OUTPUT);
  pinMode(RightFoot_L_PWM, OUTPUT);
  digitalWrite(RightFoot_R_IS, LOW);
  digitalWrite(RightFoot_L_IS, LOW);
  digitalWrite(RightFoot_R_EN, HIGH);
  digitalWrite(RightFoot_L_EN, HIGH);

  // Setup BTS7960 output pins (LeftFoot)
  pinMode(LeftFoot_R_IS, OUTPUT);
  pinMode(LeftFoot_R_EN, OUTPUT);
  pinMode(LeftFoot_R_PWM, OUTPUT);
  pinMode(LeftFoot_L_IS, OUTPUT);
  pinMode(LeftFoot_L_EN, OUTPUT);
  pinMode(LeftFoot_L_PWM, OUTPUT);
  digitalWrite(LeftFoot_R_IS, LOW);
  digitalWrite(LeftFoot_L_IS, LOW);
  digitalWrite(LeftFoot_R_EN, HIGH);
  digitalWrite(LeftFoot_L_EN, HIGH);

  // Setup BTS7960 output pins (Dome)
  pinMode(Dome_R_IS, OUTPUT);
  pinMode(Dome_R_EN, OUTPUT);
  pinMode(Dome_R_PWM, OUTPUT);
  pinMode(Dome_L_IS, OUTPUT);
  pinMode(Dome_L_EN, OUTPUT);
  pinMode(Dome_L_PWM, OUTPUT);
  digitalWrite(Dome_R_IS, LOW);
  digitalWrite(Dome_L_IS, LOW);
  digitalWrite(Dome_R_EN, HIGH);
  digitalWrite(Dome_L_EN, HIGH);
}

void loop() {
  datareceived = Serial.available();
  if (datareceived != 0) {
    Serial.readBytes(serialdata, datareceived); // Read all bytes currently received
    timeout = 0; // Reset timeout counter after reading bytes
    for (d = 0; d < datareceived; d++) {
#ifdef SERIAL_DEBUG
      Serial.print("Addr: ");
      printHex(serialdata[d]);
#endif
      switch (serialdata[d]) {

        // Sabertooth / Foot Motor Operation Section **********************************************************************
        case addr128:
          for (i = 1; i < 4; i++) {
            if (i == 1) {
#ifdef SERIAL_DEBUG
              Serial.print("ST Cmd: ");
              printHex(serialdata[d + i]);
#endif
              if (serialdata[d + i] == settimeout) {
#ifdef SERIAL_DEBUG
                Serial.println("  setTimeout Cmd");
#endif
                CMD = serialdata[d + i];
              }
              if (serialdata[d + i] == setdeadband) {
                Serial.println("  setDeadband Cmd");
                CMD = serialdata[d + i];
              }
              if (serialdata[d + i] == motor1forward) {
                Serial.println("  motor1forward Cmd");
                CMD = serialdata[d + i];
                isSTMotorsStopped = false;
              }
              if (serialdata[d + i] == motor1reverse) {
                Serial.println("  motor1reverse Cmd");
                CMD = serialdata[d + i];
                isSTMotorsStopped = false;
              }
              if (serialdata[d + i] == motor2forward) {
                Serial.println("  motor2forward Cmd");
                CMD = serialdata[d + i];
                isSTMotorsStopped = false;
              }
              if (serialdata[d + i] == motor2reverse) {
                Serial.println("  motor2reverse Cmd");
                CMD = serialdata[d + i];
                isSTMotorsStopped = false;
              }
              if (serialdata[d + i] == turnleft) {
                Serial.println("  turnleft Cmd");
                CMD = serialdata[d + i];
              }
              if (serialdata[d + i] == driveforward) {
                Serial.println("  driveforward Cmd");
                CMD = serialdata[d + i];
                isSTMotorsStopped = false;
              }
              if (serialdata[d + i] == turnright) {
                Serial.println("  turnright Cmd");
                CMD = serialdata[d + i];
              }
              if (serialdata[d + i] == drivereverse) {
                Serial.println("  drivereverse Cmd");
                CMD = serialdata[d + i];
                isSTMotorsStopped = false;
              }
            }
            if (i == 2) {
#ifdef SERIAL_DEBUG
              Serial.print("  Val: ");
              printHex(serialdata[d + i]);
#endif
              VAL = serialdata[d + i];
            }
            if (i == 3) {
#ifdef SERIAL_DEBUG
              Serial.print("  ChkSum: ");
              printHex(serialdata[d + i]);
#endif
              // Check of ChkSum here before executing movement commands
              ChkSum = ((addr128 + CMD + VAL) & B01111111);
#ifdef SERIAL_DEBUG
              Serial.print("  Calc ChkSum: ");
              printHex(ChkSum);
#endif
              if (serialdata[d + i] == ChkSum) {
                if (CMD == motor1forward) {
#ifdef SERIAL_DEBUG
                  Serial.println("  RightFoot Clockwise");
                  Serial.print("    RightFoot_R_PWM: ");
                  printHex(VAL);
                  Serial.println("    RightFoot_L_PWM: 00");
#endif
                  analogWrite(RightFoot_R_PWM, VAL);
                  analogWrite(RightFoot_L_PWM, 0);
                  if (VAL == 0) {
#ifdef SERIAL_DEBUG
                    Serial.println("  RightFoot Stopped");
#endif
                  }
                }
                if (CMD == motor1reverse) {
#ifdef SERIAL_DEBUG
                  Serial.println("  RightFoot Counter-Clockwise");
                  Serial.println("    RightFoot_R_PWM: 00");
                  Serial.print("    RightFoot_L_PWM: ");
                  printHex(VAL);
#endif
                  analogWrite(RightFoot_R_PWM, 0);
                  analogWrite(RightFoot_L_PWM, VAL);
                  if (VAL == 0) {
#ifdef SERIAL_DEBUG
                    Serial.println("  RightFoot Stopped");
#endif
                  }
                }
                if (CMD == motor2forward) {
#ifdef SERIAL_DEBUG
                  Serial.println("  LeftFoot Clockwise");
                  Serial.print("    LeftFoot_R_PWM: ");
                  printHex(VAL);
                  Serial.println("    LeftFoot_L_PWM: 00");
#endif
                  analogWrite(LeftFoot_R_PWM, VAL);
                  analogWrite(LeftFoot_L_PWM, 0);
                  if (VAL == 0) {
#ifdef SERIAL_DEBUG
                    Serial.println("  LeftFoot Stopped");
#endif
                  }
                }
                if (CMD == motor2reverse) {
#ifdef SERIAL_DEBUG
                  Serial.println("  LeftFoot Counter-Clockwise");
                  Serial.println("    LeftFoot_R_PWM: 00");
                  Serial.print("    LeftFoot_L_PWM: ");
                  printHex(VAL);
#endif
                  analogWrite(LeftFoot_R_PWM, 0);
                  analogWrite(LeftFoot_L_PWM, VAL);
                  if (VAL == 0) {
#ifdef SERIAL_DEBUG
                    Serial.println("  LeftFoot Stopped");
#endif
                  }
                }
                if (CMD == turnleft) {
#ifdef SERIAL_DEBUG
                  Serial.println("  Saving turnleft Value");
                  Serial.print("    Turnleft VAL: ");
                  printHex(VAL);
#endif
                  turnleftVal = VAL;
                  turnrightVal = 0;
                }
                if (CMD == turnright) {
#ifdef SERIAL_DEBUG
                  Serial.println("  Saving turnright Value");
                  Serial.print("    Turnright VAL: ");
                  printHex(VAL);
#endif
                  turnrightVal = VAL;
                  turnleftVal = 0;
                }
                if (CMD == driveforward) {
#ifdef SERIAL_DEBUG
                  Serial.println("  DriveForward Movement");
                  Serial.print("    DriveForward VAL: ");
                  printHex(VAL);
                  Serial.print("    Turnright VAL: ");
                  printHex(turnrightVal);
                  Serial.print("    Turnleft VAL: ");
                  printHex(turnleftVal);
#endif
                  driverightVal = VAL + turnleftVal;
                  driveleftVal = VAL + turnrightVal;
#ifdef SERIAL_DEBUG
                  Serial.print("    RightFoot DriveForward VAL: ");
                  printHex(driverightVal);
                  Serial.print("    LeftFoot DriveForward VAL: ");
                  printHex(driveleftVal);
#endif
                  analogWrite(RightFoot_R_PWM, VAL);
                  analogWrite(RightFoot_L_PWM, 0);
                  analogWrite(LeftFoot_R_PWM, VAL);
                  analogWrite(LeftFoot_L_PWM, 0);
                }
                if (CMD == drivereverse) {
#ifdef SERIAL_DEBUG
                  Serial.println("  DriveReverse Movement");
                  Serial.print("    DriveReverse VAL: ");
                  printHex(VAL);
                  Serial.print("    Turnright VAL: ");
                  printHex(turnrightVal);
                  Serial.print("    Turnleft VAL: ");
                  printHex(turnleftVal);
#endif
                  driverightVal = VAL + turnleftVal;
                  driveleftVal = VAL + turnrightVal;
#ifdef SERIAL_DEBUG
                  Serial.print("    RightFoot DriveReverse VAL: ");
                  printHex(driverightVal);
                  Serial.print("    LedftFoot DriveReverse VAL: ");
                  printHex(driveleftVal);
#endif
                  analogWrite(RightFoot_R_PWM, 0);
                  analogWrite(RightFoot_L_PWM, VAL);
                  analogWrite(LeftFoot_R_PWM, 0);
                  analogWrite(LeftFoot_L_PWM, VAL);
                }
              }
              else {
#ifdef SERIAL_DEBUG
                Serial.println("  ChkSum Error/Cmd Ignored");
#endif
              }
            }
          }
          d = d + 3;
          break;

        // SyRen / Dome Operation Section *********************************************************************************
        case addr129:
          for (i = 1; i < 4; i++) {
            if (i == 1) {
#ifdef SERIAL_DEBUG
              Serial.print("SyRen Cmd: ");
              printHex(serialdata[d + i]);
#endif
              if (serialdata[d + i] == settimeout) {
#ifdef SERIAL_DEBUG
                Serial.println("  setTimeout Cmd");
#endif
                CMD = serialdata[d + i];
              }
              if (serialdata[d + i] == setdeadband) {
#ifdef SERIAL_DEBUG
                Serial.println("  setDeadband Cmd");
#endif
                CMD = serialdata[d + i];
              }
              if (serialdata[d + i] == motor1forward) {
#ifdef SERIAL_DEBUG
                Serial.println("  motor1forward Cmd");
#endif
                CMD = serialdata[d + i];
                isSyRMotorStopped = false;
              }
              if (serialdata[d + i] == motor1reverse) {
#ifdef SERIAL_DEBUG
                Serial.println("  motor1reverse Cmd");
#endif
                CMD = serialdata[d + i];
                isSyRMotorStopped = false;
              }
              // Note: Because Shadow.ino uses Sabertooth class for SyRen 10 it
              //       generates motor2 commands when the SyR->stop command is issued.
              if (serialdata[d + i] == motor2forward) {
#ifdef SERIAL_DEBUG
                Serial.println("  motor2forward Cmd");
#endif
                CMD = serialdata[d + i];
                isSyRMotorStopped = false;
              }
              if (serialdata[d + i] == motor2reverse) {
#ifdef SERIAL_DEBUG
                Serial.println("  motor2reverse Cmd");
#endif
                CMD = serialdata[d + i];
                isSyRMotorStopped = false;
              }
            }
            if (i == 2) {
#ifdef SERIAL_DEBUG
              Serial.print("  Val: ");
              printHex(serialdata[d + i]);
#endif
              VAL = serialdata[d + i];
            }
            if (i == 3) {
#ifdef SERIAL_DEBUG
              Serial.print("  ChkSum: ");
              printHex(serialdata[d + i]);
#endif
              // Check of ChkSum here before executing movement commands
              ChkSum = ((addr129 + CMD + VAL) & B01111111);
#ifdef SERIAL_DEBUG
              Serial.print("  Calc ChkSum: ");
              printHex(ChkSum);
#endif
              if (serialdata[d + i] == ChkSum) {
                if (CMD == motor1forward) {
#ifdef SERIAL_DEBUG
                  Serial.println("  Dome Clockwise");
#endif
                  analogWrite(Dome_R_PWM, VAL);
                  analogWrite(Dome_L_PWM, 0);
                  if (VAL == 0) {
#ifdef SERIAL_DEBUG
                    Serial.println("  Dome Stopped");
#endif
                    isSyRMotorStopped = true;
                  }
                }
                if (CMD == motor1reverse) {
#ifdef SERIAL_DEBUG
                  Serial.println("  Dome Counter-Clockwise");
#endif
                  analogWrite(Dome_R_PWM, 0);
                  analogWrite(Dome_L_PWM, VAL);
                  if (VAL == 0) {
#ifdef SERIAL_DEBUG
                    Serial.println("  Dome Stopped");
#endif
                    isSyRMotorStopped = true;
                  }
                }
                if (CMD == motor2forward) {
#ifdef SERIAL_DEBUG
                  Serial.println("  motor2forward Cmd");
#endif
                  if (VAL == 0) {
#ifdef SERIAL_DEBUG
                    Serial.println("  Dome Stopped");
#endif
                    isSyRMotorStopped = true;
                  }
                }
                if (CMD == motor2reverse) {
#ifdef SERIAL_DEBUG
                  Serial.println("  motor2reverse Cmd");
#endif
                  if (VAL == 0) {
#ifdef SERIAL_DEBUG
                    Serial.println("  Dome Stopped");
#endif
                    isSyRMotorStopped = true;
                  }
                }
              }
              else {
#ifdef SERIAL_DEBUG
                Serial.println("  ChkSum Error/Cmd Ignored");
#endif
              }
            }
          }
          d = d + 3;
          break;
        case autobaud:
#ifdef SERIAL_DEBUG
          Serial.println("ST/SyRen Autobaud Cmd");
#endif
          break;
        default:
          break;
      }
    }
  }
  else {
    timeout = timeout + 1;
    if (timeout >= serialtimeout) {
      if (isSTMotorsStopped == false) {
#ifdef SERIAL_DEBUG
        Serial.println("ST: Serial timeout - motors stopped");
        Serial.println("    LeftFoot_R_PWM: 00");
        Serial.println("    LeftFoot_L_PWM: 00");
        Serial.println("    RightFoot_R_PWM: 00");
        Serial.println("    RightFoot_L_PWM: 00");
#endif
        isSTMotorsStopped = true;
        analogWrite(LeftFoot_R_PWM, 0);
        analogWrite(LeftFoot_L_PWM, 0);
        analogWrite(RightFoot_R_PWM, 0);
        analogWrite(RightFoot_L_PWM, 0);
      }
      if (isSyRMotorStopped == false) {
#ifdef SERIAL_DEBUG
        Serial.println("SyRen: Serial timeout - motor stopped");
        Serial.println("    Dome_R_PWM: 00");
        Serial.println("    Dome_L_PWM: 00");
#endif
        isSyRMotorStopped = true;
        analogWrite(Dome_R_PWM, 0);
        analogWrite(Dome_L_PWM, 0);
      }
      timeout = 0;
    }
  }
  delay(loopdelay);
}

// Routine for printing single byte values in Hex to Serial for debugging
void printHex(uint8_t num) {
  char hexCar[2];
  sprintf(hexCar, "%02X", num);
  Serial.println(hexCar);
}
