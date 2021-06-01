# astromech-mc
      Sketch: astromech-mc (the Astromech Motor Controller)
      Author: Kenneth Ripple (KenRip on Astromech.net)
        Date: May 31 2021
Description: 
This sketch is designed to emulate the Dimension Engineering SyRen 10 and Sabertooth 2x32 motor controllers on an Arduino Uno (or compatible).  It requires at least 6 pins that are capable of PWM.  It utilizes three IBT_2 motor controllers (BTS7960 H-bridge) each theoretically capable of 43A 12VDC loads to drive the Dome, Left and Right foot motors on an Astromech droid.  It responds to the packet serial commands sent from the Dimension Engineering Sabertooth library used in the SHADOW, SHADOW MD or Padawan controller sketches.  

This version of the code has been tested with the SHADOW and SHADOW MD sketches but should also be compatible with the Padawan based sketches as these all appear to utilize the same Sabertooth motor controller logic and library.  The sketch uses no external libraries to drive the IBT_2 motor drivers as these are simple to control with PWM signals.  Signal from the SHADOW sketch running on the Arduino Mega is carried from serial TX2 pin (pin 16) to the serial RX0 (pin 0) on the Arduino Uno running this sketch.

These are the IBT_2 modules I used...
https://www.amazon.com/BTS7960-Driver-Module-Arduino-Current/dp/B091TPZ9R1
US price for three (3) was $18.39.

Other compatible units can be found on Amazon or on AliExpress...
https://www.aliexpress.com/wholesale?SearchText=BTS7960

Search for "IBT_2", "IBT-2" or "BTS7960".
