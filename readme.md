#  YM2612 Test Code (AVR / Arduino Uno)

This program tests the Yamaha [YM2612](https://en.wikipedia.org/wiki/Yamaha_YM2612) FM synthesis sound chip  by configuring it to sound like a grand piano and cycling a note on and off in a loop. 

The code is designed for an ATmega328/ATmega168 mcu with a 16MHz external crystal. It has been heavily adopted from test code by Furrtek and [Fabien Batteix](skywodd@gmail.com). The 16MHz crystal comes built-in with the [Arduino Uno](https://www.arduino.cc/en/uploads/Main/arduino-uno-schematic.pdf). 

For more information about the YM2612 registers please see: http://www.smspower.org/maxim/Documents/YM2612

A copy of the YM2612 pinout can be found here: https://console5.com/wiki/YM2612
 
Folder structure and `main.cpp` have been configured for use with the Visual Studio Code [PlatformIO](https://platformio.org/) plugin.   
