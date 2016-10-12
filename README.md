#RC-Vacuum-Cleaner
Radio controlled vacuum cleaning vehicle built with Arduino Uno R3 and nRF24L01 RF transceiver. Coded in C++.

**Components used (controller):**
- Arduino Uno R3
- nRF24L01, RF transceiver IC
- 10k potentiometer
- Push-button switch (x5)
- 10k resistors (x5)
- Power supply (either through laptop or 9V battery)
- Jumper cables

**Components used (receiver/vehicle):**
- Arduino Uno R3
- nRF24L01, RF transceiver IC
- Vehicle chassis
- Gear motor + car tire (x2)
- SN754410, H-bridge IC
- NTE253, NPN transistor
- 1K resistor
- Vacuum cleaner apparatus
 - Coffee bottle
 - 9V DC motor
 - Plastic fan (manually created from plastic cup)
- 9V battery (x2)
- Jumper cables

#Brief summary of how it works
- Vacuum cleaning vehicle controlled with controller through nRF24L01 RF transceiver
- Controller consists of 6 total input controls
 - 5 push-buttons
   - 4 for vehicle movement (decelerate, accelerate, left turn, right turn)
    - 1 for vacuum fan switch
 - 1 potentiometer used to adjust vehicle speed
- Vehicle receives instructions from controller to move and/or turn on vacuum fan
- H-bridge allows for vehicle motors to go in either direction
- NPN transistor acts as switch to turn on/off vacuum fan

#Video Demonstration
[RC Vacuum Cleaner Demo](https://www.youtube.com/watch?v=dmlWOtPn2n8 "RC Vacuum Cleaner Demo")

[RC Vehicle Movement Demo](https://www.youtube.com/watch?v=9YUVWv-RvW4 "RC Vehicle Movement Demo")
