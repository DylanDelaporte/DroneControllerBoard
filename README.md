# DroneControllerBoard
This is one of the "package" of DroneOS (all repos are on github), here you will find the C embedded code for an Arduino.

## Installation

Like every sketch, you have to push it on the same way to the Arduino.

## Bugs

This is an old code that I pushed and it worked, if you find any bug please open an issue and I will correct it.

## Commands

Following you will find commands that the Arduino can receive from serial :

| Name | Synthax |
| ------------- | ----------- |
| power, rotation, axis | _P motors(%)\|rotation(°)\|XAXIS(°)\|YAXIS(°)_ |
| balance motors | _B motor1(%)\|motor2(%)\|motor3(%)\|motor4(%)_ |
| calibrate accelerometer | _A Y_ |
| mode (automatic/manual) | _M 1 or 0_ |
| options | _O axisSensibility(°)\|rotationSensibility(°)_ |
| informations | _I Y_ |
| lock drone (sleep mode) | _L Y/N_ |
| sonars | _S Y/N_ |
| test motors (on/off) | _T motorNumber_ |
