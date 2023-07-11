# BotHudson

Line follower and Sumo mobots for ASICUP Competition.

# Introduction
Line follower mobot is a mobile robot that can detect and follow the line drawn on the floor. Sumobot is a sport in which two robots attempt to push each other out of a circle (in a similar fashion to the sport of sumo). It is designed for competing in ASICUP (Agility, Strength, Intelligence) which is takes place annualy at the University of the East - Caloocan that is open to any major courses with maximum of 4 members while the mobots are limited to the dimensions 15 cm x 15 cm x 15 cm, with maximum weight of 3 kgs. Both of the Line follower and Sumobot are fully tunable for every competition they may take part.

<p align="center">
<img src="https://user-images.githubusercontent.com/69707914/235178001-e73297c8-888e-413a-9ebc-ebc72bcb7b5f.png" alt="BotHudson Line Follower and SumoBot" width="500">
</p>

---

The competition is a part of the 3rd Year ECE students' projects while encouraged 2nd and 4th Year students to compete organized by [ECES-IECEP (University of the East - Caloocan)](https://www.facebook.com/eces.iecep/).

## Installation
```
git clone https://github.com/EthanFrancisco/BotHudson.git
```

> Requirements should be installed before uploading to the Arduino to provide the necessary libraries.

## Requirements:

### QTRSensors
```
git clone https://github.com/pololu/qtr-sensors-arduino.git
```
The [QTRSensors library](https://github.com/pololu/qtr-sensors-arduino) supports Pololu's second-generation dimmable QTR and QTRX reflectance sensor boards, as well as older QTR sensors. Before continuing, careful reading of the QTR Reflectance Sensor Application Note is recommended. For complete documnentation of this library, see [the qtr-sensors-arduino documentation](https://pololu.github.io/qtr-sensors-arduino/). If you are already on that page, see the QTRSensors class reference.
> Authored by Pololu, "[qtr-sensors-arduino](https://github.com/pololu/qtr-sensors-arduino)"

### SharpIR
```
git clone https://github.com/qub1750ul/Arduino_SharpIR.git
```
An Arduino library that allows to acquire distance data from a Sharp analog distance sensor.\
Current Supported sensor models:
- GP2Y0A41SK0F
- GP2Y0A21YK0F
- GP2Y0A02YK0F
> Authored by Giuseppe Masino, "[Arduino_SharpIR](https://github.com/qub1750ul/Arduino_SharpIR)".

### Ultrasonic
```
git clone https://github.com/ErickSimoes/Ultrasonic.git
```
Minimalist library for ultrasound module to Arduino which reduces code execution, validation and unnecessary use of global variables, prioritizing smaller data types.\
Current Supported modules:
- HC-SR04
- Ping)))
- Seeed SEN136B5B
> Authored by Erick Simoes, "[Ultrasonic](https://github.com/ErickSimoes/Ultrasonic)".

## License
The BotHudson Repository by Ethan Francisco is licensed under a [MIT License](https://opensource.org/license/mit/).\
Feel free to contact the author on Twitter. [@MrEthanooo](https://twitter.com/MrEthanooo)\
The license text can be found in the [LICENSE.md](https://github.com/EthanFrancisco/BotHudson/blob/main/LICENSE) file for details.
