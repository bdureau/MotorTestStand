# MotorTestStand
Rocket motor datalogger firmware.
Can do the following
- Use any gauge sensor, it calibrate itself using a known weight
- Trigger recording from your launch controller
- Can record from 10 to 40 measures per second
- Can record up to 25 thrust curves
- Ability to set the recording time
- Long range Telemetry
- Multi languages, so far can do French and English but can be translated in other languages 
- Graphical front end using an Android device
- Connect to Android using bluetooth or 3DR module to do long range telemetry
- Ability to export to a csv file or a RASP file
- Ability to flash the firmware from your Android device
- Application is available on the Android App store
- On line help for each screen

In addition it will allow you to configure any bluetooth or 3DR module using your Android app so that it can work with your board.

You will need to use the console front end to use your test stand
https://github.com/bdureau/RocketMotorTestStandConsole

The board look like this
<p></p>
<img src="/board images/teststand_board.jpg" width="49%">   

and the testStand can be something like this
<p></p>
<img src="/board images/TestStand.jpg" width="49%">   

Note that I you are not a developper you can flash your board with this firmware using the Android app that you have installed from the App Store

# Compiling the code
The firmware runs on different board and can be compiled for ATMega 328, STM32 or ESP32
To compile it for a specific board you need to edit the config.h file and select the appropriate board using compilation directives
Only uncomment one of those:

for ATMega328

#define TESTSTAND 

for STM32

#define TESTSTANDSTM32

or

#define TESTSTANDSTM32V2

for ESP32

#define TESTSTANDESP32

