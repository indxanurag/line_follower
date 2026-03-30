**Super fast , PID controlled line following bot with easy web interface for tuning**

**Features**
- Web ui
- Turn control
- Auto stop at end point
- both digital and analog array support
- auto ir calibration
- speed control
- serial monitor in the webui for easy debug
- start, stop from web ui 


**ir sensor array to esp32 pin diagram**
**Component list -**
- ESP32
- DRV8833 motor driver
- N20 1000 RPM metal gear motor
- 44 mm wheel
- 1000uF capacitor
- 8 channel ir  array
- 8.4V battery
- ams 1117 voltage regulator IC

( add one 1000uf capacitor to the driver + and - terminal )

**- wiring to be used ( need to use 6 ir led array )**
- IR2 → GPIO 36
- IR3 → GPIO 39
- IR4 → GPIO 34  (center)
- IR5 → GPIO 35  (center)
- IR6 → GPIO 32
- IR7 → GPIO 33

**- esp32 to drv8833 motor driver -**
- IN1 → GPIO 23
- IN2 → GPIO 19
- IN3 → GPIO 13
- IN4 → GPIO 27


 original wiring , don't  use this wiring 
- IR1 → GPIO 36
- IR2 → GPIO 39
- IR3 → GPIO 34
- IR4 → GPIO 35  (center)
- IR5 → GPIO 32  (center)
- IR6 → GPIO 26
- IR7 → GPIO 25
- IR8 → GPIO 33
- (GPIO 25 and GPIO 26 is not usable while WiFi is turned on , so in this case you need to change the ir led wiring )
