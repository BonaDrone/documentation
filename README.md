# Documentation
Different bits of documentation to ease BonaDrone's developers life

## Battery Check
For BonaDrone's FC, the battery check can be performed by reading the value of the ESP32's pin 33.

## Autogenerate MSP commands Hackflight
Each time the `messages.json` file is modified the following commands should be executed for the changes to take effect:
* `cd` into `Hackflight/extras/parser`
* `$ python3 msppg.py`
* `$ cd output/python/`
* `$ sudo python3 setup.py install`


## Control loop

### Current
**Rate PID**
![Rate PID](extras/rate-pid.png)

**Level PID**
![Level PID](extras/level-pid.png)

**AltHold PID**
![AltHold PID](extras/althold-pid.png)
Based on [iNav's AltHold PID](https://github.com/iNavFlight/inav/wiki/Developer-info)

### Old
![Old control loop](extras/PID-modified.png)

### Original
![Original Control loop](extras/PID-Loop-Original.png)

## ESP32 

### Requirements

Follow the [official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md) to get the ESP32 drivers on your Arduino IDE.

Make sure you have first uploaded the `ESP32_flash_loader.ino` under the `extras` folder into the board.

### Start ESP32 in boot mode

* Remove power from board
* Connect ESP32 GPIO 0 pin to ground
* With the previous connection done, power the board

### Arduino settings to program the ESP32

* Upload Speed: 115200
* Flash Frequency: 80 MHz
* Flash Mode: DIO
* Flash Size: 4MB (32 Mb)
* Partition Scheme: Default
* Core Debug Level: None
* PSRAM: Disabled

## IMU biases
```C
// Biases
float ACCEL_BIAS[3] = {-0.016238,-0.009853,0.032948};
float GYRO_BIAS[3]  = {0.910652,-1.543902,-2.956785};
```
**15-11-2018**
```C
Gyro biases (x, y, z): 0.837052,-1.535432,-3.138039,
Accel biases (x, y, z): -0.014827,-0.000640,0.036611,
```

## Juan transmitter trims
```C
// Trim receiver via software
rc.setTrimRoll(-0.0012494f);
rc.setTrimPitch(-0.0058769f);
rc.setTrimYaw(-0.0192190f);
```

**15-11-2018**
```C
rc.setTrimRoll(-0.0030506f);
rc.setTrimPitch(-0.0372178f);
rc.setTrimYaw(-0.0384381f);
```
