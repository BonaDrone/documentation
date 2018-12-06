# Documentation
Different bits of documentation to ease BonaDrone's developers life

## EEPROM Usage

### EEPROM Parameters

The first 100 EEPROM slots are reserved for storing Mosquito parameters. The full list of parameters as well as its EEPROM addresses is listed below.

| Address |  Parameter Name | Bit Meaning                                                         |                                                                            Description                                                                            |                                                               MSP Message ID That sets it                                                              |
|:-------:|:---------------:|---------------------------------------------------------------------|:-----------------------------------------------------------------------------------------------------------------------------------------------------------------:|:------------------------------------------------------------------------------------------------------------------------------------------------------:|
|    0    | GENERAL CONFIG  | 7-,  6-,  5-,  4-,  3-,  2-,  1-hasPositioningBoard, 0-isMosquito90 | Sets the general configuration of the Mosquito. This address is reserved for boolean configuration values. Each of the bits is used to set/check a boolean flag.  | 223-MOSQUITO_VERSION: (0 if Mosquito 150, 1 if Mosquito 90) 225-POSITIONING_BOARD:  (0 if Doesn't have positioning board   1 if has positioning board) |
|   1-4   |  gyroRollPitchP |                                  -                                  | Sets the Rate P value for Roll and Pitch                                                                                                                          | 224-PID_CONSTANTS: msg param 0                                                                                                                         |
|   5-8   |  gyroRollPitchI |                                  -                                  | Sets the Rate I value for Roll and Pitch                                                                                                                          | 224-PID_CONSTANTS: msg param 1                                                                                                                         |
|   9-12  |  gyroRollPitchD |                                  -                                  | Sets the Rate D value for Roll and Pitch                                                                                                                          | 224-PID_CONSTANTS: msg param 2                                                                                                                         |
|  13-16  |     gyroYawP    |                                  -                                  | Sets the Rate P value for Yaw                                                                                                                                     | 224-PID_CONSTANTS: msg param 3                                                                                                                         |
|  17-20  |     gyroYawI    |                                  -                                  | Sets the Rate I value for Roll and Pitch                                                                                                                          | 224-PID_CONSTANTS: msg param 4                                                                                                                         |
|  21-24  |  demandsToRate  |                                  -                                  | Sets the demands to rate factor                                                                                                                                   | 224-PID_CONSTANTS: msg param 5                                                                                                                         |
|  25-28  |      levelP     |                                  -                                  | Sets the Level P value for Roll and Pitch                                                                                                                         | 224-PID_CONSTANTS: msg param 6                                                                                                                         |

The script [set_params](https://github.com/BonaDrone/documentation/blob/master/extras/scripts/set_params.py) under `/extras/scripts` can be used to change the parameters. Type:
`$ python3 set_params.py -h`
to see how to use it.

### EEPROM Mission storage

When programming a mission, the navigation related data is stored in the EEPROM from address 100 until the end.

## Battery Check
For BonaDrone's FC, the battery check can be performed by reading the value of the ESP32's pin 33.


## Autogenerate MSP commands Hackflight

Each time the `messages.json` file is modified the following commands should be executed for the changes to take effect:
* `cd` into `Hackflight/extras/parser`
* `$ python3 msppg.py`
* `$ cd output/python/`
* `$ sudo python3 setup.py install`

When adding a new MSP message to `messages.json` it should be taken into account that, due to the current implementation of [msppg.py](https://github.com/BonaDrone/Hackflight/blob/master/extras/parser/msppg.py), messages with ID codes lower than 200 are used to request data from Hackflight whereas messages with ID code equal or higher than 200 are used to receive data.  

If you would like to add some additional lines of code to `mspparser.hpp` and avoid them disappearing when running `msppg.py` be sure to add them to `Hackflight/extras/parser/resources/mspparser.hpp`  

## ESP32 

### Requirements

Follow the [official instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md) to get the ESP32 drivers on your Arduino IDE.

Make sure you have first uploaded the `ESP32_flash_loader.ino` under the `extras` folder into the board.

### Start ESP32 in boot mode

**Note**: The [ESP flash loader](https://github.com/BonaDrone/ESP32-Sketchs/blob/master/extras/ESP32_flash_loader/ESP32_flash_loader.ino) code does already force the ESP32 to enter boot mode, so the following 3 steps are not longer required.

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

**Note**: When the board is powered and initialized it automatically computes and loads Accel and Gyro biases so it is no longer requried to know this values and substitute them in the code.

```C
// Biases
float ACCEL_BIAS[3] = {-0.014827,-0.000640,0.036611};
float GYRO_BIAS[3]  = {0.837052,-1.535432,-3.138039};
```

## Juan transmitter trims

```C
// Trim receiver via software
rc.setTrimRoll(-0.0030506f);
rc.setTrimPitch(-0.0372178f);
rc.setTrimYaw(-0.0384381f);
```

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

## Hackflight Architecture
![ARchitecture](extras/hackflight_arch.png)
