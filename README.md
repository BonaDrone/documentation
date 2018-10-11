# Documentation
Different bits of documentation to ease BonaDrone's developers life


## Autogenerate MSP commands Hackflight
Each time the `messages.json` file is modified the following commands should be executed for the changes to take effect:
* `cd` into `Hackflight/extras/parser`
* `$ python3 msppg.py`
* `$ cd output/python/`
* `$ sudo python3 setup.py install`


## Control loop

### Current
![Current control loop](extras/PID-modified.png)

### Old
![Proposed Control loop](extras/PID-Loop-Original.png)

## ESP32 

### Requirements

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
float ACCEL_BIAS[3] = {0.003470, -0.000418, 0.039145};
float GYRO_BIAS[3]  = {0.896399, -1.366036, -2.004719}; 
```

