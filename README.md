# Original EjoWerks Speeder
https://sites.google.com/view/ejowerks-speeder/

- This is the original testing Teensy 4 version -- Works on Teensy 4.0 (and should work on the 4.1 but I have not tested it) 

- This code is a horrifically bastardized version of [dRehmFlight](https://github.com/nickrehm/dRehmFlight) without IMU and mixer code thrown together just to make it work. It is messy, needs unused stuff removed, lots of unused vars, etc.

- Wholly depends on Betaflight rc_smoothing to keep it from getting too jumpy. Will post settings here soon.

<hr>

### In a nutshell:

TF-Mini-Plus ToF @ 1khz # Serial #  -> Teensy 4.0 Serial2 --> Mixer PID, etc

Flysky FS-X6B RC Car RX # PPM # -> Teensy 4.0 pin 0 --> Mixer Code -> SBUS out -> BetaFlight SBUS in

VL53L1X Backup ToF @ 50hz # IC2 # -> Teensy 4.0 IC2

PMW3901 Optical Flow # SPI # -> Teensy 4.0 SPI

Teensy 4.0 # Serial3 # in <-- FC SerialX Betaflight LTM Telemetry (voltage/orientation monitoring)

<hr>



### Parts Required:

Full Betaflight/INAV quadcopter, tested, working, ready to fly (Sorry I simply cannot give guidance with this part)

Additional motor and ESC for rear thruster

Teensy 4.0 (uh oh these are getting hard to find)

TFmini Plus Lidar Range Finder Sensor

VL53L1X Backup/ Secondary ToF Sensor

PMW3901 Optical Flow Sensor (optional if you want position hold)

Flysky FS GT5 RC Transmitter

Flysky FS-X8B Receiver



### Skills That Will Be Very Helpful

FPV quad experience 

Micro soldering experience

Arduino IDE experience because you will definitely be editing these files
