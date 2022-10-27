# Original EjoWerks Speeder
https://sites.google.com/view/ejowerks-speeder/

- This is the original testing Teensy 4 version -- Works on Teensy 4.0 (and should work on the 4.1 but I have not tested it) 

- Currently tuned for a quadcopter with a 1500mah 4s battery, 4 inch props with low kv motors. Any deviations in cell count, motor size, kv, prop size, aircraft weight, etc will require tuning the hover PIDs along with max throttle settings. Start low and work your way up

- Depends on Betaflight rc_smoothing to keep it from getting too jumpy and heating up motors/ESCs. Will post Betaflight settings here soon

- This speeder flies well with this code as-is, but this it needs a lot of cleanup, more safety features, and could be made far more efficient 



****************************** WARNING ******************************

THIS EXPERIMENTAL PROJECT INTERCEPTS RC SIGNALS AND FULLY AUTOMATES THRUST CONTROL OF A POWERFUL FLYING DEVICE

MALFUNCTIONS CAN RESULT IN AIRCRAFT FLYAWAYS OR ***PERMANENT DISFIGUREMENT TO YOUR FACE, HANDS, AND OTHER BODY PARTS***

DO NOT OPERATE WITHOUT THE SECONDARY VL53L1X SENSOR - IT IS PART OF THIS PROJECT FOR AN IMPORTANT REASON 


ALWAYS TEST WITH PROPELLERS REMOVED

YOU HAVE BEEN WARNED -- USE THIS CODE AT YOUR OWN RISK!

(GOOGLE IMAGE SEARCH "DRONE INJURIES" FOR INSPIRATION)

****************************** WARNING ******************************




<hr>

### In a nutshell:

TF-Mini-Plus ToF @ 1khz # Serial #  -> Teensy 4.0 Serial2 --> Mixer PID, etc

Flysky FS-X6B RC Car RX # PPM # -> Teensy 4.0 pin 16 --> Mixer Code -> SBUS out -> BetaFlight SBUS in

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
