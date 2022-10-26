enum ltmStates {
  IDLE,
  HEADER_START1,
  HEADER_START2,
  HEADER_MSGTYPE,
  HEADER_DATA
};

#define LONGEST_FRAME_LENGTH 18

#define GFRAMELENGTH 18
#define AFRAMELENGTH 10
#define SFRAMELENGTH 11
#define OFRAMELENGTH 18
#define NFRAMELENGTH 10
#define XFRAMELENGTH 10

const char* flightModes[] = {
  "Manual",
  "Rate",
  "Angle",
  "Horizon",
  "Acro",
  "Stabilized1",
  "Stabilized2",
  "Stabilized3",
  "Altitude Hold",
  "GPS Hold",
  "Waypoints",
  "Head free",
  "Circle",
  "RTH",
  "Follow me",
  "Land",
  "Fly by wire A",
  "Fly by wire B",
  "Cruise",
  "Unknown"
};

typedef struct remoteData_s {
  int16_t pitch;
  int16_t roll;
  int16_t heading;
  float voltage;
  byte rssi;
  bool armed;
  bool failsafe;
  byte flightmode;

  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  uint8_t groundSpeed;
  int16_t hdop;
  uint8_t gpsFix;
  uint8_t gpsSats;

  int32_t homeLatitude;
  int32_t homeLongitude;

  uint8_t sensorStatus;
} remoteData_t;

remoteData_t remoteData;

uint8_t serialBuffer[LONGEST_FRAME_LENGTH];
uint8_t state = IDLE;
char frameType;
byte frameLength;
byte receiverIndex;

byte readByte(uint8_t offset) {
  return serialBuffer[offset];
}

int readInt(uint8_t offset) {
  return (int) serialBuffer[offset] + ((int) serialBuffer[offset + 1] << 8);
}

int32_t readInt32(uint8_t offset) {
  return (int32_t) serialBuffer[offset] + ((int32_t) serialBuffer[offset + 1] << 8) + ((int32_t) serialBuffer[offset + 2] << 16) + ((int32_t) serialBuffer[offset + 3] << 24);
}

uint32_t nextDisplay = 0;




void readLTM() {

  if (millis() >= nextDisplay) {

    ltm_voltage = (remoteData.voltage / 1000);
    ltm_roll = (remoteData.roll);
    ltm_pitch = (remoteData.pitch);
    nextDisplay = millis() + 50;

  }

  if (Serial3.available()) {

    char data = Serial3.read();

    if (state == IDLE) {
      if (data == '$') {
        state = HEADER_START1;
      }
    } else if (state == HEADER_START1) {
      if (data == 'T') {
        state = HEADER_START2;
      } else {
        state = IDLE;
      }
    } else if (state == HEADER_START2) {
      frameType = data;
      state = HEADER_MSGTYPE;
      receiverIndex = 0;

      switch (data) {

        case 'G':
          frameLength = GFRAMELENGTH;
          break;
        case 'A':
          frameLength = AFRAMELENGTH;
          break;
        case 'S':
          frameLength = SFRAMELENGTH;
          break;
        case 'O':
          frameLength = OFRAMELENGTH;
          break;
        case 'N':
          frameLength = NFRAMELENGTH;
          break;
        case 'X':
          frameLength = XFRAMELENGTH;
          break;
        default:
          state = IDLE;
      }

    } else if (state == HEADER_MSGTYPE) {

      /*
         Check if last payload byte has been received.
      */
      if (receiverIndex == frameLength - 4) {
        /*
           If YES, check checksum and execute data processing
        */

        if (frameType == 'A') {
          remoteData.pitch = readInt(0);
          remoteData.roll = readInt(2);
          remoteData.heading = readInt(4);
        }

        if (frameType == 'S') {
          remoteData.voltage = readInt(0);
          remoteData.rssi = readByte(4);

          byte raw = readByte(6);
          remoteData.flightmode = raw >> 2;
        }

        if (frameType == 'G') {
          remoteData.latitude = readInt32(0);
          remoteData.longitude = readInt32(4);
          remoteData.groundSpeed = readByte(8);
          remoteData.altitude = readInt32(9);

          uint8_t raw = readByte(13);
          remoteData.gpsSats = raw >> 2;
          remoteData.gpsFix = raw & 0x03;
        }

        if (frameType == 'X') {
          remoteData.hdop = readInt(0);
          remoteData.sensorStatus = readByte(2);
        }

        state = IDLE;
        memset(serialBuffer, 0, LONGEST_FRAME_LENGTH);

      } else {
        /*
           If no, put data into buffer
        */
        serialBuffer[receiverIndex++] = data;
      }

    }

  }

}
