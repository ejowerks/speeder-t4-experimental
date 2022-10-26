unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6;
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

void radioSetup() {

  //Declare interrupt pin
  pinMode(PPM_Pin, INPUT_PULLUP);
  delay(20);
  //Attach interrupt and point to corresponding ISR function
  attachInterrupt(digitalPinToInterrupt(PPM_Pin), getPPM, CHANGE);

}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from interrupt routines
  unsigned long returnPWM;

  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }

  return returnPWM;
}



//========================================================================================================================//





void getPPM() {
  unsigned long dt_ppm;
  int trig = digitalRead(PPM_Pin);
  if (trig == 1) { //only care about rising edge
    dt_ppm = micros() - time_ms;
    time_ms = micros();


    if (dt_ppm > 5000) { //waiting for long pulse to indicate a new pulse train has arrived
      ppm_counter = 0;
    }

    if (ppm_counter == 1) { //first pulse
      channel_1_raw = dt_ppm;
    }

    if (ppm_counter == 2) { //second pulse
      channel_2_raw = dt_ppm;
    }

    if (ppm_counter == 3) { //third pulse
      channel_3_raw = dt_ppm;
    }

    if (ppm_counter == 4) { //fourth pulse
      channel_4_raw = dt_ppm;
    }

    if (ppm_counter == 5) { //fifth pulse
      channel_5_raw = dt_ppm;
    }

    if (ppm_counter == 6) { //sixth pulse
      channel_6_raw = dt_ppm;
    }

    ppm_counter = ppm_counter + 1;
  }
}
