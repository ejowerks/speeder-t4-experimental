void controlSafe() {

  /// if you aren't reading LTM from the FC you will need to disable voltage and safety attitude monitoring buried in random places below

  uint32_t currentMillis = millis();

  /// this smooths out the hover when it's near the setpoint so it can remain aggressive without overshooting it and bouncing
  thrMin = 800;
  thrMaxClimb = 1600;
  float hoverRatiof = (float) PIDSetPointmm / (float) rangeFiltered;
  int hoverRatio = (float) hoverRatiof * 100.00;
  hoverRatio = constrain(hoverRatio, 100, 250);
  thrMax = map(hoverRatio, 250, 100, 1800, thrMaxClimb);


  // These next 3 lines will save your aircraft from shooting straight up at full throttle never to be seen again if something gets stuck to the main sensor (yes this happened to me)
  if ( rangeFiltered > (hoverHeight * 1.75) || rangeBackFiltered > (hoverHeight * 4) || killAll == 1 ) {
    thrMax = thrMin;
  }

  // disarm if tipped too far so you don't get all tangled up in the bushes when you crash
  int maxAngle = 75;
  if ( ( ltm_roll > maxAngle || ltm_roll < -maxAngle) || ( ltm_pitch > maxAngle || ltm_pitch < -maxAngle)) {
    killAll = 1;
  }

  // in main setup area change lion or lipo -- requires LTM telemetry working
  if (batterytype == "lion") {
    lowVolt = 2.75;
    criticalVolt = 2.52;
  } else {
    lowVolt = 3.49;
    criticalVolt = 3.25;
  }

  ltm_voltagecell_raw = ltm_voltage / cells;
  ltm_voltagecell = voltFilter.updateEstimate(ltm_voltagecell_raw);
  int battZero = ((criticalVolt / 4.20) * 100); // example: 3.25/4.20*100 = 77
  battCurr = ((ltm_voltagecell / 4.20) * 100);
  battCurrPerc = map(battCurr, battZero, 100, 0, 100);


  int maxThrusterAdj = 160;
  // Poor man's current limiter?
  if (battCurrPerc <= 10) {
    maxThrusterAdj = map(battCurrPerc, 0, 10, 60, 160);
  }
  maxThruster = map(channel_6_pwm, 1000, 2000, 60, maxThrusterAdj);

  if ( slowDown == 1 ) { // once this is set it doesn't ever go back and get your ass home
    maxThruster = 40; // override prev thruster settings/curves
  }

  PIDSetPointmm = hoverHeight + turnheight; // disabled above so this is always on unless landing


  // Voltage Stuff try to save batteries
  if (ltm_voltagecell <= lowVolt && ltm_voltage > 2.0) { // gt 2.0 so no batt needed when on usb
    voltLowTimer++;
  } else {
    voltLowTimer = 0;
  }

  if (ltm_voltagecell <= criticalVolt && ltm_voltage > 2.0) { // gt 2.0 so no batt needed when on usb
    voltCriticalTimer++;
  } else {
    voltCriticalTimer = 0;
  }

  // race is over
  if ( (autolanding == 0) && (voltLowTimer > 5000)) {
    slowDown = 1;
  }

  if ( (autolanding == 0) && (voltCriticalTimer > 1500)) { // auto land
    slowDown = 1;
    autolanding = 1;
    isFlying = 0;
  }


  if ( (autolanding == 0) && (voltCriticalTimer > 5000)) { // kill to try to save battery
    killAll = 1;
  }




  /// turn system on and start flying
  if ( (autolanding == 0) && ( armSwitch > 1300 ) && ( engageSwitch > 1800 ) && (killAll == 0) ) {
    Engage = 1;
    thrMaxTmp = thrMax;
    hoverPID.SetMode(AUTOMATIC);
    prevEngage = 1;
    isFlying = 1;
  }


  /// turn system off and land
  if ( (autolanding == 1) || ( armSwitch > 1300  &&  engageSwitch < 1800 ) ) {
    isFlying = 0;
    thrMin = 200;
    thrMax = thrMaxTmp;
    if (prevEngage == 1) {
      if (currentMillis > landing_Time) {
        thrMaxTmp = thrMaxTmp - 10;
        if (PIDSetPointmm > -200) { /// setpoint must be "under-ground" to keep from bouncing around in ground effect/propwash
          PIDSetPointmm = PIDSetPointmm - 10;
        }
        landing_Time = currentMillis + 22;
      }
      if ( (thrMaxTmp <= 1000) || (rangeFiltered < 30) ) {
        prevEngage = 0;
        thrMax = 187;
        hoverThrottle = 193;
        Engage = 0;
        killAll = 1;
        autolanding = 0;
        hoverPID.SetMode(MANUAL);
      }
    }
  }

  /// kill entire system immediately
  if ( armSwitch < 1300 || killAll == 1 ) {
    thrMax = 200;
    thrMaxTmp = thrMax;
    if (armSwitch < 1300) {
      killAll = 0;
    }
    Engage = 0;
    isFlying = 0;
    prevEngage = 0;
    ledInterval = 800000;
    hoverThrottle = 193;
    PIDOutThrottlePWM = 0;
    PIDSetPointmm = 0;
    hoverPID.SetMode(MANUAL);
  }
}

void hoverMain() {
  if (tfmP.getData( tfDist, tfFlux, tfTemp)) {
    rangeFrontRaw = tfDist;
  }
  rangeFiltered = hoverFilter.updateEstimate(rangeFrontRaw);

  if (sensorBack.dataReady()) {
    rangeBackRaw = sensorBack.read();
  }
  rangeBackFiltered = sensorBackFilter.updateEstimate(rangeBackRaw);

  int hoverThrottleraw = map(PIDOutThrottlePWM, 0, 255, thrMin, thrMax);
  hoverThrottle = throFilter.updateEstimate(hoverThrottleraw);

  PIDInputHeightmm = rangeFiltered;

  if (taranis == 1) {
    KpTune = 0.0001 * (map(channel_10_pwm, 1000, 2000, 0, 10000));
    KiTune = 0.0001 * (map(channel_12_pwm, 1000, 2000, 0, 10000));
    KdTune = 0.0001 * (map(channel_11_pwm, 1000, 2000, 0, 10000));

    KpHover = KpTune;
    KiHover = KiTune;
    KdHover = KdTune;
  }

  if (taranis == 0) {
    KpHover = 0.5;
    KiHover = 0.5;
    KdHover = 0.25;
  }

  hoverPID.SetTunings(KpHover, KiHover, KdHover);
  hoverPID.Compute();

}



void radioComms() {
  uint32_t currentMillis = millis();

  if (currentMillis > radio_rw_loop_Time) {


    if (taranis == 0) {

      float scale = 0.625;
      float bias  = 895.0;
      channel_1_pwm = getRadioPWM(1);
      channel_2_pwm = getRadioPWM(2);
      channel_3_pwm = getRadioPWM(3);
      channel_4_pwm = getRadioPWM(4);
      channel_5_pwm = getRadioPWM(5);
      channel_6_pwm = getRadioPWM(6);
      float b = 0.2; //lower=slower, higher=noiser
      channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
      channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
      channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
      channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;
      channel_5_pwm = (1.0 - b) * channel_5_pwm_prev + b * channel_5_pwm;
      channel_6_pwm = (1.0 - b) * channel_6_pwm_prev + b * channel_6_pwm;
      channel_1_pwm_prev = channel_1_pwm;
      channel_2_pwm_prev = channel_2_pwm;
      channel_3_pwm_prev = channel_3_pwm;
      channel_4_pwm_prev = channel_4_pwm;
      channel_5_pwm_prev = channel_5_pwm;
      channel_6_pwm_prev = channel_6_pwm;

      escWrite = 0;

      /// rear thruster
      if (( isFlying == 1 ) && ( killAll == 0 )  && (taranis == 0)) {
        if ( rangeFiltered > (hoverHeight * 0.2) ) {
          escWrite = map(channel_2_pwm, 1500, 2000, 8, maxThruster);
          escWrite = constrain(escWrite, 8, maxThruster);
        }
      }

      // for calibrating thruster ESC because it is PWM -- never had any luck with oneshot (slowed down main loop)
      //escWrite = map(channel_2_pwm, 1500, 2000, 0, 180);
      //Serial.println(escWrite);

      ESC.write(escWrite);


      //// MAIN MIXING

      int st_pos = channel_1_pwm;
      int pitchback = 993;

      int stmix_pos = map(channel_5_pwm, 1000, 2000, -200, -100);

      int roll_min = 1050 - stmix_pos;
      int roll_max = 1950 + stmix_pos;
      int roll_pos = map(st_pos, 1000, 2000, roll_min, roll_max);
      int yaw_pos = st_pos;
      int yaw_min = 1250 + stmix_pos;
      int yaw_max = 1750 - stmix_pos;
      yaw_pos = map(st_pos, 1000, 2000, yaw_min, yaw_max);

      roll_pos = map(roll_pos, 1000, 2000, 172, 1811);
      yaw_pos = map(yaw_pos, 1000, 2000, 172, 1811);

      thro_channel_rx = map(channel_2_pwm, 1242, 1768, 172, 1811);
      armSwitch = channel_4_pwm;
      engageSwitch = armSwitch;
      channel_4_pwm = map(channel_4_pwm, 1000, 2000, 172, 1811);


      // Set all SBUS to mid channel (sort of a failsafe)
      for (uint8_t i = 0; i < 16; i++) {
        sbusChannels[i] = 993; /// set all SBUS out to mid channel
      }
      // except qwad throttle needs to go to 172
      sbusChannels[0] = 172;


      // Throttle Channel
      if (Engage == 1) {
        sbusChannels[0] = hoverThrottle;
      }

      // Roll Channel
      roll_sum = roll_pos + roll_des_flow;
      sbusChannels[1] = constrain(roll_sum, 172, 1811);

      /// Pitch Channel & HoverHeight stuff
      /// Pitchback: Pitch back a little while yawing and rolling so the nose isn't looking at the ground
      /// TurnHeight: Add some hover height on sharp turns because ToF sensor thinks it gains altitude when it rolls

      turnheight = 0;

      if (st_pos > 1515) {
        pitchback = map(st_pos, 1500, 2000, 993, 800);
        turnheight = map(st_pos, 1500, 2000, 0, 200);
      }
      if (st_pos < 1485) {
        pitchback = map(st_pos, 1000, 1500, 800, 993);
        turnheight = map(st_pos, 1000, 1500, 200, 0);
      }
      sbusChannels[2] = pitchback + pitch_des_flow;

      // Tilt back a bit for a little bit of oh-shit braking effect
      if (thro_channel_rx < 850) {
        sbusChannels[2] = map(thro_channel_rx, 193, 850, 850, 993);
      }

      // Yaw Channel
      sbusChannels[3] = yaw_pos; //yaw

      // Arming, Pre-Arming, Angle Mode
      // Make sure Betaflight aux channels match. Do not disable pre-arm if you like your face
      sbusChannels[4] = constrain(channel_4_pwm, 172, 1811); //armsw aux1 betaflight
      sbusChannels[15] = constrain(thro_channel_rx, 172, 1811); // for pre-arm with throttle trigger
      sbusChannels[8] = 193; /// force angle mode
      sbusChannels[9] = constrain(channel_3_pwm, 172, 1811);
      sbusChannels[10] = constrain(channel_5_pwm, 172, 1811);
      sbusChannels[11] = constrain(channel_6_pwm, 172, 1811);


    }


    // Xm+ receiver connected to other serial port, for flying this like a qwad and testing and whatnot
    if (taranis == 1) {
      if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame)) {

        float scale = 0.615;
        float bias  = 895.0;
        int thro_channel_rx = sbusChannels[0];
        int pitch_channel_rx = sbusChannels[2];
        int roll_channel_rx = sbusChannels[1];
        armSwitch = sbusChannels[5] * scale + bias;
        engageSwitch = sbusChannels[11] * scale + bias;
        channel_1_pwm = sbusChannels[0] * scale + bias;
        channel_5_pwm = sbusChannels[5] * scale + bias;
        channel_6_pwm = sbusChannels[6] * scale + bias;
        channel_7_pwm = sbusChannels[7] * scale + bias;
        channel_8_pwm = sbusChannels[8] * scale + bias;
        channel_10_pwm = sbusChannels[9] * scale + bias;
        channel_11_pwm = sbusChannels[10] * scale + bias;
        channel_12_pwm = sbusChannels[14] * scale + bias;

        if (Engage == 1) {
          sbusChannels[0] = hoverThrottle;
          sbusChannels[2] = pitch_channel_rx + pitch_des_flow;
          sbusChannels[1] = roll_channel_rx + roll_des_flow;
        }

      }
    }



    // Kill switch
    if (killAll == 1) {
      sbusChannels[0] = 193; // throttle to 0
      sbusChannels[4] = 193; // this should cause BF to disarm
    }


    //What makes time-travel possible
    sbus.write(&sbusChannels[0]);

    radio_rw_loop_Time = currentMillis + 10;

  }
}


void flowIt() {

  uint32_t currentMillis = millis();

  if (currentMillis > flow_loop_Time) {

    //KpTune = 0.0001 * (map(channel_5_pwm, 1000, 2000, 0, 10000));
    //KdTune = 0.0001 * (map(channel_6_pwm, 1000, 2000, 0, 10000));

    float KpFlow = 0.25;
    float KiFlow = 0.00;
    float KdFlow = 0.25;


    flow.readMotionCount(&deltaX, &deltaY);

    if ( (channel_2_pwm > 1450) && (channel_2_pwm < 1550) && (channel_1_pwm > 1450) && (channel_1_pwm < 1550) ) {
      idleFlow++;
    } else {
      idleFlow = 0;
    }


    if ( idleFlow > 100 ) {

      if ( (moveX > 500) || (moveX < -500)) {
        moveX = 0;
        moveY = 0;
      }

      moveX = moveX + deltaX;
      moveY = moveY + deltaY;

      xPid.SetTunings(KpFlow, KiFlow, KdFlow);
      xPid.Compute();

      yPid.SetTunings(KpFlow, KiFlow, KdFlow);
      yPid.Compute();

      int maxLean = 40;
      pitch_des_flow = constrain(deltaXpitch, -maxLean, maxLean);
      roll_des_flow = constrain(deltaYroll, -maxLean, maxLean);

      pitch_des_flow = pitchFilter.updateEstimate(pitch_des_flow);
      roll_des_flow = rollFilter.updateEstimate(roll_des_flow);


    } else {

      moveX = 0.00;
      moveY = 0.00;
      pitch_des_flow = 0;
      roll_des_flow = 0;

    }

    flow_loop_Time = currentMillis + 10;


  }
}



void loopRate(int freq) {
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}



/// init functions



void flowInit() {
  if (!flow.begin()) {

    while (true) {
      Serial.println("Initialization of the flow sensor failed");
      delay(500);
    }
  }
}

void tfInit() {
  delay(20);
  tfmP.begin( &Serial5);

  if ( tfmP.sendCommand( SOFT_RESET, 0))
  {
    Serial.printf( "passed.\r\n");
  }
  else tfmP.printReply();
  delay(50);

  if ( tfmP.sendCommand( SET_FRAME_RATE, FRAME_1000))
  {
    Serial.printf( "%2uHz.\r\n", FRAME_1000);
  }
  else tfmP.printReply();
  delay(50);

  if ( tfmP.sendCommand( STANDARD_FORMAT_MM, 0))
  {
    Serial.printf( "%2uHz.\r\n", STANDARD_FORMAT_MM);
  }
  else tfmP.printReply();
  delay(50);

  if ( tfmP.sendCommand( SAVE_SETTINGS, 0))
  {
    Serial.printf( "%2uHz.\r\n", FRAME_1000);
  }
  else tfmP.printReply();
  delay(50);

  delay(500);            // And wait for half a second.
}


void rearToFinit() {

  sensorBack.setTimeout(3000);
  if (!sensorBack.init()) {
    Serial.println("Failed to detect and initialize sensorBack!");
    while (5);
  }

  delay(50);

  sensorBack.setDistanceMode(VL53L1X::Short);
  sensorBack.setMeasurementTimingBudget(10000);
  sensorBack.startContinuous(10);

}
