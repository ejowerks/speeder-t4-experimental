void printRadioData() {
  Serial.print(F(" CH1: "));
  Serial.print(channel_1_pwm);
  Serial.print(F(" CH2: "));
  Serial.print(channel_2_pwm);
  Serial.print(F(" CH3: "));
  Serial.print(channel_3_pwm);
  Serial.print(F(" CH4: "));
  Serial.print(channel_4_pwm);
  Serial.print(F(" CH5: "));
  Serial.print(channel_5_pwm);
  Serial.print(F(" CH6: "));
  Serial.println(channel_6_pwm);
}

void printBig() {
  Serial.print(" thrMax: ");
  Serial.print(thrMax);
  Serial.print(" thrMaxTmp: ");
  Serial.print(thrMaxTmp);
  Serial.print(" throttleSbus: ");
  Serial.print(sbusChannels[0]);
  Serial.print(" rangeFrontRaw: ");
  Serial.print(rangeFrontRaw);
  Serial.print("rangeBackRaw: ");
  Serial.print(rangeBackRaw);
  Serial.print(" armSwitch: ");
  Serial.print(armSwitch);
  Serial.print(" engageSwitch: ");
  Serial.print(engageSwitch);
  Serial.print(" Engage: ");
  Serial.print(Engage);
  Serial.print(" prevEngage: ");
  Serial.print(prevEngage);
  Serial.print(" killAll: ");
  Serial.print(killAll);
  Serial.print(" hoverHeight: ");
  Serial.print(hoverHeight);
  Serial.print(" PIDSetPointmm: ");
  Serial.print(PIDSetPointmm);
  Serial.println();
}

void printRangers() {
  Serial.print("rangeFRONTFiltered: ");
  Serial.print(rangeFiltered);
  Serial.print(" rangeBACKFiltered: ");
  Serial.println(rangeBackFiltered);
}

void printFlow() {
  Serial.print(" deltaX: ");
  Serial.print(deltaX);
  Serial.print("\t");
  Serial.print(" deltaY: ");
  Serial.print(deltaY);
  Serial.print("\t");
  Serial.print(" moveX: ");
  Serial.print(moveX);
  Serial.print("\t");
  Serial.print(" moveY: ");
  Serial.print(moveY);
  Serial.print("\t");
  Serial.print(" pitch_des_flow: ");
  Serial.print(pitch_des_flow);
  Serial.print("\t");
  Serial.print(" roll_des_flow: ");
  Serial.print(roll_des_flow);
  Serial.print("\t");
  Serial.print(" idleFlow: ");
  Serial.print(idleFlow);
  Serial.println();
}

void printFlowControl() {
  Serial.print(" pitch_des_flow: ");
  Serial.print(pitch_des_flow);
  Serial.print("\t");
  Serial.print(" roll_des_flow: ");
  Serial.print(roll_des_flow);
  Serial.print("  KpTune");
  Serial.print(KpTune);
  Serial.print("  KiTune");
  Serial.print(KiTune);
  Serial.print("  KdTune");
  Serial.println(KdTune);
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt = "));
    Serial.println(dt * 1000000.0);
  }
}
