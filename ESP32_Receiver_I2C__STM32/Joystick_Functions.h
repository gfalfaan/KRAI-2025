#ifndef JOYSTICK_FUNCTIONS_H
#define JOYSTICK_FUNCTIONS_H

#include "variables.h"

// ======================== FUNGSI JOYSTICK ========================

void readJoystick() {
  lx = map(incomingData.joyData[0], -4095, 4095, -128, 127);
  ly = map(incomingData.joyData[1], -4095, 4095, -128, 127);
  rx = map(incomingData.joyData[2], -4095, 4095, -128, 127);
  
  if (abs(lx) < 10) lx = 0;
  if (abs(ly) < 10) ly = 0;
  if (abs(rx) < 10) rx = 0;
}

void onConnect() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(pinBuzzer, HIGH);
    delay(100);
    digitalWrite(pinBuzzer, LOW);
    delay(100);
  }
  Serial.println("✅ Remote Connected!");
  remoteConnected = true;
}

#endif
