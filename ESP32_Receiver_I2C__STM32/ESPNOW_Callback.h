#ifndef ESPNOW_CALLBACK_H
#define ESPNOW_CALLBACK_H

#include "variables.h"
#include "Joystick_Functions.h"
#include "EEPROM_Functions.h"

// ======================== CALLBACK ESP-NOW ========================

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  packetCount++;
  memcpy(&incomingData, data, sizeof(incomingData));
  
  if (!remoteConnected) onConnect();
  
  if (storedMac != incomingData.remoteIndex) {
    saveRemoteIndex(incomingData.remoteIndex);
    storedMac = incomingData.remoteIndex;
    Serial.println("\n🔄 Restarting...\n");
    delay(100);
    ESP.restart();
  }
  
  if (DEBUG_MODE) {
    Serial.print("📦 #");
    Serial.print(packetCount);
    Serial.print(" | Joy(");
    Serial.print(incomingData.joyData[0]);
    Serial.print(",");
    Serial.print(incomingData.joyData[1]);
    Serial.print(") Rx=");
    Serial.println(incomingData.joyData[2]);
  }
  
  lastReceiveTime = millis();
  connectionStatus = true;
}

#endif
