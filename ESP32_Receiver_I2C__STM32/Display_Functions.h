#ifndef DISPLAY_FUNCTIONS_H
#define DISPLAY_FUNCTIONS_H

#include "variables.h"

// ======================== FUNGSI DISPLAY ========================

void printHeader() {
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║  ESP32 I2C SWERVE - MODULAR .h       ║");
  Serial.println("╚═══════════════════════════════════════╝");
}

void printStatus() {
  Serial.print("\n📡 MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("💾 Remote Index: ");
  Serial.println(storedMac);
  Serial.println("\n✅ Waiting for remote...\n");
}

#endif
