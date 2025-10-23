#ifndef EEPROM_FUNCTIONS_H
#define EEPROM_FUNCTIONS_H

#include "config.h"
#include "variables.h"

// ======================== FUNGSI EEPROM ========================

void saveRemoteIndex(uint8_t index) {
  EEPROM.write(EEPROM_ADDR_REMOTE, index);
  EEPROM.commit();
  Serial.println("[EEPROM] Remote saved: " + String(index));
}

uint8_t loadRemoteIndex() {
  uint8_t idx = EEPROM.read(EEPROM_ADDR_REMOTE);
  if (idx > 3) idx = 0;
  Serial.println("[EEPROM] Loaded: " + String(idx));
  return idx;
}

#endif
