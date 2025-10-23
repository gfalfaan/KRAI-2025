#ifndef I2C_TASK_H
#define I2C_TASK_H

#include "variables.h"

// ======================== TASK I2C ========================

void I2CCommunicationSendTask(void *parameter) {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  
  for (;;) {
    for (int i = 0; i < 4; i++) {
      dataModule[i] = String(Drive_SP[i]) + "#" + String(Steer_SP[i]);
      int sz = dataModule[i].length();
      char data[sz + 1];
      dataModule[i].toCharArray(data, sz + 1);
      
      Wire.beginTransmission(SLAVE_ADDR_MODULE[i]);
      Wire.write((const uint8_t *)data, sz);
      Wire.endTransmission();
      
      if (DEBUG_MODE) {
        Serial.print("📤 M");
        Serial.print(i);
        Serial.print("[0x");
        Serial.print(SLAVE_ADDR_MODULE[i], HEX);
        Serial.print("]: ");
        Serial.println(dataModule[i]);
      }
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}

#endif