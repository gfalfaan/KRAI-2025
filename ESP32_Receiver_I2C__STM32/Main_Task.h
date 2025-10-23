#ifndef MAIN_TASK_H
#define MAIN_TASK_H

#include "variables.h"
#include "Joystick_Functions.h"

// ======================== TASK UTAMA ========================

void mainApplicationTask(void *parameter) {
  pinMode(2, OUTPUT);
  
  for (;;) {
    if (remoteConnected) {
      readJoystick();
      
      if (ly <= -30 && (lx >= -40 && lx <= 40) || !incomingData.stat[0]) {
        Serial.println("➡️ MAJU");
        for (int i = 0; i < 4; i++) { Steer_SP[i] = 0; Drive_SP[i] = 100; }
      }
      else if (ly >= 30 && (lx >= -50 && lx <= 50) || !incomingData.stat[1]) {
        Serial.println("⬅️ MUNDUR");
        for (int i = 0; i < 4; i++) { Steer_SP[i] = 180; Drive_SP[i] = 100; }
      }
      else if (lx >= 30 && (ly >= -50 && ly <= 50) || !incomingData.stat[2]) {
        Serial.println("↗️ KANAN");
        for (int i = 0; i < 4; i++) { Steer_SP[i] = 90; Drive_SP[i] = 100; }
      }
      else if (lx <= -30 && (ly >= -50 && ly <= 50) || !incomingData.stat[3]) {
        Serial.println("↖️ KIRI");
        for (int i = 0; i < 4; i++) { Steer_SP[i] = -90; Drive_SP[i] = 100; }
      }
      else if (ly <= -30 && (lx >= 31 && lx <= 127)) {
        Serial.println("↗️ DIAGONAL KANAN DEPAN");
        for (int i = 0; i < 4; i++) { Steer_SP[i] = 45; Drive_SP[i] = 100; }
      }
      else if (ly >= 30 && (lx >= 31 && lx <= 127)) {
        Serial.println("↘️ DIAGONAL KANAN BELAKANG");
        for (int i = 0; i < 4; i++) { Steer_SP[i] = 135; Drive_SP[i] = 100; }
      }
      else if (ly <= -50 && (lx <= -36 && lx >= -128)) {
        Serial.println("↖️ DIAGONAL KIRI DEPAN");
        for (int i = 0; i < 4; i++) { Steer_SP[i] = -45; Drive_SP[i] = 100; }
      }
      else if (ly >= 35 && (lx <= -36 && lx <= -128)) {
        Serial.println("↙️ DIAGONAL KIRI BELAKANG");
        for (int i = 0; i < 4; i++) { Steer_SP[i] = -135; Drive_SP[i] = 100; }
      }
      else if (rx <= -40) {
        Serial.println("↺ ROTASI KIRI");
        int deg[4] = {-45, -135, 45, 135};
        for (int i = 0; i < 4; i++) { Steer_SP[i] = deg[i]; Drive_SP[i] = 200; }
      }
      else if (rx >= 40) {
        Serial.println("↻ ROTASI KANAN");
        int deg[4] = {135, 45, -135, -45};
        for (int i = 0; i < 4; i++) { Steer_SP[i] = deg[i]; Drive_SP[i] = 200; }
      }
      else {
        for (int i = 0; i < 4; i++) Drive_SP[i] = 0;
      }
      
      if (Steer_SP[0] > 0) eepromVal = -1;
      EEPROM.write(addsDegree, eepromVal);
      EEPROM.commit();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

#endif
