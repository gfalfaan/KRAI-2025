// ========================================================================================
// ESP32 RECEIVER I2C - FILE UTAMA
// Remote Joystick ESP-NOW → ESP32 → 4 STM32 (I2C)
// ========================================================================================

#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <Wire.h>

// ===== INCLUDE =====
#include "config.h"
#include "variables.h"
#include "EEPROM_Functions.h"
#include "Display_Functions.h"
#include "Joystick_Functions.h"
#include "ESPNOW_Callback.h"
#include "Main_Task.h"
#include "I2C_Task.h"

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  printHeader();
  
  pinMode(pinBuzzer, OUTPUT);
  digitalWrite(pinBuzzer, LOW);
  
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("❌ EEPROM FAIL!");
    delay(1000);
    ESP.restart();
  }
  
  zeroDegree = EEPROM.read(addsDegree);
  for (int i = 0; i < 4; i++) Steer_SP[i] = zeroDegree;
  Serial.println("📍 Zero: " + String(zeroDegree));
  
  storedMac = loadRemoteIndex();
  incomingData.remoteIndex = storedMac;
  
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW FAIL!");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("✅ I2C: SDA=" + String(I2C_SDA) + " SCL=" + String(I2C_SCL));
  printStatus();
  
  xTaskCreatePinnedToCore(I2CCommunicationSendTask, "I2C", 10000, NULL, 1, &I2CTask, 0);
  xTaskCreatePinnedToCore(mainApplicationTask, "Main", 10000, NULL, 1, &mainTask, 1);
  
  Serial.println("✅ Tasks started!\n");
}

// ======================== LOOP ========================
void loop() {
  vTaskDelay(1 / portTICK_PERIOD_MS);
}
