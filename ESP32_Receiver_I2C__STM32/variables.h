#ifndef VARIABLES_H
#define VARIABLES_H

#include "config.h"

// ======================== VARIABEL GLOBAL ========================

// ESP-NOW
extern struct_message incomingData;
extern uint8_t storedMac;

// MAC Address Remote
extern uint8_t broadcastAddress[4][6];

// I2C Slave Address
extern int SLAVE_ADDR_MODULE[4];

// Swerve Setpoint
extern int Steer_SP[4];
extern int Drive_SP[4];
extern String dataModule[4];

// Joystick
extern short int lx, ly, rx;

// EEPROM
extern int8_t zeroDegree, eepromVal;

// Status
extern unsigned long packetCount;
extern unsigned long lastReceiveTime;
extern bool failsafeTriggered;
extern bool connectionStatus;
extern bool remoteConnected;

// Task Handle
extern TaskHandle_t mainTask;
extern TaskHandle_t I2CTask;

// ===== DEFINISI VARIABEL (Hanya di 1 file!) =====
#ifndef VARIABLES_IMPLEMENTATION
#define VARIABLES_IMPLEMENTATION

struct_message incomingData;
uint8_t storedMac = 0;

uint8_t broadcastAddress[4][6] = {
  {0x80, 0x7D, 0x3A, 0xEA, 0xB1, 0x98},
  {0xE8, 0x6B, 0xEA, 0xD4, 0xA3, 0xA0},
  {0x80, 0x7D, 0x3A, 0xB9, 0x1F, 0xB4},
  {0xA4, 0xCF, 0x12, 0x42, 0xAC, 0x50}
};

int SLAVE_ADDR_MODULE[4] = {10, 11, 12, 13};
int Steer_SP[4] = {0, 0, 0, 0};
int Drive_SP[4] = {0, 0, 0, 0};
String dataModule[4];

short int lx = 0, ly = 0, rx = 0;
int8_t zeroDegree = 0, eepromVal = 0;

unsigned long packetCount = 0;
unsigned long lastReceiveTime = 0;
bool failsafeTriggered = false;
bool connectionStatus = false;
bool remoteConnected = false;

TaskHandle_t mainTask;
TaskHandle_t I2CTask;

#endif // VARIABLES_IMPLEMENTATION

#endif // VARIABLES_H
