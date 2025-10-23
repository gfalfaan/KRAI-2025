#ifndef CONFIG_H
#define CONFIG_H

// ======================== KONFIGURASI ========================
#define DEBUG_MODE true
#define EEPROM_SIZE 512
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 100000
#define pinBuzzer 25
#define addsDegree 0
#define EEPROM_ADDR_REMOTE 100

// ======================== STRUKTUR DATA ========================
typedef struct struct_message {
    bool stat[15];
    int joyData[4];
    uint8_t remoteIndex;
} struct_message;

#endif
