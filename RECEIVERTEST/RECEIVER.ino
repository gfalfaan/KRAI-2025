#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>

// ======================== KONFIGURASI ========================
#define DEBUG_MODE true    // Set false untuk disable debug
#define STM_BAUD 115200
#define EEPROM_SIZE 512
#define FAILSAFE_TIMEOUT 1000

// Pin UART ke STM32
#define STM_TX_PIN 17  // ESP32 TX -> STM32 RX (PA10)
#define STM_RX_PIN 16  // ESP32 RX -> STM32 TX (PA9)

// ======================== STRUKTUR DATA ========================
typedef struct struct_message {
    bool stat[15];      // 15 tombol
    int joyData[4];     // 4 axis joystick
    uint8_t remoteIndex;
} struct_message;

struct_message incomingData;
uint8_t storedMac = 0;

// ======================== MAC ADDRESS REMOTE ========================
uint8_t broadcastAddress[4][6] = {
  {0x80, 0x7D, 0x3A, 0xEA, 0xB1, 0x98},
  {0xE8, 0x6B, 0xEA, 0xD4, 0xA3, 0xA0},
  {0x80, 0x7D, 0x3A, 0xB9, 0x1F, 0xB4},
  {0xA4, 0xCF, 0x12, 0x42, 0xAC, 0x50}
};

// ======================== VARIABEL MONITORING ========================
unsigned long packetCount = 0;
unsigned long lastPacketTime = 0;
float packetRate = 0;
struct_message lastReceivedData;
unsigned long lastReceiveTime = 0;
bool failsafeTriggered = false;
bool connectionStatus = false;

// ======================== FUNGSI EEPROM ========================
void saveRemoteIndex(uint8_t index) {
  EEPROM.write(100, index);
  EEPROM.commit();
  Serial.println("[EEPROM] Remote Index saved: " + String(index));
}

uint8_t loadRemoteIndex() {
  uint8_t index = EEPROM.read(100);
  if (index > 3) index = 0;
  return index;
}

// ======================== TAMPILAN HEADER ========================
void printHeader() {
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║   ESP32 RECEIVER - FULL MONITORING MODE       ║");
  Serial.println("╚════════════════════════════════════════════════╝");
}

// ======================== TAMPILAN STATUS ========================
void printStatus() {
  Serial.println("\n━━━━━━━━━━ SYSTEM STATUS ━━━━━━━━━━");
  Serial.print("📡 ESP32 MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("🔗 UART Baud: ");
  Serial.println(STM_BAUD);
  Serial.print("📍 TX Pin: GPIO ");
  Serial.print(STM_TX_PIN);
  Serial.print(" | RX Pin: GPIO ");
  Serial.println(STM_RX_PIN);
  Serial.print("💾 Stored Remote Index: ");
  Serial.println(storedMac);
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
  Serial.println("✅ Waiting for data from Remote Joystick...\n");
}

// ======================== TAMPILAN DATA DETAIL ========================
void printDetailedData(struct_message &data) {
  Serial.println("\n┌─────────────────────────────────────────┐");
  Serial.println("│        📦 DATA PACKET RECEIVED          │");
  Serial.println("└─────────────────────────────────────────┘");
  
  // Info Packet
  Serial.print("📊 Packet #");
  Serial.print(packetCount);
  Serial.print(" | Rate: ");
  Serial.print(packetRate, 1);
  Serial.println(" pkt/s");
  
  // Remote Index
  Serial.print("🎮 Remote Index: ");
  Serial.println(data.remoteIndex);
  
  // Joystick Data
  Serial.println("\n🕹️  JOYSTICK DATA:");
  Serial.print("   ├─ Joystick 1 X: ");
  printJoystickBar(data.joyData[0]);
  Serial.print("   ├─ Joystick 1 Y: ");
  printJoystickBar(data.joyData[1]);
  Serial.print("   ├─ Joystick 2 X: ");
  printJoystickBar(data.joyData[2]);
  Serial.print("   └─ Joystick 2 Y: ");
  printJoystickBar(data.joyData[3]);
  
  // Button Status
  Serial.println("\n🔘 BUTTON STATUS:");
  Serial.print("   ");
  for (int i = 0; i < 15; i++) {
    Serial.print(i < 9 ? " " : "");
    Serial.print(i);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("   ");
  for (int i = 0; i < 15; i++) {
    if (data.stat[i]) {
      Serial.print(" ○ "); // Not pressed
    } else {
      Serial.print(" ● "); // Pressed
    }
  }
  Serial.println();
  
  // Tampilkan button yang ditekan
  Serial.print("\n   ⚡ Pressed: ");
  bool anyPressed = false;
  for (int i = 0; i < 15; i++) {
    if (!data.stat[i]) {
      Serial.print("Btn");
      Serial.print(i);
      Serial.print(" ");
      anyPressed = true;
    }
  }
  if (!anyPressed) Serial.print("None");
  Serial.println();
  
  // Data CSV Format (untuk STM32)
  Serial.println("\n📤 CSV FORMAT (To STM32):");
  String csvData = "   ";
  for (int i = 0; i < 4; i++) csvData += String(data.joyData[i]) + ",";
  for (int i = 0; i < 15; i++) csvData += String(data.stat[i]) + ",";
  csvData += String(data.remoteIndex);
  Serial.println(csvData);
  
  Serial.println("\n" + String('─', 43));
}

// ======================== BAR GRAFIK JOYSTICK ========================
void printJoystickBar(int value) {
  Serial.print(value >= 0 ? " " : "");
  Serial.print(value);
  Serial.print(" [");
  
  int barLength = map(abs(value), 0, 4095, 0, 10);
  
  if (value < 0) {
    for (int i = 10; i > barLength; i--) Serial.print(" ");
    for (int i = 0; i < barLength; i++) Serial.print("◄");
    Serial.print("|");
    for (int i = 0; i < 10; i++) Serial.print(" ");
  } else if (value > 0) {
    for (int i = 0; i < 10; i++) Serial.print(" ");
    Serial.print("|");
    for (int i = 0; i < barLength; i++) Serial.print("►");
    for (int i = barLength; i < 10; i++) Serial.print(" ");
  } else {
    for (int i = 0; i < 10; i++) Serial.print(" ");
    Serial.print("|");
    for (int i = 0; i < 10; i++) Serial.print(" ");
  }
  
  Serial.println("]");
}

// ======================== KIRIM DATA KE STM32 ========================
void sendToSTM32(struct_message &data) {
  String dataLine = "";
  for (int i = 0; i < 4; i++) dataLine += String(data.joyData[i]) + ",";
  for (int i = 0; i < 15; i++) dataLine += String(data.stat[i]) + ",";
  dataLine += String(data.remoteIndex);
  
  Serial2.println(dataLine);
}

// ======================== FAILSAFE ========================
void checkFailsafe() {
  bool dataChanged = false;
  
  for (int i = 0; i < 4; i++) {
    if (incomingData.joyData[i] != lastReceivedData.joyData[i]) {
      dataChanged = true;
      break;
    }
  }
  
  if (!dataChanged) {
    for (int i = 0; i < 15; i++) {
      if (incomingData.stat[i] != lastReceivedData.stat[i]) {
        dataChanged = true;
        break;
      }
    }
  }
  
  if (dataChanged) {
    lastReceiveTime = millis();
    lastReceivedData = incomingData;
    failsafeTriggered = false;
    connectionStatus = true;
  }
  
  unsigned long timeSinceLastPacket = millis() - lastReceiveTime;
  
  if (connectionStatus && timeSinceLastPacket >= FAILSAFE_TIMEOUT) {
    if (!failsafeTriggered) {
      Serial.println("\n⚠️  FAILSAFE TRIGGERED! (No data for " + String(FAILSAFE_TIMEOUT) + "ms)");
      
      bool wasActive = false;
      for (int i = 0; i < 4; i++) {
        if (lastReceivedData.joyData[i] != 0) { wasActive = true; break; }
      }
      if (!wasActive) {
        for (int i = 0; i < 15; i++) {
          if (!lastReceivedData.stat[i]) { wasActive = true; break; }
        }
      }
      
      if (wasActive) {
        struct_message resetData;
        for (int i = 0; i < 4; i++) resetData.joyData[i] = 0;
        for (int i = 0; i < 15; i++) resetData.stat[i] = true;
        resetData.remoteIndex = incomingData.remoteIndex;
        
        sendToSTM32(resetData);
        Serial.println("🔄 Reset command sent to STM32");
        failsafeTriggered = true;
      }
    }
    
    if (timeSinceLastPacket >= 3000 && connectionStatus) {
      connectionStatus = false;
      Serial.println("\n❌ CONNECTION LOST! Waiting for reconnection...\n");
    }
  }
}

// ======================== CALLBACK ESP-NOW ========================
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  packetCount++;
  unsigned long currentTime = millis();
  if (lastPacketTime > 0) {
    unsigned long timeDiff = currentTime - lastPacketTime;
    packetRate = 1000.0 / timeDiff;
  }
  lastPacketTime = currentTime;
  
  memcpy(&incomingData, data, sizeof(incomingData));
  
  if (storedMac != incomingData.remoteIndex) {
    saveRemoteIndex(incomingData.remoteIndex);
    storedMac = incomingData.remoteIndex;
    Serial.println("\n🔄 Remote Index Changed! Restarting...\n");
    delay(100);
    ESP.restart();
  }
  
  printDetailedData(incomingData);
  sendToSTM32(incomingData);
}

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  printHeader();
  
  // Inisialisasi Serial2 ke STM32
  Serial2.begin(STM_BAUD, SERIAL_8N1, STM_RX_PIN, STM_TX_PIN);
  
  // Inisialisasi EEPROM
  EEPROM.begin(EEPROM_SIZE);
  storedMac = loadRemoteIndex();
  incomingData.remoteIndex = storedMac;
  
  // WiFi Mode
  WiFi.mode(WIFI_STA);
  
  // ESP-NOW Init
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed!");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  
  printStatus();
}

// ======================== LOOP ========================
void loop() {
  checkFailsafe();
  
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 5000 && !connectionStatus && packetCount == 0) {
    Serial.println("⏳ Still waiting for remote connection...");
    lastStatusPrint = millis();
  }
  
  delay(10);
}
