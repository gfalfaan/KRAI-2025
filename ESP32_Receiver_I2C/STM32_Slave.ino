// ========================================================================================
// PROGRAM STM32 - 1 MODUL SWERVE (PID + I2C Slave)
// Upload program ini ke 4 STM32, ganti ALAMAT I2C saja!
// ========================================================================================

#include <Arduino.h>
#include <Wire.h>

// ======================== KONFIGURASI ========================

// stm32
#define I2C_SLAVE_ADDRESS 10  // STM32 #1 = 10, #2 = 11, #3 = 12, #4 = 13

// Pin Motor Steer
#define pin1 PA3     // PWM Steer A
#define pin2 PA1     // PWM Steer B
#define enAM1 PB9    // Encoder Steer A (Interrupt)
#define enBM1 PB8    // Encoder Steer B

// Pin Motor Drive
#define pin3 PA6     // PWM Drive A
#define pin4 PA7     // PWM Drive B
#define enAM2 PB7    // Encoder Drive A (Interrupt)
#define enBM2 PB6    // Encoder Drive B

// Pin Hall Sensor (Homing)
#define pinHall PB5

// Pin LED Status
#define pinLed PC13

// ======================== KONSTANTA PID/PD ========================

// Koefisien PD Steer
const double Kp_steer = 1.175;
const double Kd_steer = 0.472;

// Koefisien PID Drive
const double Kp_drive = 0.09;
const double Ki_drive = 0.0905;
const double Kd_drive = 0.0059;

// Konstanta Encoder
const double PPR_steer = 330;          
const double GEAR_RATIO_steer = 2.285714;
const double PPR_drive = 200;           
const double GEAR_RATIO_drive = 0.432;
const double SAMPLING_TIME = 100;      // ms

// ======================== VARIABEL GLOBAL ========================

// Encoder Count
volatile long encoderCount1 = 0;  // Steer
volatile long encoderCount2 = 0;  // Drive

// Setpoint dari ESP32 (via I2C)
short int degree = 0;   // Setpoint Sudut
short int speed = 0;    // Setpoint Kecepatan

// Variabel PID
double lastError_steer = 0;
double lastRpm = 0;
double eIntegral_drive = 0;

// ======================== INTERRUPT ENCODER ========================

void EN_steer() {
  if (digitalRead(enBM1) == LOW) encoderCount1++;
  else encoderCount1--;
}

void EN_drive() {
  if (digitalRead(enBM2) == LOW) encoderCount2++;
  else encoderCount2--;
}

// ======================== KONTROL PD STEERING ========================

void TaskControlAngle(float setPoint) {
  double degreePerPulse = (setPoint / 360.0) * (PPR_steer * GEAR_RATIO_steer);
  double error = degreePerPulse - encoderCount1;
  double derivative_steer = error - lastError_steer;
  double pDOutput_steer = (Kp_steer * error) + (Kd_steer * derivative_steer);

  pDOutput_steer = constrain(pDOutput_steer, -255, 255);

  double PwM_L = (pDOutput_steer >= 0) ? 0 : abs(pDOutput_steer);
  double PwM_R = (pDOutput_steer >= 0) ? abs(pDOutput_steer) : 0;

  analogWrite(pin1, PwM_L);
  analogWrite(pin2, PwM_R);

  lastError_steer = error;
}

// ======================== KONTROL PID DRIVE ========================

void TaskControlSpeed(short int setpoint) {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  double rev = (double)encoderCount2 / (PPR_drive * GEAR_RATIO_drive);
  double deltaTime = (double)(currentTime - lastTime);
  double currentRPM = (deltaTime > 0) ? (rev / (deltaTime / 1000.0)) * 60.0 : 0.0; 
  
  encoderCount2 = 0;
  lastTime = currentTime;

  double error = setpoint - currentRPM;
  eIntegral_drive = eIntegral_drive + error;
  double eDerivative_drive = (currentRPM - lastRpm);
  double pidOutput_drive = (Kp_drive * error) + (Ki_drive * eIntegral_drive) + (Kd_drive * eDerivative_drive);

  pidOutput_drive = constrain(pidOutput_drive, -255, 255);

  double PwM_L, PwM_R;
  if (setpoint == 0) {
    PwM_L = 0;
    PwM_R = 0;
    eIntegral_drive = 0;
  } else {
    PwM_L = (pidOutput_drive >= 0) ? abs(pidOutput_drive) : 0;
    PwM_R = (pidOutput_drive >= 0) ? 0 : abs(pidOutput_drive);
  }

  analogWrite(pin3, PwM_L);
  analogWrite(pin4, PwM_R);

  lastRpm = currentRPM;
}

// ======================== CALLBACK I2C RECEIVE ========================

void receiveEvent(int howMany) {
  String receivedData = "";
  
  // Baca semua data yang dikirim
  while (Wire.available()) {
    char c = Wire.read();
    receivedData += c;
  }
  
  // Parse data "Speed#Degree"
  int separatorIndex = receivedData.indexOf('#');
  
  if (separatorIndex > 0) {
    String speedStr = receivedData.substring(0, separatorIndex);
    String degreeStr = receivedData.substring(separatorIndex + 1);
    
    speed = speedStr.toInt();
    degree = degreeStr.toInt();
    
    // Debug (opsional)
    Serial.print("Received: Speed=");
    Serial.print(speed);
    Serial.print(" Degree=");
    Serial.println(degree);
  }
}

// ======================== SETUP ========================

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.print("║  STM32 SWERVE MODULE - Addr ");
  Serial.print(I2C_SLAVE_ADDRESS);
  Serial.println("        ║");
  Serial.println("╚════════════════════════════════════════╝");

  // Setup LED
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);

  // Setup Motor Steer
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(enBM1, INPUT);
  pinMode(enAM1, INPUT);

  // Setup Motor Drive
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(enBM2, INPUT);
  pinMode(enAM2, INPUT);

  // Setup Hall Sensor
  pinMode(pinHall, INPUT);

  // Attach Interrupt
  attachInterrupt(digitalPinToInterrupt(enAM1), EN_steer, RISING);
  attachInterrupt(digitalPinToInterrupt(enAM2), EN_drive, RISING);

  // ===== HOMING STEER MOTOR =====
  Serial.println("🏠 Homing steer motor...");
  while (digitalRead(pinHall) != LOW) {
    analogWrite(pin1, 0);
    analogWrite(pin2, 30);
  }
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);
  encoderCount1 = 0;
  encoderCount2 = 0;
  degree = 0;
  Serial.println("✅ Homing complete!");

  // ===== SETUP I2C SLAVE =====
  Wire.begin(I2C_SLAVE_ADDRESS);  // ⚠️ Alamat unik setiap STM32
  Wire.onReceive(receiveEvent);   // Register callback
  
  Serial.print("📡 I2C Slave Address: 0x");
  Serial.println(I2C_SLAVE_ADDRESS, HEX);
  Serial.println("✅ Ready!\n");
  
  // LED blink ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(pinLed, LOW);
    delay(200);
    digitalWrite(pinLed, HIGH);
    delay(200);
  }
}

// ======================== LOOP ========================

void loop() {
  static unsigned long lastTimePID_steer = 0;
  static unsigned long lastTimePID_drive = 0;
  unsigned long currentTime = millis();

  // Kontrol PD Steer (setiap 50ms)
  if ((currentTime - lastTimePID_steer) >= 50) {
    TaskControlAngle((float)degree);
    lastTimePID_steer = currentTime;
  }

  // Kontrol PID Drive (setiap 100ms)
  if ((currentTime - lastTimePID_drive) >= SAMPLING_TIME) {
    TaskControlSpeed(speed);
    lastTimePID_drive = currentTime;
  }
}