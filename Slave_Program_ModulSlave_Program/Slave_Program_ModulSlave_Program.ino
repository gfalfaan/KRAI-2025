// ========================================================================================
// STM32 I2C SLAVE - MOTOR CONTROL 
// ESP32 Master → STM32 Slave → Motor Steer + Motor Drive
// ========================================================================================

#include <Wire.h>

// ======================== PIN KONFIGURASI ===============================================

// Pin Motor Steer (untuk belok/steering)
#define pin1 PA3   // Motor Steer A
#define pin2 PA1   // Motor Steer B
#define enAM1 PB8  // Encoder Steer A
#define enBM1 PB9  // Encoder Steer B

// Pin Motor Drive (untuk maju/mundur)
#define pin3 PA6   // Motor Drive A
#define pin4 PA7   // Motor Drive B
#define enAM2 PB7  // Encoder Drive A
#define enBM2 PB6  // Encoder Drive B

// Pin Hall Sensor dan LED
#define pinHall PB5  // Hall sensor untuk homing
#define pinLed PC13  // LED onboard

// ======================== KONFIGURASI I2C (SESUAI ROBOT ASLI) ========================

// Pin I2C (Wire2)
#define I2C_SDA PB11
#define I2C_SCL PB10

// Alamat I2C Slave (GANTI untuk tiap modul: 10, 11, 12, 13)
#define SLAVE_ADDR 10

// Buffer data
#define DATA_SIZE 20

// ======================== KONSTANTA KONTROL PID ========================

// PID Steering (PD Controller)
const double Kp_steer = 1.175;
const double Kd_steer = 0.472;

// PID Drive (PID Controller)
const double Kp_drive = 0.09;
const double Ki_drive = 0.0905;
const double Kd_drive = 0.0059;

// Encoder dan Gear Ratio
const double PPR_steer = 330;
const double GEAR_RATIO_steer = 2.285714;
const double PPR_drive = 200;
const double GEAR_RATIO_drive = 0.432;
const double SAMPLING_TIME = 100;

// ======================== VARIABEL GLOBAL ========================

TwoWire Wire2(I2C_SDA, I2C_SCL);  // Objek I2C custom

char receivedData[DATA_SIZE + 1];
short int degree = 0;   // Target sudut steering (dari ESP32)
short int speed = 0;    // Target kecepatan drive (dari ESP32)

// Variabel Encoder Steer
volatile long encoderCount1 = 0;
double lastError_steer = 0;

// Variabel Encoder Drive
volatile long encoderCount2 = 0;
double lastRPM = 0;
double eIntegral_drive = 0;

// Timing
unsigned long lastTimePID_steer = 0;
unsigned long lastTimePID_drive = 0;

// ======================== INTERRUPT ENCODER ========================

void EN_steer() {
  if (digitalRead(enBM1) == LOW) encoderCount1++;
  else encoderCount1--;
}

void EN_drive() {
  if (digitalRead(enBM2) == LOW) encoderCount2++;
  else encoderCount2--;
}

// ======================== CALLBACK I2C ========================

void receiveEvent(int dataSize) {
  int index = 0;
  
  // Baca data dari ESP32
  while (Wire2.available()) {
    char receivedChar = Wire2.read();
    receivedData[index] = receivedChar;
    index++;
    if (index >= DATA_SIZE) break;
  }
  receivedData[index] = '\0';

  // Parse format: "speed#degree"
  char *token = strtok(receivedData, "#");
  if (token != NULL) {
    speed = atoi(token);           // Kecepatan motor drive
    token = strtok(NULL, "#");
    if (token != NULL) {
      degree = atoi(token);        // Sudut steering
    }
  }
}

// ======================== PID STEERING (KONTROL SUDUT) ========================

void TaskControlAngle(float setPoint) {
  static double PWM_L = 0, PWM_R = 0;
  
  // Hitung target pulse encoder
  int degreePerPulse = (setPoint / 360.0) * (PPR_steer * GEAR_RATIO_steer);
  
  // PD Controller
  double error = degreePerPulse - encoderCount1;
  double dError = error - lastError_steer;
  double pDOutput_steer = (Kp_steer * error) + (Kd_steer * dError);
  
  pDOutput_steer = constrain(pDOutput_steer, -255, 255);
  
  // Set PWM motor
  PWM_L = (pDOutput_steer >= 0) ? 0 : abs(pDOutput_steer);
  PWM_R = (pDOutput_steer >= 0) ? abs(pDOutput_steer) : 0;
  
  analogWrite(pin1, PWM_L);
  analogWrite(pin2, PWM_R);
  
  lastError_steer = error;
}

// ======================== PID DRIVE (KONTROL KECEPATAN) ========================

void TaskControlSpeed(short int setpoint) {
  static double PWM_L = 0, PWM_R = 0;
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  
  // Hitung RPM dari encoder
  double rev = (double)encoderCount2 / (PPR_drive * GEAR_RATIO_drive);
  double currentRPM = (rev / ((currentTime - lastTime) / 1000.0)) * 60.0;
  encoderCount2 = 0;
  lastTime = currentTime;
  
  // PID Controller
  double error = setpoint - currentRPM;
  eIntegral_drive += error;
  double eDerivative_drive = currentRPM - lastRPM;
  double pidOutput_drive = (Kp_drive * error) + (Ki_drive * eIntegral_drive) + (Kd_drive * eDerivative_drive);
  
  pidOutput_drive = constrain(pidOutput_drive, -255, 255);
  
  if (setpoint != 0) {
    PWM_L = (pidOutput_drive >= 0) ? abs(pidOutput_drive) : 0;
    PWM_R = (pidOutput_drive >= 0) ? 0 : abs(pidOutput_drive);
  } else {
    PWM_L = 0;
    PWM_R = 0;
    pidOutput_drive = 0;
    eIntegral_drive = 0;
    lastRPM = 0;
  }
  
  analogWrite(pin3, PWM_L);
  analogWrite(pin4, PWM_R);
  
  lastRPM = currentRPM;
}

// ======================== SETUP ========================

void setup() {
  // Setup I2C Slave
  Wire2.begin(SLAVE_ADDR);
  Wire2.onReceive(receiveEvent);
  
  // Setup LED
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);
  
  // Setup Motor Steer
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(enAM1, INPUT);
  pinMode(enBM1, INPUT);
  
  // Setup Motor Drive
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(enAM2, INPUT);
  pinMode(enBM2, INPUT);
  
  // Setup Hall Sensor
  pinMode(pinHall, INPUT);
  
  // Attach Interrupt Encoder
  attachInterrupt(digitalPinToInterrupt(enAM1), EN_steer, RISING);
  attachInterrupt(digitalPinToInterrupt(enAM2), EN_drive, RISING);
  
  // Homing Motor Steer (cari posisi Hall sensor)
  while (digitalRead(pinHall) != LOW) {
    short int dirA = (degree >= 0) ? 0 : 30;
    short int dirB = (degree >= 0) ? 30 : 0;
    analogWrite(pin1, dirA);
    analogWrite(pin2, dirB);
  }
  degree = 0;
  digitalWrite(pinLed, LOW);
  analogWrite(pin1, 0);
  analogWrite(pin2, 0);
  delay(1000);
  
  encoderCount1 = 0;
  encoderCount2 = 0;
}

// ======================== LOOP ========================

void loop() {
  unsigned long currentTime = millis();
  
  // PID kontrol sudut steer (setiap 50ms)
  if (currentTime - lastTimePID_steer >= 50) {
    TaskControlAngle(double(degree));
    lastTimePID_steer = currentTime;
  }
  
  // PID kontrol kecepatan drive (setiap 100ms)
  if (currentTime - lastTimePID_drive >= SAMPLING_TIME) {
    TaskControlSpeed(speed);
    lastTimePID_drive = currentTime;
  }
}
