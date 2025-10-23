#include <Wire.h>

// Definisikan pin-pin yang digunakan untuk motor steer dan drive
#define pin1 PA3
#define pin2 PA1
#define enAM1 PB8   // Encoder 1A
#define enBM1 PB9   // Encoder 1B

#define pin3 PA6
#define pin4 PA7
#define enAM2 PB7   // Encoder 2A
#define enBM2 PB6   // Encoder 2B

#define pinHall PB5 // Hall sensor steer
#define pinLed PC13

// Konstanta kontrol
const double Kp_steer = 1.175;
const double Kd_steer = 0.472;

const double Kp_drive = 0.09;
const double Ki_drive = 0.0905;
const double Kd_drive = 0.0059;

// Encoder dll
const double PPR_steer = 330;
const double GEAR_RATIO_steer = 2.285714;
const double PPR_drive = 200;
const double GEAR_RATIO_drive = 0.432;
const double SAMPLING_TIME = 100;

// I2C Wire2 di STM32
TwoWire Wire2(PB11, PB10); // SDA, SCL (cek wiring hardwaremu!)

// Alamat slave dan buffer data
#define SLAVE_ADDR 10           // GANTI ini untuk tiap modul: 10 / 11 / 12 / 13
#define DATA_SIZE 20
char receivedData[DATA_SIZE + 1];

// Waktu interval
unsigned long lastTimePID_steer = 0;
unsigned long lastTimePID_drive = 0;
unsigned long lastTimeI2C = 0;

// Variabel steer
volatile long encoderCount1 = 0;
double lastError_steer = 0;
short int degree = 0;

// Variabel drive
volatile long encoderCount2 = 0;
short int speed = 0;
double lastRPM = 0;
double eIntegral_drive = 0;

// ---------- Interrupt untuk Encoder Steer ----------
void EN_steer() {
  if (digitalRead(enBM1) == LOW) encoderCount1++;
  else encoderCount1--;
}

// ---------- Interrupt untuk Encoder Drive ----------
void EN_drive() {
  if (digitalRead(enBM2) == LOW) encoderCount2++;
  else encoderCount2--;
}

// ---------- Callback I2C terima data dari Master ----------
void receiveEvent(int dataSize) {
  int index = 0;
  while (Wire2.available()) {
    char receivedChar = Wire2.read();
    receivedData[index] = receivedChar;
    index++;
    if (index >= DATA_SIZE) break;
  }
  receivedData[index] = '\0';

  // Parse: "speed#degree"
  char *token = strtok(receivedData, "#");
  if (token != NULL) {
    speed = atoi(token);
    token = strtok(NULL, "#");
    if (token != NULL) {
      degree = atoi(token);
    }
  }
}

// ---------- PID Steering ----------
void TaskControlAngle(float setPoint) {
  static double PWM_L = 0, PWM_R = 0;
  int degreePerPulse = (setPoint / 360.0) * (PPR_steer * GEAR_RATIO_steer);
  double error = degreePerPulse - encoderCount1;
  double dError = error - lastError_steer;
  double pDOutput_steer = (Kp_steer * error) + (Kd_steer * dError);

  pDOutput_steer = constrain(pDOutput_steer, -255, 255);

  PWM_L = (pDOutput_steer >= 0) ? 0 : abs(pDOutput_steer);
  PWM_R = (pDOutput_steer >= 0) ? abs(pDOutput_steer) : 0;

  analogWrite(pin1, PWM_L);
  analogWrite(pin2, PWM_R);

  lastError_steer = error;
}

// ---------- PID Drive ----------
void TaskControlSpeed(short int setpoint) {
  static double PWM_L = 0, PWM_R = 0;
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  double rev = (double)encoderCount2 / (PPR_drive * GEAR_RATIO_drive);
  double currentRPM = (rev / ((currentTime - lastTime) / 1000.0)) * 60.0;
  encoderCount2 = 0;
  lastTime = currentTime;

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

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
//Konfigurasi alamt slave fungsi terima data
  Wire2.begin(SLAVE_ADDR);
  Wire2.onReceive(receiveEvent);

  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(enAM1, INPUT);
  pinMode(enBM1, INPUT);

  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(enAM2, INPUT);
  pinMode(enBM2, INPUT);

  pinMode(pinHall, INPUT);

  attachInterrupt(digitalPinToInterrupt(enAM1), EN_steer, RISING);
  attachInterrupt(digitalPinToInterrupt(enAM2), EN_drive, RISING);

  // Homing motor steer
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

// ---------- Loop ----------
void loop() {
  unsigned long currentTime = millis();

  // (Jeda pembacaan I2C)
  if (currentTime - lastTimeI2C >= 100) {
    lastTimeI2C = currentTime;
    // Data speed/degree sudah terupdate otomatis via receiveEvent
  }

  // PID kontrol sudut steer
  if (currentTime - lastTimePID_steer >= 50) {
    TaskControlAngle(double(degree));
    lastTimePID_steer = currentTime;
  }

  // PID kontrol kecepatan drive
  if (currentTime - lastTimePID_drive >= SAMPLING_TIME) {
    TaskControlSpeed(speed);
    lastTimePID_drive = currentTime;
  }
}