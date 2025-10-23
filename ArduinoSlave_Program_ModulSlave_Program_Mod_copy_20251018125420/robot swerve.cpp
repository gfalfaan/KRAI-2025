#include <Arduino.h>

// --- DEFINISI PIN STM32 --- //
/* Motor Steer */
#define pin1 PA3     
#define pin2 PA1     
#define enAM1 PB9    // Encoder Steer A (Interrupt)
#define enBM1 PB8    // Encoder Steer B (Digital Read)

/* Motor Drive */
#define pin3 PA6     
#define pin4 PA7     
#define enAM2 PB7    // Encoder Drive A (Interrupt)
#define enBM2 PB6    // Encoder Drive B (Digital Read)

#define pinHall PB5  // Sensor efek hall (Homing)
#define pinLed PC13  // LED Status

// --- KONSTANTA KONTROL & GEAR --- //
/* Koefisien PD Steer */
const double Kp_steer = 1.175;
const double Kd_steer = 0.472;

/* Koefisien PID Drive */
const double Kp_drive = 0.09;
const double Ki_drive = 0.0905;
const double Kd_drive = 0.0059;

// --- KONSTANTA UMUM & GEAR --- //
const double PPR_steer = 330;          
const double GEAR_RATIO_steer = 2.285714;
const double PPR_drive = 200;           
const double GEAR_RATIO_drive = 0.432;
const double SAMPLING_TIME = 100;      // Waktu sampling PID Drive (ms)

// --- UART SETUP --- //
#define SERIAL_BAUD 115200
#define DATA_SIZE 20
char receivedData[DATA_SIZE + 1];

// --- VARIABEL KONTROL & WAKTU --- //
volatile long encoderCount1 = 0;       
volatile long encoderCount2 = 0;       
double lastError_steer = 0;
short int degree = 0;                  // Setpoint Sudut dari Master (ESP32)
short int speed = 0;                   // Setpoint Kecepatan dari Master (ESP32)
double lastRpm = 0;
double eIntegral_drive = 0;
unsigned long lastTimePID_steer = 0;
unsigned long lastTimePID_drive = 0;
unsigned long lastTimeUART = 0;

// --- PROTOTIPE FUNGSI --- //
void EN_steer();
void EN_drive();
void receiveSerial(); 
void TaskControlAngle(float setPoint);
void TaskControlSpeed(short int setpoint);

// ========================================================================= //
// --- INTERRUPT ENCODER --- //

void EN_steer() {
  if (digitalRead(enBM1) == LOW) encoderCount1++;
  else encoderCount1--;
}

void EN_drive() {
  if (digitalRead(enBM2) == LOW) encoderCount2++;
  else encoderCount2--;
}

// ========================================================================= //
// --- KOMUNIKASI SERIAL DARI ESP32 --- //

void receiveSerial() {
  static int index = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      receivedData[index] = '\0';
      index = 0;

      // Parsing format "DriveSP#SteerSP"
      char *token = strtok(receivedData, "#");
      if (token != NULL) speed = atoi(token);
      token = strtok(NULL, "#");
      if (token != NULL) degree = atoi(token);

      // Debug tampilan
      Serial.print("Received DriveSP = ");
      Serial.print(speed);
      Serial.print(" | SteerSP = ");
      Serial.println(degree);

    } else if (index < DATA_SIZE - 1) {
      receivedData[index++] = c;
    }
  }
}

// ========================================================================= //
// --- KONTROL PD STEERING --- //

void TaskControlAngle(float setPoint) {
  static double PwM_L = 0;
  static double PwM_R = 0;

  double degreePerPulse = (setPoint / 360.0) * (PPR_steer * GEAR_RATIO_steer);
  double error = degreePerPulse - encoderCount1;
  double derivative_steer = error - lastError_steer;
  double pDOutput_steer = (Kp_steer * error) + (Kd_steer * derivative_steer);

  pDOutput_steer = constrain(pDOutput_steer, -255, 255);

  PwM_L = (pDOutput_steer >= 0) ? 0 : abs(pDOutput_steer);
  PwM_R = (pDOutput_steer >= 0) ? abs(pDOutput_steer) : 0;

  analogWrite(pin1, PwM_L);
  analogWrite(pin2, PwM_R);

  lastError_steer = error;
}

// ========================================================================= //
// --- KONTROL PID DRIVE --- //

void TaskControlSpeed(short int setpoint) {
  static double PwM_L = 0;
  static double PwM_R = 0;
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

// ========================================================================= //
// --- SETUP & LOOP --- //

void setup() {
  Serial.begin(SERIAL_BAUD); // UART komunikasi dengan ESP32
  Serial.println("STM32 UART control ready...");

  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(enBM1, INPUT);
  pinMode(enAM1, INPUT);

  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  pinMode(enBM2, INPUT);
  pinMode(enAM2, INPUT);

  pinMode(pinHall, INPUT);
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);
  delay(500);

  attachInterrupt(digitalPinToInterrupt(enAM1), EN_steer, RISING);
  attachInterrupt(digitalPinToInterrupt(enAM2), EN_drive, RISING);

  // Homing posisi steer
  while (digitalRead(pinHall) != LOW) {
    short int dirA = (degree >= 0) ? 30 : 0;
    short int dirB = (degree >= 0) ? 0 : 30;
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

void loop() {
  unsigned long currentTime = millis();

  // Baca data dari ESP32 setiap 50 ms
  if ((currentTime - lastTimeUART) >= 50) {
    receiveSerial();
    lastTimeUART = currentTime;
  }

  // Kontrol PD steer
  if ((currentTime - lastTimePID_steer) >= 50) {
    TaskControlAngle((float)degree);
    lastTimePID_steer = currentTime;
  }

  // Kontrol PID drive
  if ((currentTime - lastTimePID_drive) >= SAMPLING_TIME) {
    TaskControlSpeed(speed);
    lastTimePID_drive = currentTime;
  }
}
