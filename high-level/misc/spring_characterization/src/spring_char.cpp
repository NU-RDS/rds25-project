#include <Arduino.h>
#include <SPI.h>
#include "HX711.h"

// HX711 Configuration 
#define SCK 27
#define DT 26
#define BAUD_RATE 9600
#define CALIBRATION_FACTOR 120.75

HX711 scale;
// Scale weight reading counter
int count = 1;

// SPI Encoder Configuration 
#define NOP 0x0000
#define ERRFL 0x0001
#define PROG 0x0003
#define DIAAGC 0x3FFC
#define MAG 0X3FFD
#define ANGLEUNC 0x3FFE
#define ANGLECOM 0x3FFF
#define SPI_MODE SPI_MODE1

// Define CS pins for both encoders
const int cs1 = 10;  // First encoder
const int cs2 = 37;  // Second encoder

SPISettings settings(10000000, MSBFIRST, SPI_MODE);
uint16_t nop, error, cmd, diag;

// Function to convert raw encoder value to degrees
float convertTo360(uint16_t rawValue) {
  float angle = (rawValue * 360.0) / 16383.0;
  return angle;
}

// Function to read angle from a specific encoder
uint16_t readEncoderRaw(int csPin) {
  uint16_t pos_temp;

  // Read angle command
  cmd = (0b11 << 14) | ANGLECOM;
  digitalWriteFast(csPin, LOW);
  nop = SPI.transfer16(cmd);
  digitalWriteFast(csPin, HIGH);
  delayNanoseconds(400);

  // Read error flags
  cmd = (0b01 << 14) | ERRFL;
  digitalWriteFast(csPin, LOW);
  pos_temp = SPI.transfer16(cmd);
  digitalWriteFast(csPin, HIGH);
  delayNanoseconds(400);

  // Read diagnostics
  cmd = (0b11 << 14) | DIAAGC;
  digitalWriteFast(csPin, LOW);
  error = SPI.transfer16(cmd);
  digitalWriteFast(csPin, HIGH);
  delayNanoseconds(400);

  // NOP command
  cmd = (0b11 << 14) | NOP;
  digitalWriteFast(csPin, LOW);
  diag = SPI.transfer16(cmd);
  digitalWriteFast(csPin, HIGH);

  // Extract the 14-bit angle value and return it
  return pos_temp & 0b11111111111111;
}

// Initialising the teensy, load cell & encoders
void setup() {
  // Start Serial
  Serial.begin(BAUD_RATE);
  unsigned long timeout = millis();
  while (!Serial && millis() - timeout < 3000);

  Serial.println("Initializing HX711 and SPI Encoders...");

  // HX711 Init
  scale.begin(DT, SCK);

  // Calibration values
  scale.set_scale(CALIBRATION_FACTOR);

  // Set the scale to 0
  Serial.println("Tare. Remove any weights from the scale.");
  delay(10000);
  scale.tare();
  Serial.println("Please place weight on scales");

  // SPI encoder init
  pinMode(cs1, OUTPUT);
  pinMode(cs2, OUTPUT);
  digitalWriteFast(cs1, HIGH);
  digitalWriteFast(cs2, HIGH);

  SPI.begin(); 
}

// Calculating & displaying the weight & encoder readings
void loop() {
  // Reading scale input
  float weight = scale.get_units();

  // Record start time
  SPI.beginTransaction(settings);

  // Read from first encoder
  uint16_t rawAngle1 = readEncoderRaw(cs1);
  float angleDegrees1 = convertTo360(rawAngle1);

  // Read from second encoder
  uint16_t rawAngle2 = readEncoderRaw(cs2);
  float angleDegrees2 = convertTo360(rawAngle2);

  SPI.endTransaction();

  // Print results
  Serial.print(angleDegrees1, 2);
  Serial.print(",");
  Serial.print(angleDegrees2, 2);
  Serial.print(",");
  Serial.println(weight, 3);

  delay(100);
}
