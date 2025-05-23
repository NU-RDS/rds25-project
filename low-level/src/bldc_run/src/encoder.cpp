#include <Arduino.h>
#include <SPI.h>

#define NOP 0x0000
#define ERRFL 0x0001
#define PROG 0x0003
#define DIAAGC 0x3FFC
#define MAG 0X3FFD
#define ANGLEUNC 0x3FFE
#define ANGLECOM 0x3FFF
#define SPI_MODE SPI_MODE1

// Define CS pins for both encoders
const int cs1 = 7;  // First encoder
const int cs2 = 37;  // Second encoder

SPISettings settings(10000000, MSBFIRST, SPI_MODE);
uint16_t nop, error, cmd, diag;

float convertTo360(uint16_t rawValue) {
  float angle = (rawValue * 360.0) / 16383.0;
  return angle;
}

// Function to read angle from a specific encoder
uint16_t readEncoderRaw(int csPin) {
  uint16_t pos_temp;
  
  // Read angle command
  
  cmd = (0b11<<14) | ANGLECOM;
  digitalWriteFast(csPin, LOW);
  nop = SPI.transfer16(cmd);
  digitalWriteFast(csPin, HIGH);
  delayNanoseconds(400);
  
  // Read error flags
  cmd = (0b01<<14) | ERRFL;
  digitalWriteFast(csPin, LOW);
  pos_temp = SPI.transfer16(cmd);
  digitalWriteFast(csPin, HIGH);
  delayNanoseconds(400);
  
  // Read diagnostics
  cmd = (0b11<<14) | DIAAGC;
  digitalWriteFast(csPin, LOW);
  error = SPI.transfer16(cmd);
  digitalWriteFast(csPin, HIGH);
  delayNanoseconds(400);
  
  // NOP command
  cmd = (0b11<<14) | NOP;
  digitalWriteFast(csPin, LOW);
  diag = SPI.transfer16(cmd);
  digitalWriteFast(csPin, HIGH);
  
  // Extract the 14-bit angle value and return it
  return pos_temp & 0b11111111111111;
}

void setup() {
  pinMode(cs1, OUTPUT);
  pinMode(cs2, OUTPUT);
  pinMode(36, OUTPUT);  // Set pin 36 as output for CS2
  digitalWriteFast(36, HIGH);  // Initialize CS2 pin HIGH
  digitalWriteFast(cs1, HIGH);  // Initialize CS pins HIGH
  digitalWriteFast(cs2, HIGH);
  
  Serial.begin(9600);  // Initialize serial communication
  
  SPI.begin(); 
}

void loop() {
  // Record start time
  
  SPI.beginTransaction(settings);
  
  // Read from second encoder
  uint16_t rawAngle1 = readEncoderRaw(cs1);
  float angleDegrees1 = convertTo360(rawAngle1);
  uint16_t rawAngle2 = readEncoderRaw(cs2);
  float angleDegrees2 = convertTo360(rawAngle2);
  

  SPI.endTransaction();
  
  // Calculate elapsed time
  
  // Print results
  Serial.print("Encoder 1 - Degrees: ");
  Serial.println(angleDegrees1, 2);
  
  //Serial.print("Encoder 2 - Degrees: ");
  //Serial.println(angleDegrees2, 2);
  
  // Print timing information
  // Serial.print(" | Loop Time: ");
  // Serial.print(elapsedTime);
  // Serial.println(" microseconds");
  
  delay(100);
}