#include <SPI.h>

// Pin definitions for AS5047 encoder - use primary SPI bus (SPI0)
const int ENCODER_CS = 10;    // Connected to encoder nCS
const int ENCODER_MISO = 12;  // Connected to encoder MISO/DATA
const int ENCODER_MOSI = 11;  // Connected to encoder MOSI (optional)
const int ENCODER_CLK = 13;   // Connected to encoder SCK

// Pin definitions for ODrive - use secondary SPI bus (SPI1)
const int ODRIVE_CS = 0;      // Connected to ODrive SPI_nCS
const int ODRIVE_MISO = 1;    // Connected to ODrive SPI_MISO
const int ODRIVE_MOSI = 26;   // Connected to ODrive SPI_MOSI (optional)
const int ODRIVE_CLK = 27;    // Connected to ODrive SPI_SCK

// SPI settings for AS5047 encoder
SPISettings encoderSettings(1000000, MSBFIRST, SPI_MODE1);

// Buffer to store encoder data
volatile uint16_t rawEncoderValue = 0;
volatile bool newDataReady = false;


// Read encoder position
uint16_t readEncoder() {
  uint16_t value = 0;
  
  SPI.beginTransaction(encoderSettings);
  digitalWrite(ENCODER_CS, LOW); // Select encoder
  
  // Send read command (0x3FFF is the position read command for AS5047)
  SPI.transfer16(0x3FFF);
  
  // Release and reselect to get the actual data
  digitalWrite(ENCODER_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(ENCODER_CS, LOW);
  
  // Read the actual position value
  value = SPI.transfer16(0);
  
  digitalWrite(ENCODER_CS, HIGH); // Deselect encoder
  SPI.endTransaction();
  
  return value;
}

// Interrupt handler for ODrive SPI requests
void handleODriveRequest() {
  // When ODrive selects us (CS goes LOW), send the latest encoder value
  noInterrupts(); // Disable interrupts during critical SPI timing
  
  // Send the latest encoder value bit by bit (MSB first)
  uint16_t valueToSend = rawEncoderValue;
  for (int i = 15; i >= 0; i--) {
    // Wait for clock rising edge
    while (digitalRead(ODRIVE_CLK) == LOW);
    
    // Set output bit
    digitalWrite(ODRIVE_MISO, (valueToSend >> i) & 0x01);
    
    // Wait for clock falling edge
    while (digitalRead(ODRIVE_CLK) == HIGH);
  }
  
  interrupts(); // Re-enable interrupts
  
  // Log transfer for debugging (but do this outside the interrupt handler)
  newDataReady = true;
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait for serial to connect
  
  // Configure encoder SPI pins (SPI0)
  pinMode(ENCODER_CS, OUTPUT);
  digitalWrite(ENCODER_CS, HIGH); // Deselect initially
  SPI.begin(); // Initialize primary hardware SPI for encoder
  
  pinMode(ODRIVE_CS, INPUT);  
  pinMode(ODRIVE_MISO, OUTPUT);
  pinMode(ODRIVE_MOSI, INPUT);
  pinMode(ODRIVE_CLK, INPUT);
  
  // Set up interrupt for ODrive CS pin
  attachInterrupt(digitalPinToInterrupt(ODRIVE_CS), handleODriveRequest, FALLING);
  
  Serial.println("SPI Relay initialized with hardware SPI pins");
  Serial.println("Encoder: CS=10, MISO=12, MOSI=11, CLK=13");
  Serial.println("ODrive: CS=0, MISO=1, MOSI=26, CLK=27");
}

void loop() {
  // Continuously read encoder
  rawEncoderValue = readEncoder();
  
  // Print encoder reading and transfers periodically
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) {
    Serial.print("Encoder value: 0x");
    Serial.println(rawEncoderValue, HEX);
    lastPrintTime = millis();
    
    if (newDataReady) {
      Serial.print("Sent to ODrive: 0x");
      Serial.println(rawEncoderValue, HEX);
      newDataReady = false;
    }
  }
  
  // Small delay between readings
  delay(1);
}