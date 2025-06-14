/*
 * Encoder Library for Magnetic Angle Sensors
 * Designed for AS5048 or similar SPI-based magnetic encoders
 */

 #ifndef ENCODER_HPP
 #define ENCODER_HPP
  
 #include <Arduino.h>
 #include <SPI.h>
 
 // Register definitions
 #define SPIx SPI 
 #define ENC_NOP 0x0000
 #define ENC_ERRFL 0x0001
 #define ENC_PROG 0x0003
 #define ENC_DIAAGC 0x3FFC
 #define ENC_MAG 0X3FFD
 #define ENC_ANGLEUNC 0x3FFE
 #define ENC_ANGLECOM 0x3FFF
 #define ENC_SPI_MODE SPI_MODE1
 #define SPI_CLOCK_SPEED 1000000
 
/**
 * @class Encoder
 * @brief Handles SPI communication with a rotary encoder and provides methods to read its value.
 *
 * This class manages the SPI interface to communicate with a rotary encoder device.
 * It allows initialization of the SPI bus, reading raw encoder values, and converting
 * those values to degrees.
 */
 class Encoder
 {
 private:
     int _cs;
     int _id;
     SPISettings settings;
 
 public:
     Encoder(int cs, int id);
 
     void beginSPI();
     uint16_t readEncoderRaw();
     float rawToDegree(uint16_t raw);
     float readEncoderDeg();
 
 };
 
 #endif // ENCODER_HPP
