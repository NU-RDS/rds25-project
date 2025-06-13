/*
 * Encoder Library for Magnetic Angle Sensors
 * Designed for AS5048 or similar SPI-based magnetic encoders
 */

#ifndef ENCODER_HPP
#define ENCODER_HPP
 
#include <Arduino.h>
#include <SPI.h>

// Register definitions
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
 * @brief Represents an SPI-based rotary encoder interface.
 *
 * This class provides methods to initialize SPI communication, read raw encoder values,
 * and convert those values to degrees. Each Encoder instance is associated with a chip select (CS) pin
 * and an identifier.
 *
 * @private
 *   - int _cs: Chip select pin for SPI communication.
 *   - int _id: Identifier for the encoder instance.
 *   - SPISettings settings: SPI configuration settings.
 *
 * @public
 *   - Encoder(int cs, int id): Constructs an Encoder with the specified chip select pin and ID.
 *   - void beginSPI(): Initializes the SPI interface for the encoder.
 *   - uint16_t readEncoderRaw(): Reads the raw value from the encoder via SPI.
 *   - float rawToDegree(uint16_t raw): Converts a raw encoder value to degrees.
 *   - float readEncoderDeg(): Reads the encoder value and returns the position in degrees.
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
