#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <SPI.h>

// AS5047P encoder registers
#define ENC_NOP         0x0000
#define ENC_ERRFL       0x0001
#define ENC_PROG        0x0003
#define ENC_DIAAGC      0x3FFC
#define ENC_MAG         0x3FFD
#define ENC_ANGLEUNC    0x3FFE
#define ENC_ANGLECOM    0x3FFF

// SPI mode
#define ENC_SPI_MODE    SPI_MODE1
#define SPI_SPEED 1000000

/**
 * @class Encoder
 * @brief Handles interfacing with a rotary encoder via SPI, providing angle readings in degrees.
 *
 * The Encoder class manages SPI communication with a rotary encoder, tracks unwrapped angles,
 * and provides methods to read the current angle in degrees. It also handles chip select pin management
 * and maintains state for continuous rotation tracking.
 *
 * Private Members:
 * - _cs: Chip select pin for SPI communication.
 * - settings: SPI settings for communication.
 * - _prevRaw: Previous raw encoder value for unwrapping.
 * - _rotationCount: Number of full rotations detected.
 * - _firstReading: Flag to indicate if the first reading has been taken.
 *
 * Private Methods:
 * - beginSPI(): Initializes SPI communication.
 * - rawToDegree(uint16_t raw): Converts a raw encoder value to degrees.
 * - readEncoderRaw(): Reads the raw value from the encoder.
 *
 * Public Methods:
 * - Encoder(int cs): Constructor, initializes the encoder with the specified chip select pin.
 * - readEncoderDeg(): Reads the current encoder angle in degrees, accounting for multiple rotations.
 * - getCS(): Returns the chip select pin used by the encoder.
 */
class Encoder {
private:
    int _cs;            // Chip select pin
    SPISettings settings;
    
    // Variables for unwrapped angle tracking
    uint16_t _prevRaw;
    int32_t _rotationCount;
    bool _firstReading;
    
    // Initialize SPI communication
    void beginSPI();
    
    // Convert raw encoder reading to degrees
    float rawToDegree(uint16_t raw);
    
    // Read raw encoder value
    uint16_t readEncoderRaw();
    
public:
    // Constructor
    Encoder(int cs);
    
    // Read encoder value in degrees
    float readEncoderDeg();
    
    // Getters
    int getCS() const { return _cs; }
};

#endif // ENCODER_HPP