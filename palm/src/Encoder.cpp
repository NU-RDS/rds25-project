#include "Encoder.hpp"

/**
 * @brief Constructs an Encoder object with specified chip select pin and identifier.
 *
 * Initializes the SPI settings for the encoder, sets the chip select (CS) pin as output,
 * and sets it to HIGH (inactive state).
 *
 * @param cs The chip select (CS) pin number used to communicate with the encoder via SPI.
 * @param id The unique identifier for this encoder instance.
 */
Encoder::Encoder(int cs, int id) : 
    _cs(cs), _id(id), settings(500000, MSBFIRST, ENC_SPI_MODE) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
}

void Encoder::beginSPI()
{
    SPIx.beginTransaction(settings);
}

/**
 * @brief Converts a raw encoder value to degrees.
 *
 * This function takes a 14-bit raw value from an encoder (range: 0 to 16383)
 * and converts it to an angle in degrees (range: 0.0 to 360.0).
 *
 * @param raw The raw 14-bit encoder value.
 * @return The corresponding angle in degrees.
 */
float Encoder::rawToDegree(uint16_t raw)
{
    float angle = (raw * 360.0) / 16383.0;
    return angle;
}

/**
 * @brief Reads the raw 14-bit angle value from the encoder via SPI.
 *
 * This function communicates with the encoder using SPI to retrieve the current angle,
 * error flags, and diagnostic information. It sends specific commands to the encoder,
 * waits for the required timing between transactions, and extracts the 14-bit angle value
 * from the received data.
 *
 * @note The function assumes that the SPI interface and chip select pin have been properly
 *       initialized. Timing requirements between SPI transactions are handled with
 *       delayNanoseconds.
 *
 * @return uint16_t The raw 14-bit angle value read from the encoder.
 */
uint16_t Encoder::readEncoderRaw()
{
    uint16_t pos_temp;
    uint16_t cmd;
  
    // Read angle command
    cmd = (0b11<<14) | ENC_ANGLECOM;
    digitalWrite(this->_cs, LOW);
    SPIx.transfer16(cmd);
    digitalWrite(this->_cs, HIGH);
    delayNanoseconds(400);

    // Read error flags
    cmd = (0b01<<14) | ENC_ERRFL;
    digitalWrite(this->_cs, LOW);
    pos_temp = SPIx.transfer16(cmd);
    digitalWrite(this->_cs, HIGH);
    delayNanoseconds(400);

    // Read diagnostics
    cmd = (0b11<<14) | ENC_DIAAGC;
    digitalWrite(this->_cs, LOW);
    uint16_t error = SPIx.transfer16(cmd);
    digitalWrite(this->_cs, HIGH);
    delayNanoseconds(400);

    // NOP command
    cmd = (0b11<<14) | ENC_NOP;
    digitalWrite(this->_cs, LOW);
    uint16_t diag = SPIx.transfer16(cmd);
    digitalWrite(this->_cs, HIGH);

    // Extract the 14-bit angle value and return it
    return pos_temp & 0b11111111111111;
}

/**
 * @brief Reads the current encoder position in degrees.
 *
 * Initiates an SPI transaction to read the raw encoder value,
 * ends the SPI transaction, and converts the raw value to degrees.
 *
 * @return The current encoder position in degrees as a float.
 */
float Encoder::readEncoderDeg()
{
    this->beginSPI();
    uint16_t raw = this->readEncoderRaw();
    SPIx.endTransaction();
    return this->rawToDegree(raw);
}

bool Encoder::initialize() {
    return true;
}

float Encoder::read() {
    return this->readEncoderDeg();
}

void Encoder::cleanup() {
    // nothing
}