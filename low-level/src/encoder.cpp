#include "encoder.hpp"


/**
 * @brief Constructs an Encoder object and initializes the SPI settings and chip select pin.
 * 
 * @param cs The chip select (CS) pin number used to communicate with the encoder via SPI.
 * 
 * This constructor sets up the SPI communication parameters, initializes internal state variables,
 * and configures the specified CS pin as an output, setting it to a HIGH state by default.
 */
Encoder::Encoder(int cs) :
    _cs(cs), settings(SPI_SPEED, MSBFIRST, ENC_SPI_MODE), 
    _prevRaw(0), _rotationCount(0), _firstReading(true) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
}

/**
 * @brief Initializes an SPI transaction with the encoder's SPI settings.
 *
 * This function begins an SPI transaction using the pre-configured settings
 * for communication with the encoder device. It should be called before
 * performing any SPI operations to ensure proper configuration.
 */
void Encoder::beginSPI()
{
    SPI.beginTransaction(settings);
}

/**
 * @brief Converts a raw encoder value to an unwrapped angle in degrees.
 *
 * This function takes a 14-bit raw encoder reading and converts it to a corresponding
 * angle in degrees, accounting for multiple full rotations by tracking wraparounds.
 * It detects forward and backward wrapping when the raw value crosses the threshold,
 * incrementing or decrementing the rotation count accordingly. The returned angle
 * is unwrapped, meaning it can exceed 0-360 degrees to reflect multiple rotations.
 *
 * @param raw The current raw encoder value (0 to 16383).
 * @return The unwrapped angle in degrees.
 */
float Encoder::rawToDegree(uint16_t raw)
{
    // Threshold for detecting wrapping (3/4 of full range)
    const uint16_t WRAP_THRESHOLD = 12287;
    
    if (!_firstReading) {
        // Calculate the difference between current and previous reading
        int32_t diff = (int32_t)raw - (int32_t)_prevRaw;
        
        // Check for forward wrap
        if (diff < -WRAP_THRESHOLD) {
            _rotationCount++;
        }
        // Check for backward wrap
        else if (diff > WRAP_THRESHOLD) {
            _rotationCount--;
        }
    } else {
        _firstReading = false;
    }
    
    _prevRaw = raw;
    
    // Calculate unwrapped angle
    float baseAngle = (raw * 360.0) / 16383.0;
    float unwrappedAngle = baseAngle + (_rotationCount * 360.0);
    return unwrappedAngle;
}

/**
 * @brief Reads the raw 14-bit angle value from the encoder via SPI.
 *
 * This function communicates with the encoder using SPI to perform the following steps:
 * 1. Sends a command to read the angle (ENC_ANGLECOM).
 * 2. Sends a command to read error flags (ENC_ERRFL) and stores the response.
 * 3. Sends a command to read diagnostics (ENC_DIAAGC).
 * 4. Sends a NOP command (ENC_NOP) to complete the SPI transaction.
 * 
 * Each SPI transaction is separated by a short delay to meet timing requirements.
 * The function extracts and returns the 14 least significant bits from the error flags response,
 * which represent the raw angle value from the encoder.
 *
 * @return uint16_t The 14-bit raw angle value read from the encoder.
 */
uint16_t Encoder::readEncoderRaw()
{
    uint16_t pos_temp;
    uint16_t cmd;
  
    // Read angle command
    cmd = (0b11<<14) | ENC_ANGLECOM;
    digitalWrite(this->_cs, LOW);
    SPI.transfer16(cmd);
    digitalWrite(this->_cs, HIGH);
    delayNanoseconds(400);

    // Read error flags
    cmd = (0b01<<14) | ENC_ERRFL;
    digitalWrite(this->_cs, LOW);
    pos_temp = SPI.transfer16(cmd);
    digitalWrite(this->_cs, HIGH);
    delayNanoseconds(400);

    // Read diagnostics
    cmd = (0b11<<14) | ENC_DIAAGC;
    digitalWrite(this->_cs, LOW);
    uint16_t error = SPI.transfer16(cmd);
    digitalWrite(this->_cs, HIGH);
    delayNanoseconds(400);

    // NOP command
    cmd = (0b11<<14) | ENC_NOP;
    digitalWrite(this->_cs, LOW);
    uint16_t diag = SPI.transfer16(cmd);
    digitalWrite(this->_cs, HIGH);

    // Extract the 14-bit angle value and return it
    return pos_temp & 0b11111111111111;
}

/**
 * @brief Reads the current encoder position in degrees.
 *
 * Initiates an SPI transaction to communicate with the encoder,
 * retrieves the raw encoder value, ends the SPI transaction,
 * and converts the raw value to degrees.
 *
 * @return The current encoder position in degrees as a float.
 */
float Encoder::readEncoderDeg()
{
    this->beginSPI();
    uint16_t raw = this->readEncoderRaw();
    SPI.endTransaction();
    return this->rawToDegree(raw);
    //return 0.;
}