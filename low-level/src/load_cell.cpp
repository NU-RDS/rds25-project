#include "load_cell.hpp"

/**
 * @brief Constructs a Loadcell object with the specified I2C interface.
 * 
 * Initializes the Loadcell with the provided TwoWire I2C pointer, and sets
 * the offset and newtonsPerCount calibration values to zero.
 * 
 * @param i2c Pointer to the I2C interface (TwoWire) used for communication with the load cell.
 */
Loadcell::Loadcell(TwoWire* i2c)
    : i2c(i2c), offset(0.0f), newtonsPerCount(0.0f)
{}

/**
 * @brief Configures the load cell by initializing the I2C interface and NAU7802 ADC.
 *
 * This function performs the following steps:
 * - Initializes the I2C communication and sets the clock speed to 400kHz.
 * - Attempts to initialize the NAU7802 ADC; retries if the device is not found.
 * - Configures the NAU7802 with a 3.3V LDO, 320 samples per second rate, and a gain of 128.
 * - Performs both internal and offset calibration on the NAU7802.
 * - Stabilizes the ADC by discarding the first 10 readings.
 */
void Loadcell::configure()
{
    i2c->begin();
    i2c->setClock(400000);

    while (!nau.begin(i2c)) {
       Serial.println("ERROR: NAU7802 not found");
       delay(1000);
    }

    nau.setLDO(NAU7802_3V3);
    nau.setRate(NAU7802_RATE_320SPS);
    nau.setGain(NAU7802_GAIN_128);
    
    nau.calibrate(NAU7802_CALMOD_INTERNAL);
    nau.calibrate(NAU7802_CALMOD_OFFSET);

    // Stabilize ADC
    for (int i = 0; i < 10; i++) {
        while (!nau.available());
        nau.read();
    }
}

void Loadcell::setOffset(float offset)
{
    this->offset = offset;
}

void Loadcell::setNewtonsPerCount(float newtonsPerCount)
{
    this->newtonsPerCount = newtonsPerCount;
}

float Loadcell::getOffset() const
{
    return offset;
}

float Loadcell::getNewtonsPerCount() const
{
    return newtonsPerCount;
}

/**
 * @brief Reads the current force measurement from the load cell.
 *
 * This function checks if new data is available from the NAU sensor. If available,
 * it reads the raw value, applies the offset correction, and converts the result
 * to Newtons using the calibration factor (newtonsPerCount).
 *
 * @return The measured force in Newtons. Returns 0.0f if no new data is available.
 */
float Loadcell::readForce()
{
    if (nau.available()) {
        int32_t reading = nau.read();
        return (reading - offset) * newtonsPerCount;
    }
    return 0.0f;
}