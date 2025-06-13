/*
 * Force feedback using Loadcell
 * Designed for NAU7802 Amplifier
 */

#ifndef LOADCELL_HPP
#define LOADCELL_HPP

#include <Wire.h>
#include <Adafruit_NAU7802.h>

class Loadcell
/**
 * @class Loadcell
 * @brief Interface for interacting with a load cell sensor using the NAU7802 ADC.
 *
 * This class provides methods to configure the load cell, set calibration parameters,
 * and read force measurements in Newtons. It uses the Adafruit_NAU7802 library for
 * communication with the ADC over I2C.
 *
 * @private
 *   - Adafruit_NAU7802 nau: Instance of the NAU7802 ADC driver.
 *   - TwoWire* i2c: Pointer to the I2C interface.
 *   - float offset: Calibration offset for zeroing the load cell.
 *   - float newtonsPerCount: Conversion factor from ADC counts to Newtons.
 *
 * @public
 *   - void configure(): Initializes and configures the load cell hardware.
 *   - Loadcell(TwoWire* i2c): Constructor that accepts an I2C interface pointer.
 *   - void setOffset(float offset): Sets the calibration offset.
 *   - void setNewtonsPerCount(float newtonsPerCount): Sets the conversion factor.
 *   - float getOffset() const: Returns the current offset value.
 *   - float getNewtonsPerCount() const: Returns the current conversion factor.
 *   - float readForce(): Reads the current force value from the load cell in Newtons.
 */
{
private:
    Adafruit_NAU7802 nau;
    TwoWire* i2c;
    float offset;
    float newtonsPerCount;
    
    
public:
    void configure();
    Loadcell(TwoWire* i2c);
    void setOffset(float offset);
    void setNewtonsPerCount(float newtonsPerCount);
    float getOffset() const;
    float getNewtonsPerCount() const;
    float readForce();
};

#endif // LOADCELL_HPP