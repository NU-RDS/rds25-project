/*
 * Force feedback using Loadcell
 * Designed for NAU7802 Amplifier
 */

#ifndef LOADCELL_HPP
#define LOADCELL_HPP

#include <Wire.h>
#include <Adafruit_NAU7802.h>

class Loadcell
{
private:
    Adafruit_NAU7802 nau;
    TwoWire* i2c;
    float offset;
    float newtonsPerCount;

    void configure();

public:
    Loadcell(TwoWire* i2c = &Wire);
    void setOffset(float offset);
    void setNewtonsPerCount(float newtonsPerCount);
    float getOffset() const;
    float getNewtonsPerCount() const;
    float readForce();
};

#endif // LOADCELL_HPP