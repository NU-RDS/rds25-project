#include "load_cell.hpp"

Loadcell::Loadcell(TwoWire* i2c)
    : i2c(i2c), offset(0.0f), newtonsPerCount(0.0f)
{
    configure();
}

void Loadcell::configure()
{
    i2c->begin();
    i2c->setClock(400000);

    //if (!nau.begin(i2c)) {
    //    while (1); // Hang if device not detected
    //}

    nau.setLDO(NAU7802_3V3);
    nau.setGain(NAU7802_GAIN_128);
    nau.setRate(NAU7802_RATE_320SPS);

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

float Loadcell::readForce()
{
    if (nau.available()) {
        int32_t reading = nau.read();
        return (reading - offset) * newtonsPerCount;
    }
    return 0.0f;
}