#include "encoder.hpp"

Encoder::Encoder(int cs, int id) : 
    _cs(cs), _id(id), settings(500000, MSBFIRST, ENC_SPI_MODE) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
}

void Encoder::beginSPI()
{
    SPIx.beginTransaction(settings);
}

float Encoder::rawToDegree(uint16_t raw)
{
    float angle = (raw * 360.0) / 16383.0;
    return angle;
}

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

float Encoder::readEncoderDeg()
{
    this->beginSPI();
    uint16_t raw = this->readEncoderRaw();
    SPIx.endTransaction();
    return this->rawToDegree(raw);
}