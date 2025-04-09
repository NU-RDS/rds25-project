#ifndef HAND_LIBRARY_H
#define HAND_LIBRARY_H

#include <string>
#include <vector>
#include <memory>
#include <iostream>

using namespace std;

class Joint {
private:
    int ID;
    float Angle;
    float Speed;
    float Torque;

    float CalculateSpeed();

public:
    explicit Joint(int id = -1, float angle = 0., float speed = 0., float torque = 0.)
    : ID(id), Angle(angle), Speed(speed), Torque(torque) {}

    float GetAngle();
    float SetAngle();
    float GetSpeed();
    float SetSpeed();
    float SetTorque();

    virtual ~Joint() = default;
};