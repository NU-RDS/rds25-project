#ifndef RDS_HAND_HPP
#define RDS_HAND_HPP

#include <string>
#include <vector>
#include <memory>
#include <iostream>

using namespace std;

class Tendon {
private:
    int ID;
    float Force;

public:
    explicit Tendon(int id = -1, float force = 0.)
    : ID(id), Force(force) {}

    virtual ~Tendon() = default;
    
    int GetID();
    float GetForce();
};

class Joint {
private:
    int ID;
    float Angle;
    float Speed;
    float Torque;
    vector<Tendon> Tendons;

    float CalculateSpeed();

public:
    explicit Joint(int id = -1, float angle = 0., float speed = 0., float torque = 0.)
    : ID(id), Angle(angle), Speed(speed), Torque(torque) {}

    virtual ~Joint() = default;

    int GetID();
    float GetAngle();
    float SetAngle();
    float GetSpeed();
    float SetSpeed();
    float SetTorque();

};

class Wrist {
private:
    Joint Roll;
    Joint Pitch;
    Joint Yaw;

public:

};


class DexFinger {
private:
    Joint DIP;
    Joint PIP;
    Joint MCP;
    Joint SPLAIN;

public:

};

class PowFinger {
private:
    Joint Grasp;

public:

};

#endif