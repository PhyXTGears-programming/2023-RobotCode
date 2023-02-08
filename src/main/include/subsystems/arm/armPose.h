#pragma once

class ArmPose {
    public:
        ArmPose(float turretAng, float shoulderAng, float elbowAng) : turretAng(turretAng), shoulderAng(shoulderAng), elbowAng(elbowAng) {}

        float turretAng;
        float shoulderAng;
        float elbowAng;
};
