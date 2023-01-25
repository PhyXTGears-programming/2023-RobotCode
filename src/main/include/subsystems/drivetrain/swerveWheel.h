#pragma once
#include "Mandatory.h"

#include <frc2/command/SubsystemBase.h>

class SwerveWheel : public frc2::SubsystemBase {
    public:
        SwerveWheel(double x_pos, double y_pos);

        void setStrife(double x);

        void setForeward(double y);

        void setRotation(double r);

        void setMotion(double x, double y, double r);

        double getHeading();

        double getVelocity();
};