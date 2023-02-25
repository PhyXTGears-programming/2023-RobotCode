#pragma once

#include "Mandatory.h"
#include <frc2/command/SubsystemBase.h>

#include "util/point.h"
#include "subsystems/drivetrain/drivetrain.h"

class Odometry : frc2::SubsystemBase {
    public:
        Odometry(Drivetrain* drivetrain);
        ~Odometry();
        void Periodic() override;
        Point getPosition();
        void setPosition(double x, double y);
    private:
        Drivetrain * m_drivetrain = nullptr;
        Point m_position;
        double m_previousTime = 0;
};