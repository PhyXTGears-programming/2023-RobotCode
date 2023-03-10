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
        void disableCalculation();
        void enableCalculation();
        bool calculationEnabled();
        Point getPosition();
        void setPosition(double x, double y);
    private:
        Drivetrain * c_drivetrain = nullptr;
        bool m_calculate = true;
        Point m_position;
        double m_previousTime = 0;
};
