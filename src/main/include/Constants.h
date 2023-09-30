#pragma once
#include "Interfaces.h"

#include <cmath>

namespace Constants {
    const int k_NumberOfSwerveModules = 4;

    const double k_minDriveSpeed = 0.25;
    const double k_normalDriveSpeed = 0.50;
    const double k_maxDriveSpeed = 1.00;
    const double k_maxSpinSpeed = 1.00;

    const double k_kickstandServoReleaseAngle = 120;

    namespace Arm {
        const double k_forearmLenMeters = 32.0 /*inch*/ * 0.0254;
        const double k_bicepLenMeters   = 36.0 /*inch*/ * 0.0254;

        const double k_ChassisXSize = 0.775; // meters
        const double k_ChassisYSize = 0.826; // meters
        const double k_ChassisZSize = 0.229; // meters

        const double k_TurretZOffset = 0.210; // meters

        const double k_PickupXSize = 0.076;  // meters
        const double k_PickupYSize = 0.340;  // meters
        const double k_PickupZSize = 1.000;  // meters

        // These constants belong to ArmTeleopCommand.
        const double k_maxPointSpeed    = 0.004;
        const double k_maxPointRotSpeed = (2.0 * M_PI / 24.0) * 0.02;   // radians per second in 20ms.
        const double k_maxTurnSpeed     = 0.1;  // % motor speed.
        const double k_maxWristRotSpeed = (2.0 * M_PI / 2.0) * 0.02;
        const double k_maxGripSpeed     = 0.2;
    };
};
