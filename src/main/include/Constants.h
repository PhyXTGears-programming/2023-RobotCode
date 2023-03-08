#pragma once
#include "Interfaces.h"

namespace Constants {
    const int k_NumberOfSwerveModules = 4;

    const double k_minDriveSpeed = 0.15;
    const double k_normalDriveSpeed = 0.30;
    const double k_maxDriveSpeed = 0.60;
    const double k_maxSpinSpeed = 1.00;

    namespace Arm {
        const double k_forearmLenInches = 24.0; // Update to correct value
        const double k_bicepLenInches = 24.0;// Update to correct value

        const double k_shoulderMinDeg = 0;
        const double k_shoulderMaxDeg = 270;
    };
};
