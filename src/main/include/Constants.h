#pragma once
#include "Interfaces.h"

namespace Constants {
    const int k_NumberOfSwerveModules = 4;

    const double k_minDriveSpeed = 0.25;
    const double k_normalDriveSpeed = 0.50;
    const double k_maxDriveSpeed = 0.70;
    const double k_maxSpinSpeed = 1.00;

    namespace Arm {
        const double k_forearmLenMeters = 32.0 /*inch*/ * 0.0254;
        const double k_bicepLenMeters   = 36.0 /*inch*/ * 0.0254;
    };
};
