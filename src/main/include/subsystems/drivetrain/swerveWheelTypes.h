#pragma once
#include "Mandatory.h"

#define PROTOCOL_CAN 0
#define PROTOCOL_PWM 1

#define VENDOR_CTRE_CANCODER 0
#define VENDOR_CTRE_TALON 1
#define VENDOR_CTRE_FALCON 2
#define VENDOR_REV_SPARKMAX 6

namespace SwerveWheelTypes {
    struct SwerveWheelTypes {
        int ID;
        int Protocol;
        int Vendor;
    };
}