#pragma once

#include "Mandatory.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/Servo.h>

class Kickstand : public frc2::SubsystemBase {
public:
    Kickstand();
    void releasePosition();
    void disable();
private:
    frc::Servo c_kickstandServo{Interfaces::k_kickstandServo};
};