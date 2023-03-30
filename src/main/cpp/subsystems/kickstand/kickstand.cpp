#include "Mandatory.h"

#include "subsystems/kickstand/kickstand.h"

Kickstand::Kickstand(){
    //nothing to initialize
}

void Kickstand::releasePosition(){
    c_kickstandServo.SetAngle(Constants::k_kickstandServoReleaseAngle);
}

void Kickstand::disable() {
    c_kickstandServo.SetDisabled();
}