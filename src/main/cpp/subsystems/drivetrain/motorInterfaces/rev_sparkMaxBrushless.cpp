#include "Mandatory.h"

#include "subsystems/drivetrain/motorInterfaces/rev_sparkMaxBrushless.h"

#include <rev/CANSparkMax.h>

RevSparkMaxBrushless::RevSparkMaxBrushless(int canID){
    motor = new rev::CANSparkMax(canID, rev::CANSparkMax::MotorType::kBrushless);
}

void RevSparkMaxBrushless::setMotion(double speed){
    motor->Set(speed);
}