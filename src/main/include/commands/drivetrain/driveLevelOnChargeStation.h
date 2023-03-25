#pragma once

#include "Mandatory.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/drivetrain/drivetrain.h"

#include "commands/drivetrain/autoBalance.h"

class DriveLevelCommand : public frc2::CommandHelper<frc2::CommandBase, DriveLevelCommand> {
public:
    DriveLevelCommand(Drivetrain* drivetrain);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    Drivetrain* c_drivetrain;
    autoBalance* c_autoBalance;

    enum driveState{
        approachingChargingStation,
        engagingChargingStation,
        balancingOnStation
    };
};