#pragma once

#include "Mandatory.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/drivetrain/drivetrain.h"

#include "commands/drivetrain/autoBalance.h"
#include <frc2/command/PIDCommand.h>

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

    enum DRIVE_STATE{
        approachingChargingStation,
        engagingChargingStation,
        balancingOnStation,
        done
    };

    DRIVE_STATE currentAutoStage = DRIVE_STATE::approachingChargingStation;
    frc2::PIDController* balancePID = new frc2::PIDController(0.05,0,0);
};