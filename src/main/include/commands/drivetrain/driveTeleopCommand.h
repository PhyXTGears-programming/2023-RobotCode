#pragma once

#include "subsystems/drivetrain/drivetrain.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/drivetrain/drivetrain.h"
#include <frc/XboxController.h>

class DriveTeleopCommand : public frc2::CommandHelper<frc2::CommandBase, DriveTeleopCommand> {
public:
    DriveTeleopCommand(Drivetrain* drivetrain, frc::XboxController* driverController);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    Drivetrain* c_drivetrain;
    frc::XboxController* c_driverController;
};