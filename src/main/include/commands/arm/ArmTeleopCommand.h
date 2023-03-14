#pragma once

#include "Mandatory.h"
#include "subsystems/arm/arm.h"
#include "util/geom.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/XboxController.h>

class ArmTeleopCommand : public frc2::CommandHelper<frc2::CommandBase, ArmTeleopCommand> {
public:
    ArmTeleopCommand(ArmSubsystem* arm, frc::XboxController* operatorController);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
    void resetTarget();
private:
    ArmSubsystem* c_arm;
    frc::XboxController* c_operatorController;

    Point m_target;
};