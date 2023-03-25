#pragma once

#include "Mandatory.h"

#include "subsystems/drivetrain/drivetrain.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class AccelerateCommand : public frc2::CommandHelper<frc2::CommandBase, AccelerateCommand> {
public:
    explicit AccelerateCommand(double forward, double strafe, double percent, Drivetrain * drive);

    void Initialize() override;

    void Execute() override;

    void End(bool isInterrupted) override;

    bool IsFinished() override;

private:
    Drivetrain * c_drive = nullptr;

    double c_percent;

    double c_finalForward;
    double c_finalStrafe;

    double m_prevForward;
    double m_prevStrafe;
};