#pragma once

#include "subsystems/arm/arm.h"
#include "subsystems/arm/motionPath.h"

#include <util/point.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>


class MoveToIntakeCommand : public frc2::CommandHelper<frc2::CommandBase, MoveToIntakeCommand> {
public:
    MoveToIntakeCommand(ArmSubsystem * arm, Point finalTarget);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    ArmSubsystem * m_arm = nullptr;
    Point m_finalTarget;
    Point m_target;
    std::optional<MotionPath> m_path;

    Point m_currentPoint;
};