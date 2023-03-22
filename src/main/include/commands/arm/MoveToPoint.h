#pragma once

#include "Mandatory.h"
#include "subsystems/arm/arm.h"
#include "subsystems/arm/motionPath.h"

#include <util/geom.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>


class MoveToPointCommand : public frc2::CommandHelper<frc2::CommandBase, MoveToPointCommand> {
public:
    MoveToPointCommand(ArmSubsystem * arm, Point finalTarget);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    ArmSubsystem * c_arm = nullptr;
    Point c_finalTarget;

    Point m_target;
    std::optional<MotionPath> m_path;
};
