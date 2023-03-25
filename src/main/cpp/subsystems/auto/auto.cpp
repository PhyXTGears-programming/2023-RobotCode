#include "Mandatory.h"
#include "commands/auto/AccelerateCommand.h"
#include "subsystems/auto/auto.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/StartEndCommand.h>

#include <optional>

frc2::CommandPtr makeAutoDumpCubeAndScore(Drivetrain * drivetrain) {
    auto orientWheels = frc2::StartEndCommand{
    //dump cube and move auto
        [=] () { drivetrain->setMotion(0,0.05,0); },
        [=] () { drivetrain->setMotion(0,0,0); },
        { drivetrain }
    }.ToPtr();

    auto forceOffCube = frc2::StartEndCommand{
        [=] () { drivetrain->setMotion(0,0.5,0); },
        [=] () { drivetrain->setMotion(0,0,0); },
        { drivetrain }
    }.ToPtr();

    auto putCubeIntoStation = frc2::StartEndCommand{
        [=] () { drivetrain->setMotion(0,-0.15,0); },
        [=] () { drivetrain->setMotion(0,0,0); },
        {drivetrain}
    }.ToPtr();

    auto autoDumpCubeAndScore = std::move(orientWheels).WithTimeout(1.0_s)
        .AndThen(std::move(forceOffCube).WithTimeout(0.3_s))
        .AndThen(AccelerateCommand(0.0, 0.0, 0.075, drivetrain).ToPtr())
        .AndThen(std::move(putCubeIntoStation).WithTimeout(2.0_s));

    return std::move(autoDumpCubeAndScore);
}


frc2::CommandPtr makeAutoDumpCubeAndScoreAndLeaveSafeZone(Drivetrain * drivetrain) {
    auto orientWheels = frc2::StartEndCommand{
    //dump cube and move auto
        [=] () { drivetrain->setMotion(0,0.05,0); },
        [=] () { drivetrain->setMotion(0,0,0); },
        { drivetrain }
    }.ToPtr();

    auto forceOffCube = frc2::StartEndCommand{
        [=] () { drivetrain->setMotion(0,0.5,0); },
        [=] () { drivetrain->setMotion(0,0,0); },
        { drivetrain }
    }.ToPtr();

    auto putCubeIntoStation = frc2::StartEndCommand{
        [=] () { drivetrain->setMotion(0,-0.15,0); },
        [=] () { drivetrain->setMotion(0,0,0); },
        {drivetrain}
    }.ToPtr();

    std::optional<frc2::CommandPtr> driveOutOfSafeZone = std::nullopt;
    {
        static const double forward = 0.25;
        static const double percent = 0.05;
        driveOutOfSafeZone = AccelerateCommand(0.0, forward, percent, drivetrain).ToPtr()
            .AndThen(frc2::StartEndCommand{
                [=] () { drivetrain->setMotion(0, forward, 0); },
                [=] () { drivetrain->setMotion(0, 0, 0); },
                { drivetrain }
            }.ToPtr());
    }

    auto autoDumpCubeScoreAndLeaveSafeZone = std::move(orientWheels).WithTimeout(1.0_s)
        .AndThen(std::move(forceOffCube).WithTimeout(0.3_s))
        .AndThen(AccelerateCommand(0.0, 0.0, 0.075, drivetrain).ToPtr())
        .AndThen(std::move(putCubeIntoStation).WithTimeout(2.0_s))
        .AndThen(std::move(*driveOutOfSafeZone).WithTimeout(3.0_s));

    return std::move(autoDumpCubeScoreAndLeaveSafeZone);
}