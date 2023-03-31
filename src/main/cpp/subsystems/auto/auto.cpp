#include "Mandatory.h"
#include "subsystems/auto/auto.h"

// #include <frc2/command/StartEndCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <iostream>

frc2::CommandPtr makeAutoDumpCubeAndScore(Drivetrain * drivetrain) {
    auto orientWheels = frc2::FunctionalCommand{
    //dump cube and move auto
        [=] () { std::cout << "drivetrainPTR: " << drivetrain << std::endl; },
        [=] () { drivetrain->setMotion(0,0.05,0); },
        [=] (bool done) { drivetrain->setMotion(0,0,0); },
        [] () { return false; },
        { drivetrain }
    }.ToPtr();

    auto forceOffCube = frc2::FunctionalCommand{
        [] () {},
        [=] () { drivetrain->setMotion(0,0.5,0); },
        [=] (bool done) { drivetrain->setMotion(0,0,0); },
        [] () { return false; },
        { drivetrain }
    }.ToPtr();

    auto putCubeIntoStation = frc2::FunctionalCommand{
        [] () {},
        [=] () { drivetrain->setMotion(0,-0.15,0); },
        [=] (bool done) { drivetrain->setMotion(0,0,0); },
        [] () { return false; },
        {drivetrain}
    }.ToPtr();

    auto driveOutOfSafeZone = frc2::FunctionalCommand{
        [] () {},
        [=] () { drivetrain->setMotion(0,0.25,0); },
        [=] (bool done) { drivetrain->setMotion(0,0,0); },
        [] () { return false; },
        { drivetrain }
    }.ToPtr();

    auto autoDumpCubeAndScore = std::move(orientWheels).WithTimeout(1.0_s)
        .AndThen(std::move(forceOffCube).WithTimeout(0.3_s))
        .AndThen(std::move(putCubeIntoStation).WithTimeout(2.0_s));

    return std::move(autoDumpCubeAndScore);
}


frc2::CommandPtr makeAutoDumpCubeAndScoreAndLeaveSafeZone(Drivetrain * drivetrain) {
    auto orientWheels = frc2::FunctionalCommand{
    //dump cube and move auto
        [=] () { std::cout << "drivetrainPTR1: " << drivetrain << std::endl; },
        [=] () { drivetrain->setMotion(0,0.05,0); },
        [=] (bool done) { drivetrain->setMotion(0,0,0); },
        [] () { return false; },
        { drivetrain }
    }.ToPtr();

    auto forceOffCube = frc2::FunctionalCommand{
        [=] () { std::cout << "drivetrainPTR2: " << drivetrain << std::endl; },
        [=] () { drivetrain->setMotion(0,0.5,0); },
        [=] (bool done) { drivetrain->setMotion(0,0,0); },
        [] () { return false; },
        { drivetrain }
    }.ToPtr();

    auto putCubeIntoStation = frc2::FunctionalCommand{
        [=] () { std::cout << "drivetrainPTR3: " << drivetrain << std::endl; },
        [=] () { drivetrain->setMotion(0,-0.15,0); },
        [=] (bool done) { drivetrain->setMotion(0,0,0); },
        [] () { return false; },
        {drivetrain}
    }.ToPtr();

    auto driveOutOfSafeZone = frc2::FunctionalCommand{
        [=] () { std::cout << "drivetrainPTR4: " << drivetrain << std::endl; },
        [=] () { drivetrain->setMotion(0,0.25,0); },
        [=] (bool done) { drivetrain->setMotion(0,0,0); },
        [] () { return false; },
        { drivetrain }
    }.ToPtr();

    auto autoDumpCubeScoreAndLeaveSafeZone = std::move(orientWheels).WithTimeout(1.0_s)
        .AndThen(std::move(forceOffCube).WithTimeout(0.3_s))
        .AndThen(std::move(putCubeIntoStation).WithTimeout(2.0_s))
        .AndThen(std::move(driveOutOfSafeZone).WithTimeout(3.5_s));

    return std::move(autoDumpCubeScoreAndLeaveSafeZone);
}