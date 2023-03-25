#pragma once
#include "Mandatory.h"
#include "subsystems/drivetrain/drivetrain.h"

#include <frc2/command/CommandPtr.h>

#include "commands/drivetrain/driveLevelOnChargeStation.h"

frc2::CommandPtr makeAutoDumpCubeAndScore(Drivetrain * drivetrain);
frc2::CommandPtr makeAutoDumpCubeAndScoreAndLeaveSafeZone(Drivetrain * drivetrain);
frc2::CommandPtr makeAutoDumpCubeAndScoreAndLeaveSafeZoneThenBalance(Drivetrain * drivetrain);