#pragma once

#include "Mandatory.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/kickstand/kickstand.h"

class KickstandReleaseCommand : public frc2::CommandHelper<frc2::CommandBase, KickstandReleaseCommand> {
public:
    KickstandReleaseCommand(Kickstand* kickstand);
    
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
private:
    Kickstand* c_kickstand;
};