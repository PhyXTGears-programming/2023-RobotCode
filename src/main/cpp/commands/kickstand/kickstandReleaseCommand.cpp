#include "Mandatory.h"

#include "commands/kickstand/kickstandReleaseCommand.h"

#include "subsystems/kickstand/kickstand.h"

KickstandReleaseCommand::KickstandReleaseCommand(Kickstand* kickstand){
    c_kickstand = kickstand;

    AddRequirements(c_kickstand);
}

void KickstandReleaseCommand::Initialize(){
    c_kickstand->releasePosition();
}

void KickstandReleaseCommand::Execute(){
    return; // dont do anything
}

bool KickstandReleaseCommand::IsFinished(){
    return true; // end immediately
}

void KickstandReleaseCommand::End(bool interrupted){
    c_kickstand->disable();
    return; // dont do anything
}