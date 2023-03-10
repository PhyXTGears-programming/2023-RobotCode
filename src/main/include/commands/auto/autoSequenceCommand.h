#pragma once
#include "Mandatory.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

// suggested, but don't remember the reason why (in the GitHub readme)
//
// https://github.com/nlohmann/json
#define JSON_USE_IMPLICIT_CONVERSIONS 0
// for whatever reason the program breaks without the below line on the last line of the file... I have no clue what is up with it...
#define JSON_DIAGNOSTICS 1
#include "external/json.hpp"

#include "commands/drivetrain/driveAutoCommand.h"

class AutoSequenceCommand : public frc2::CommandHelper<frc2::CommandBase, AutoSequenceCommand> {
public:
    AutoSequenceCommand(std::string pathPlannerFileName, DriveAutoCommand* driveCommand);
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

    void nextCommand();
    void dispatcher(std::string name);
private:
    nlohmann::json c_json;
    unsigned int m_currentIndex;
    bool m_finishedMovement = false;
    std::vector<int> m_subCommand;
    int m_currentSubtree = 0;

    bool m_done = false; // used to tell the scheduler if the auto command sequence is done

    DriveAutoCommand* c_driveAutoCommand;

    std::vector<frc2::CommandBase*> m_activeCommands;
};