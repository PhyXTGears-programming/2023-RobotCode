#include "Mandatory.h"

#include "commands/auto/autoSequenceCommand.h"

// suggested, but don't remember the reason why (in the GitHub readme)
//
// https://github.com/nlohmann/json
#define JSON_USE_IMPLICIT_CONVERSIONS 0
// for whatever reason the program breaks without the below line on the last line of the file... I have no clue what is up with it...
#define JSON_DIAGNOSTICS 1
#include "external/json.hpp"

#include <frc/Filesystem.h>

#include <fstream>
#include <stdexcept>

#include "util/point.h"

#include "commands/drivetrain/driveAutoCommand.h"

#ifdef DEBUG_MODE
#include <iostream> // just outputs the commands to run rather than calls the commands
#endif

AutoSequenceCommand::AutoSequenceCommand(std::string pathPlannerFileName, DriveAutoCommand* driveCommand){
    c_driveAutoCommand = driveCommand; // no need for addRequirement because these are not subsystems

    std::fstream file (frc::filesystem::GetDeployDirectory() + pathPlannerFileName);
    c_json = nlohmann::json::parse(file);
    m_currentIndex = 0;
}

void AutoSequenceCommand::Initialize(){}

void AutoSequenceCommand::Execute(){
    bool advance = true; //presume that we can advance, unless told otherwise
    for(int i = 0; i < ((int)m_activeCommands.size()); i++){ // if there is no commands scheduled, it will by default schedule the next command
        if(m_activeCommands.at(i)->IsScheduled()){ // if a command is scheduled, then we know it is still running
            advance = false;
        }
    }
    if(advance){
        nextCommand();
    } else {
        //do nothing, just wait till all commands have finished
    }
}

void AutoSequenceCommand::nextCommand(){
    try{
        c_json["segments"][m_currentIndex];
    } catch (...){ // any error
        //we know we are done because the index is out of range
        m_done = true;
        return;
    } // if no exception, then we have a valid set of points
    if(c_json["waypoints"][m_currentIndex]["shallHalt"]){
        m_activeCommands.clear(); // clear the previous
        if(m_finishedMovement){
            std::string nextCommand;
            try{
                nextCommand = c_json["waypoints"][m_currentIndex]["commands"]["children"][m_subCommand.at(0)].get<std::string>(); // put the next command to be ready for the dispatcher
            } catch (...){ // any error
                m_finishedMovement = false; // reset the movement variable if there are no child sequential commands to run
                m_currentIndex++;
                this->nextCommand(); // run this again to get the next section
                return; // make sure to not go any further
            }
            dispatcher(nextCommand); // execute the next command
            m_subCommand.at(0)++;
        } else {
            Bezier curve = Bezier{(float)c_json["segments"][m_currentIndex][0][0],(float)c_json["segments"][m_currentIndex][0][1], //a
                (float)c_json["segments"][m_currentIndex][1][0],(float)c_json["segments"][m_currentIndex][1][1],                   //b
                (float)c_json["segments"][m_currentIndex][2][0],(float)c_json["segments"][m_currentIndex][2][1],                   //c
                (float)c_json["segments"][m_currentIndex][3][0],(float)c_json["segments"][m_currentIndex][3][1]                    //d
                };
            c_driveAutoCommand->bezierDrive(curve, c_json["segments"]["time"][m_currentIndex].get<double>());// temporary. get the second parameter from the "preprocessor new.py" file and add to end of json
            m_activeCommands.insert(m_activeCommands.end(),c_driveAutoCommand);
            c_driveAutoCommand->Schedule();
            m_finishedMovement = true; // make sure on the next loop we run the other commands
        }
    } else {
        std::string nextCommand;
        try{
            nextCommand = c_json["waypoints"][m_currentIndex]["commands"]["children"][m_subCommand.at(0)].get<std::string>(); // put the next command to be ready for the dispatcher
        } catch (...){ // any error
            if(!c_driveAutoCommand->IsScheduled()){
                m_subCommand.at(0) = 0;
                // do it at the end because the variable is unsigned, so we need to have it ready for the next "loop"
                // we can ensure that if the robot should not halt, we can move on
                m_currentIndex++;
                this->nextCommand(); // run this again to get the next section
                return; // make sure to not go any further
            } else {
                return; // wait for the drivetrain to finish
            }
        }
        dispatcher(nextCommand); // execute the next command
        m_subCommand.at(0)++;
    }
}


#ifdef DEBUG_MODE
void AutoSequenceCommand::dispatcher(std::string name){
    if(name == "halt"){
        std::cout << "halt robot" << std::endl;
    } else if (name == "arm_top"){
        std::cout << "moving arm to top peg" << std::endl;
    } else if (name == "arm_mid"){
        std::cout << "moving arm to mid peg" << std::endl;
    } else if (name == "arm_hybrid"){
        std::cout << "moving arm to hybrid" << std::endl;
    } else if (name == "grip_open"){
        std::cout << "opening gripper" << std::endl;
    } else if (name == "grip_close"){
        std::cout << "closing gripper" << std::endl;
    } else {
        std::cout << "Unrecognized command: " << name << std::endl;
    }
}
#endif
#ifndef DEBUG_MODE
void AutoSequenceCommand::dispatcher(std::string name){
    if(name == "halt"){
        c_driveAutoCommand->Cancel();
    } else if (name == "arm_top"){
        // arm top command
    } else if (name == "arm_mid"){
        // arm mid command
    } else if (name == "arm_hybrid"){
        // arm hybrid command
    } else if (name == "grip_open"){
        // grip open command
    } else if (name == "grip_close"){
        // grip open command
    } else {
        // do nothing. hope and pray this did not get past the testing phase :)
    }
}
#endif

void AutoSequenceCommand::End(bool interrupted){}

bool AutoSequenceCommand::IsFinished(){
    return m_done;
}