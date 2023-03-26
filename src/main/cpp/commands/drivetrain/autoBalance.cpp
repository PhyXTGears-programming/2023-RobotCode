
#include "commands/drivetrain/autoBalance.h"
#include "subsystems/drivetrain/drivetrain.h"

#define USE_MATH_DEFINES
#include <math.h>

#define DEGREES_TO_RADIANS(deg) ((deg/180)*M_PI)
//50 ticks per second
#define SECONDS_TO_TICKS(time) (time*50)

autoBalance::autoBalance(Drivetrain* drivetrain){
    c_drivetrain = drivetrain;
    state = 0;
    debounceCount = 0;

    /**********
     * CONFIG *
     **********/
    //Speed the robot drive while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.4;
    
    //Speed the robot drives while balancing itself on the charge station.
    //Should be roughly half the fast speed, to make the robot more accurate, default = 0.2
    robotSpeedSlow = 0.2;

    //Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 13.0;
    // onChargeStationDegree = DEGREES_TO_RADIANS(13.0);

    //Angle where the robot can assume it is level on the charging station
    //Used for exiting the drive forward sequence as well as for auto balancing, default = 6.0
    levelDegree = 6.0;
    // levelDegree = DEGREES_TO_RADIANS(6.0);
    //Amount of time a sensor condition needs to be met before changing states in seconds
    //Reduces the impact of sensor noise, but too high can make the auto run slower, default = 0.2
    debounceTime = 0.2;
		
	//Amount of time to drive towards to scoring target when trying to bump the game piece off
	//Time it takes to go from starting position to hit the scoring target
	singleTapTime = 0.4;
		
	//Amount of time to drive away from knocked over gamepiece before the second tap
	scoringBackUpTime = 0.2;
		
	//Amount of time to drive forward to secure the scoring of the gamepiece
	doubleTapTime = 0.3;
}

double autoBalance::getPitch(){ //return in radians
    return  std::atan2((- c_drivetrain->getXTilt()) ,
                    std::sqrt( std::pow(c_drivetrain->getYTilt(),2) + std::pow(c_drivetrain->getFieldHeading(),2) )
            ) * (180/M_PI);
}

double autoBalance::getRoll(){ // return in radians
    return  std::atan2(c_drivetrain->getYTilt(),
            c_drivetrain->getFieldHeading()) * (180/M_PI);
}

//returns the magnititude of the robot's tilt calculated by the root of
//pitch^2 + roll^2, used to compensate for diagonally mounted rio
double autoBalance::getTilt(){
	// double pitch = getPitch();
	// double roll = getRoll();
    // double roll = 0;
    // if((pitch + roll)>= 0){
    //     return std::sqrt(pitch*pitch + roll*roll);
    // } else {
    //     return -std::sqrt(pitch*pitch + roll*roll);
    // }
    return c_drivetrain->getYTilt() + DEGREES_TO_RADIANS(2);
}

//routine for automatically driving onto and engaging the charge station.
//returns a value from -1.0 to 1.0, which left and right motors should be set to.
double autoBalance::autoBalanceRoutine(){
    switch (state){
        //drive forwards to approach station, exit when tilt is detected
        case 0:
            if(getTilt() > onChargeStationDegree){
                debounceCount++;
            }
            if(debounceCount > SECONDS_TO_TICKS(debounceTime)){
                state = 1;
                debounceCount = 0;
                return robotSpeedSlow;
            }
            return robotSpeedFast;
        //driving up charge station, drive slower, stopping when level
        case 1:
            if (getTilt() < levelDegree){
                debounceCount++; 
            }
            if(debounceCount > SECONDS_TO_TICKS(debounceTime)){
                state = 2;
                debounceCount = 0;
                return 0;
            }
            return robotSpeedSlow;
        //on charge station, stop motors and wait for end of auto
        case 2:
            if(fabs(getTilt()) <= levelDegree/2){
                debounceCount++;
            }
            if(debounceCount>SECONDS_TO_TICKS(debounceTime)){
                state = 4;
                debounceCount = 0;
                return 0;
            }
            if(getTilt() >= levelDegree) {
                return 0.1;
            } else if(getTilt() <= -levelDegree) {
                return -0.1;
            }
        case 3:
            return 0;
    }
    return 0;
}