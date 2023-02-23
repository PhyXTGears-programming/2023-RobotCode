#include "subsystems/drivetrain/drivetrain.h"
#include "Mandatory.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include "util/point.h"
#include "util/polar.h"

#include "subsystems/drivetrain/swerveWheel.h"
#include "subsystems/drivetrain/swerveWheelTypes.h"


#define DEGREES_TO_RADIANS(deg) ((deg/180.0)*M_PI)
#include <AHRS.h>
/*
NOTE ON UNITS:

all math in here is based on the units of meters, radians and seconds.

there should not be any other units in here, so if there turns out to be,
it should be removed or put in a debug preprocessor.
*/

Drivetrain::Drivetrain(bool fieldOriented) {
    this->m_fieldOriented = fieldOriented;
    Drivetrain::setupWheels();
    Drivetrain::resetNavxHeading();
}

Drivetrain::Drivetrain() {
    this->m_fieldOriented = false;
    Drivetrain::setupWheels();
    Drivetrain::resetNavxHeading();
}

Drivetrain::~Drivetrain() {
    // cleanup classes and prevent memory leaks
    for (int i = 0; i < Constants::k_NumberOfSwerveModules; i += 1) {
        delete c_wheels[i];
        c_wheelPositions[i].x = 0;
        c_wheelPositions[i].y = 0;
    }
}

void Drivetrain::Periodic(){
    if(m_fieldOriented){
        //gets the current angle of the NavX (reported in degrees, converted to radians)
        m_fieldOrientedOffset = DEGREES_TO_RADIANS(m_navX->GetYaw())+M_PI;
    }
    //just tell the motor abstractions to pet the watchdog and update the motors
    for (int i = 0; i < Constants::k_NumberOfSwerveModules; i++) {
        Drivetrain::c_wheels[i]->Periodic();
    }
}

void Drivetrain::setupWheels() {
    //setup wheel classes with correct motors from toml file
    Drivetrain::c_wheels[0] = new SwerveWheel(
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainFrontLeftSteer,  .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_REV_SPARKMAX },
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainFrontLeftDrive,  .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_CTRE_FALCON },
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainFrontLeftEncoder, .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_CTRE_CANCODER }
    );
    Drivetrain::c_wheels[1] = new SwerveWheel(
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainFrontRightSteer,  .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_REV_SPARKMAX },
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainFrontRightDrive,  .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_CTRE_FALCON },
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainFrontRightEncoder, .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_CTRE_CANCODER }
    );
    Drivetrain::c_wheels[2] = new SwerveWheel(
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainBackRightSteer,  .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_REV_SPARKMAX },
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainBackRightDrive,  .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_CTRE_FALCON },
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainBackRightEncoder, .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_CTRE_CANCODER }
    );
    Drivetrain::c_wheels[3] = new SwerveWheel(
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainBackLeftSteer,  .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_REV_SPARKMAX },
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainBackLeftDrive,  .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_CTRE_FALCON },
        SwerveWheelTypes::SwerveWheelTypes{ .ID = Interfaces::k_drivetrainBackLeftEncoder, .Protocol = PROTOCOL_CAN, .Vendor = VENDOR_CTRE_CANCODER }
    );
}

void Drivetrain::calculateWheelAnglesAndSpeeds() {
    if ((abs(Drivetrain::m_strife) <= 0.001) && (abs(Drivetrain::m_forwards) <= 0.001)) {
        if(abs(Drivetrain::m_rotation) <= 0.001){
            for (int i = 0; i < Constants::k_NumberOfSwerveModules; i++) {
                Drivetrain::m_motorDirectionAngleSpeed[i].magnitude = 0;
            }
            Drivetrain::sendToSwerveModules();
            return;
        }
    }

    double maxSpeed = 0.0;
    for (int i = 0; i < Constants::k_NumberOfSwerveModules; i += 1) {
        //combine the movement and turning vectors
        double strife;
        double forwards;
        if(m_fieldOriented){
            strife = -m_forwards*sin(m_fieldOrientedOffset) + m_strife*cos(m_fieldOrientedOffset);
            forwards = m_forwards*cos(m_fieldOrientedOffset) + m_strife*sin(m_fieldOrientedOffset);
        } else {
            strife = m_strife;
            forwards = m_forwards;
        }
        double horizontal_motion = (Drivetrain::m_rotation * Drivetrain::c_wheelPositions[i].x) + strife;
        double vertical_motion   = (Drivetrain::m_rotation * Drivetrain::c_wheelPositions[i].y) + forwards;

        // calculate the change in radians
        double newRadians = atan2(horizontal_motion, vertical_motion); //flipped so that 0 is going towards the front
        double radiansChanged = newRadians - Drivetrain::m_motorDirectionAngleSpeed[i].radian;

        // get the final angle of the module
        // if change is greater than 3pi/2 (270 degrees); just turn the other way to go 360 degrees away from it
        // if change is greater than pi (180 degrees); turn to be 180 degrees from it, and reverse wheel
        // if change is greater than pi/2 (90 degrees); turn other way to go 360 degrees away from it and reverse wheel
        // if change is anything else (>0 degrees); turn like normal

        // NOTE: this is split into 2 first (<pi and >pi) then split a second time (>3pi/2 and <3pi/2 OR >pi/2 and <pi/2) for performance reasons
        bool speedReversed = false;
        double directionDelta = 0;
        if (radiansChanged >= M_PI) { //>=pi
            if (radiansChanged >= (1.5 * M_PI)) { //>=3pi/2
                speedReversed = false;
                directionDelta = radiansChanged - (2 * M_PI); // will be <0
            } else { //<3pi/2 and >pi
                speedReversed = true;
                directionDelta = radiansChanged - M_PI; // will be >0
            }
        } else { //<pi
            if (radiansChanged >= (0.5 * M_PI)) { //>=pi/2 and <pi
                speedReversed = true;
                directionDelta = radiansChanged - M_PI; // will be <0
            } else { //<pi/2
                speedReversed = false;
                directionDelta = radiansChanged; // will be >0
            }
        }
        // using the previous radians
        // add the new angle delta to the current quarter turn
        Drivetrain::m_motorDirectionAngleSpeed[i].radian =
            Drivetrain::m_motorDirectionAngleSpeed[i].radian + directionDelta;

        //reverse the speed in the event speedReversed is true
        double speed =
            std::sqrt(std::pow(horizontal_motion, 2) + std::pow(vertical_motion, 2))
            * (speedReversed ? -1 : 1);

        Drivetrain::m_motorDirectionAngleSpeed[i].magnitude = speed;

        //absolute value to prevent random reversing of the wheels (and to make it the magnitude instead of the raw value)
        if (abs(speed) > maxSpeed) {
            maxSpeed = abs(speed);
        }
    }

    // make sure the speed does not go above 1 (AKA: max power);
    // if above 1, scales the speed down to have the max speed at 1
    if (maxSpeed > 1.0) {
        for (int i = 0; i < Constants::k_NumberOfSwerveModules; i += 1) {
            m_motorDirectionAngleSpeed[i].magnitude = m_motorDirectionAngleSpeed[i].magnitude / maxSpeed;
        }
    }
    Drivetrain::sendToSwerveModules();
}

void Drivetrain::enableFieldCentric() {
    Drivetrain::m_fieldOriented = true;
}

void Drivetrain::disableFieldCentric() {
    Drivetrain::m_fieldOriented = false;
}

void Drivetrain::toggleFieldCentric() {
    if(m_fieldOriented){
        m_fieldOriented = false;
    } else {
        m_fieldOriented = true;
    }
}

bool Drivetrain::getFieldCentric() {
    return Drivetrain::m_fieldOriented;
}

void Drivetrain::setStrife(double x) {
    Drivetrain::m_strife = x;
    calculateWheelAnglesAndSpeeds();
}

void Drivetrain::setForward(double y) {
    Drivetrain::m_forwards = y;
    calculateWheelAnglesAndSpeeds();
}

void Drivetrain::setRotation(double r) {
    Drivetrain::m_rotation = r;
    calculateWheelAnglesAndSpeeds();
}

void Drivetrain::setMotion(double x, double y, double r) {
    Drivetrain::m_strife = x;
    Drivetrain::m_forwards = y;
    Drivetrain::m_rotation = r;
    calculateWheelAnglesAndSpeeds();
}

double Drivetrain::getHeading() {
    return atan2(Drivetrain::m_strife, Drivetrain::m_forwards); //swapped x & y so forwards is 0 radians
}

double Drivetrain::getVelocity() {
    //using pythagorean to find the magnitude of the vector components (forwards and strife)
    return std::sqrt((std::pow(Drivetrain::m_strife, 2) + std::pow(Drivetrain::m_forwards, 2)));
}

void Drivetrain::sendToSwerveModules() {
    for (int i = 0; i < Constants::k_NumberOfSwerveModules; i++) {
        c_wheels[i]->setMotion(
            m_motorDirectionAngleSpeed[i].magnitude,
            m_motorDirectionAngleSpeed[i].radian
        );
    }
}

void Drivetrain::resetNavxHeading(){
    Drivetrain::m_navX->ZeroYaw();
}
