#include "Mandatory.h"

#include "commands/drivetrain/driveAutoCommand.h"

#include "subsystems/drivetrain/drivetrain.h"

#include <frc/controller/PIDController.h>
#include <frc/Timer.h>
#include <math.h>

#include "subsystems/drivetrain/odometry.h"

DriveAutoCommand::DriveAutoCommand(Drivetrain* drivetrain, Odometry* odometry) {
    c_drivetrain = drivetrain;
    c_odometry = odometry; //not an FRC subsystem, so no need for a subsystem base

    c_pidXController.SetTolerance(c_pidTolerance);
    c_pidXController.SetTolerance(c_pidTolerance);

    AddRequirements(c_drivetrain);
}

void DriveAutoCommand::Initialize() {
    c_drivetrain->setMotion(0, 0, 0); // make sure nothing is moving
    m_finished = false;
}

void DriveAutoCommand::Execute() {
    double strafe = 0;
    double forwards = 0;
    Point currentPosition = c_odometry->getPosition();
    Point calc_position = calculateBezierPosition(); // calculate the position on the bezier curve based on the current time
    changeSetpoint(calc_position.x, calc_position.y); // give the position to the PID controller

    //determine what to do with the position (if within the tolerance, don't do anything, else; calculate strafe and forwards)
    if(c_pidXController.AtSetpoint()){
        strafe = 0; //if it is at its setpoint, don't do anything
    } else {
        strafe = c_pidXController.Calculate(currentPosition.x);
    }

    if(c_pidYController.AtSetpoint()){
        forwards = 0; //if at setpoint, don't do anything
    } else {
        forwards = c_pidYController.Calculate(currentPosition.y);
    }
    if((strafe == 0)&&(forwards==0)){
        m_finished = true;
    }
}

void DriveAutoCommand::End(bool interrupted){
    // set the movement to 0 just as a safety
    c_drivetrain->setMotion(0, 0, 0);
}

bool DriveAutoCommand::IsFinished(){
    return m_finished; // if it is at its setpoint from execute, then call it done
}

void DriveAutoCommand::bezierDrive(Bezier curve, double time){
    m_curve = curve;
    m_bezierTime = time;
}

void DriveAutoCommand::changeSetpoint(double x, double y){
    c_pidXController.SetSetpoint(x);
    c_pidYController.SetSetpoint(y);
}

Point DriveAutoCommand::calculateBezierPosition(){
    Point point;
    double t = frc::Timer::GetFPGATimestamp().value()/m_bezierTime; // clamp the output to be between 0 and 1
    point.x = (pow((1-t),3))*m_curve.x_a + (pow((1-t),2))*(t)*m_curve.x_b + (1-t)*pow(t,2)*m_curve.x_c + pow(t,3)*m_curve.x_d;
    point.y = (pow((1-t),3))*m_curve.y_a + (pow((1-t),2))*(t)*m_curve.y_b + (1-t)*pow(t,2)*m_curve.y_c + pow(t,3)*m_curve.y_d;

    return point;
}