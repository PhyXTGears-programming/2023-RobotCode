#pragma once
#include "Mandatory.h"

#include <frc2/command/SubsystemBase.h>

#include "subsystems/drivetrain/swerveWheel.h"
#include "util/geom.h"
#include "util/polar.h"

#include <AHRS.h>
#include <frc/SPI.h>
#include <frc2/command/PIDCommand.h>
/*
NOTE ON UNITS:

all math in here is based on the units of meters, radians and seconds.

there should not be any other units in here, so if there turns out to be,
it should be removed or put in a debug preprocessor.
*/

class Drivetrain : public frc2::SubsystemBase {
public:
    /**
     * Used to have the swerve drive setup with how ever many drive wheels you need
     *
     * @param fieldOriented wether the robot should be in field oriented mode (non-functional)
     */
    Drivetrain(bool fieldOriented);

    /**
     * Used to have the swerve drive setup with how ever many drive wheels you need without field
     * oriented
     */
    Drivetrain();

    ~Drivetrain();

    void Periodic() override;

    /**
     * Turns Field Oriented Driving on regardless of whether it was on or off previously
     */
    void enableFieldCentric();

    /**
     * Turns Field Oriented Driving on regardless of whether it was on or off previously
     */
    void disableFieldCentric();

    /**
     * Sets field oriented driving to the other setting
     *
     * - if Field Oriented Driving was off, it will now be on
     * - if Field Oriented Driving was on, it will not be off
     */
    void toggleFieldCentric();

    /**
     * returns current state of Field Oriented Driving
     *
     * - if TRUE, then Field Oriented Driving is enabled
     * - if FALSE, then Field Oriented Driving is disabled
     */
    bool getFieldCentric();

    /**
     * sets the left/right percentage of the max speed
     */
    void setStrife(double x);

    /**
     * sets the forward/backwards percentage of the max speed
     */
    void setForward(double y);

    /**
     * sets the percentage of the max rotation speed
     */
    void setRotation(double r);

    /**
     * sets all parameters of the robot movement
     *
     * @param x sets the left/right percentage of the max speed
     * @param y sets the forward/backwards percentage of the max speed
     * @param r sets the percentage of the max rotation speed
     */
    void setMotion(double x, double y, double r);

    /**
     * @returns direction of movement (this is a calculation based on controller inputs)
     */
    double getCalculatedHeading();

    /**
     * @returns the field heading from the NavX
     */
    double getFieldHeading();

    /**
     * @returns velocity of movement (this is a calculation based on controller inputs)
     */
    double getCalculatedVelocity();

    /**
     * @param module the module to get the heading from
     *
     * @returns the field heading from the modules
     */
    double getMovementHeading(int module);

    /**
     * @param module the module to get the heading from
     *
     * @returns velocity of movement calculated from the Talon built-in encoders
     */
    double getMovementVelocity(int module);

    /**
     * resets the yaw to be 0 in the NavX (parallel to the field floor)
     */
    void resetNavxHeading();

    /**
     * @param restrictMovement whether the Drivetrain should lock out any movement
     */
    void lockMovement(bool restrictMovement);

    void enableHeadingControl();
    void disableHeadingControl();
    void toggleHeadingControl();
    bool getHeadingControlState();
    void headingControl(bool blockRotationSets);
    void setHeadingSetpoint(double setpoint);
    double getHeadingSetpoint();

private:
    void setupWheels();

    void calculateWheelAnglesAndSpeeds();

    void sendToSwerveModules();

    bool m_fieldOriented         = false;
    double m_fieldOrientedOffset = 0;

    double m_strife   = 0;
    double m_forwards = 0;
    double m_rotation = 0;

    bool m_forceLockMovement = false;

    SwerveWheel * c_wheels[Constants::k_NumberOfSwerveModules] = { nullptr };

    Point c_wheelPositions[Constants::k_NumberOfSwerveModules] = {
        Point{ /*y=*/(float)(-0.4953 / 2), /*x=*/(float)(-0.4445 / 2) }, // -8.75in, -9.75in
        Point{ /*y=*/(float)(0.4953 / 2), /*x=*/(float)(-0.4445 / 2) },  // -8.75in, 9.75in
        Point{ /*y=*/(float)(0.4953 / 2), /*x=*/(float)(0.4445 / 2) },   // 8.75in, 9.75in
        Point{ /*y=*/(float)(-0.4953 / 2), /*x=*/(float)(0.4445 / 2) }   // 8.75in, -9.75in
    };

    AHRS * m_navX = new AHRS(frc::SPI::Port::kMXP);

    // class-wide so we can optimize turning path
    Polar m_motorDirectionAngleSpeed[Constants::k_NumberOfSwerveModules];

    bool m_headingControlEnabled = false;
    frc2::PIDController c_headingControlPID{ 0.4, 0.0, 0 };
    double m_headingControlSetpoint = 0.0;
};
