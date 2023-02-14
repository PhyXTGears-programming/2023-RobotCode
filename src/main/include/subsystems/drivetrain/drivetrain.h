#pragma once
#include "Mandatory.h"

#include <frc2/command/SubsystemBase.h>

#include "subsystems/drivetrain/swerveWheel.h"
#include "util/point.h"
#include "util/polar.h"

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
         * Used to have the swerve drive setup with how ever many drive wheels you need without field oriented
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
         * @returns direction of movement
        */
        double getHeading();

        /**
         * @returns velocity of movement
        */
       double getVelocity();

    private:
        void setupWheels();

        void calculateWheelAnglesAndSpeeds();

        void sendToSwerveModules();

        bool m_fieldOriented = false;

        double m_strife = 0;
        double m_forwards = 0;
        double m_rotation = 0;

        SwerveWheel * c_wheels[Constants::k_NumberOfSwerveModules] = {nullptr};

        Point c_wheelPositions[Constants::k_NumberOfSwerveModules] = {
            Point{/*x=*/(float)(-0.4445/2), /*y=*/(float)(0.4953/2), /*z=*/0.0F}, // -8.75in, 9.75in
            Point{/*x=*/(float)(0.4445/2), /*y=*/(float)(0.4953/2), /*z=*/0.0F}, // 8.75in, 9.75in
            Point{/*x=*/(float)(0.4445/2), /*y=*/(float)(-0.4953/2), /*z=*/0.0F}, // 8.75in, -9.75in
            Point{/*x=*/(float)(-0.4445/2), /*y=*/(float)(-0.4953/2), /*z=*/0.0F} // -8.75in, -9.75in
        };

        // class-wide so we can optimize turning path
        Polar m_motorDirectionAngleSpeed[Constants::k_NumberOfSwerveModules];
};
