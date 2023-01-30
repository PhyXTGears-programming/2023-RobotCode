#pragma once
#include "Mandatory.h"

#include <frc2/command/SubsystemBase.h>

#include "subsystems/drivetrain/swerveWheel.h"

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
         * Turns Field Oriented Driving on reguardless of wether it was on or off previously
        */
        void enableFieldCentric();


        /**
         * Turns Field Oriented Driving on reguardless of wether it was on or off previously
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
         * returns current state of Sield Oriented Driving
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
         * sets the foreward/backwards percentage of the max speed
        */
        void setForeward(double y);

        /**
         * sets the percentage of the max rotation speed
        */
        void setRotation(double r);

        /**
         * sets all parameters of the robot movement
         * 
         * @param x sets the left/right percentage of the max speed
         * @param y sets the foreward/backwards percentage of the max speed
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

        void calculateWheelPositionsAndSpeeds();

        bool m_fieldOriented = false;

        double m_strife = 0;
        double m_forewards = 0;
        double m_rotation = 0;

        SwerveWheel * c_wheels[Constants::k_NumberOfSwerveModules] = {nullptr};

        int m_wheelPositions[Constants::k_NumberOfSwerveModules][2] = {};
};