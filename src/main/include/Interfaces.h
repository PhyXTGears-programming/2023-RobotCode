#pragma once

namespace Interfaces
{
    namespace
    {
        const int k_USB00 = 0;
        const int k_USB01 = 1;
        const int k_USB02 = 2;
        const int k_USB03 = 3;
        const int k_USB04 = 4;

        // block 0
        const int k_CAN00 = 0; // reserved for PDP
        const int k_CAN01 = 1;
        const int k_CAN02 = 2;
        const int k_CAN03 = 3;
        const int k_CAN04 = 4;
        const int k_CAN05 = 5;
        const int k_CAN06 = 6;
        const int k_CAN07 = 7;
        const int k_CAN08 = 8;
        const int k_CAN09 = 9;

        // block 1
        const int k_CAN10 = 10;
        const int k_CAN11 = 11;
        const int k_CAN12 = 12;
        const int k_CAN13 = 13;
        const int k_CAN14 = 14;
        const int k_CAN15 = 15;
        const int k_CAN16 = 16;
        const int k_CAN17 = 17;
        const int k_CAN18 = 18;
        const int k_CAN19 = 19;

        // block 2
        const int k_CAN20 = 20;
        const int k_CAN21 = 21;
        const int k_CAN22 = 22;
        const int k_CAN23 = 23;
        const int k_CAN24 = 24;
        const int k_CAN25 = 25;
        const int k_CAN26 = 26;
        const int k_CAN27 = 27;
        const int k_CAN28 = 28;
        const int k_CAN29 = 29;

        // block 3
        const int k_CAN30 = 30;
        const int k_CAN31 = 31;
        const int k_CAN32 = 32;
        const int k_CAN33 = 33;
        const int k_CAN34 = 34;
        const int k_CAN35 = 35;
        const int k_CAN36 = 36;
        const int k_CAN37 = 37;
        const int k_CAN38 = 38;
        const int k_CAN39 = 39;

        // block 4
        const int k_CAN40 = 40;
        const int k_CAN41 = 41;
        const int k_CAN42 = 42;
        const int k_CAN43 = 43;
        const int k_CAN44 = 44;
        const int k_CAN45 = 45;
        const int k_CAN46 = 46;
        const int k_CAN47 = 47;
        const int k_CAN48 = 48;
        const int k_CAN49 = 49;

        // DIO
        const int k_DIO0 = 0;
        const int k_DIO1 = 1;
        const int k_DIO2 = 2;
        const int k_DIO3 = 3;
        const int k_DIO4 = 4;
        const int k_DIO5 = 5;
        const int k_DIO6 = 6;
        const int k_DIO7 = 7;
        const int k_DIO8 = 8;
        const int k_DIO9 = 9;

        // AIO
        const int k_AIO0 = 0;
        const int k_AIO1 = 1;
        const int k_AIO2 = 2;
        const int k_AIO3 = 3;

        // Solenoids
        const int k_AIR0 = 0;
        const int k_AIR1 = 1;
        const int k_AIR6 = 6;
        const int k_AIR7 = 7;

        // PWM
        const int k_PWM0 = 0;
        const int k_PWM1 = 1;
    }

    const int k_driverXboxController = k_USB00;
    const int k_operatorXboxController = k_USB01;

    const int k_drivetrainFrontLeftDrive = k_CAN01;
    const int k_drivetrainFrontRightDrive = k_CAN02;
    const int k_drivetrainBackRightDrive = k_CAN03;
    const int k_drivetrainBackLeftDrive = k_CAN04;

    const int k_drivetrainFrontLeftSteer = k_CAN05;
    const int k_drivetrainFrontRightSteer = k_CAN06;
    const int k_drivetrainBackRightSteer = k_CAN07;
    const int k_drivetrainBackLeftSteer = k_CAN08;

    const int k_drivetrainFrontLeftEncoder = k_CAN21;
    const int k_drivetrainFrontRightEncoder = k_CAN22;
    const int k_drivetrainBackRightEncoder = k_CAN23;
    const int k_drivetrainBackLeftEncoder = k_CAN24;

    const int k_kickstandServo = 1;

    namespace Arm {
        // Arm subsystem
        const int k_TurretMotor = k_CAN10;
        const int k_LowJointMotor = k_CAN11;
        const int k_MidJointMotor = k_CAN12;
        const int k_GripperRotateMotor = k_CAN13;
        const int k_GripperGraspMotor = k_CAN14;

        const int k_TurretSensor = k_AIO0; // FIX VALS LATER
        const int k_ElbowSensor = k_AIO1;
        const int k_WristRollSensor = k_AIO2;
        const int k_GripSensor = k_AIO3;

        const int k_ShoulderSensor = k_DIO0;

        const int k_CameraServo = k_PWM0;
    }
}
