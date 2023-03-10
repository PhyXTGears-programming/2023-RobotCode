#pragma once

#include "Mandatory.h"

#include "subsystems/drivetrain/drivetrain.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>

#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/drivetrain/odometry.h"
#include "util/bezier.h"
#include "util/point.h"

class DriveAutoCommand : public frc2::CommandHelper<frc2::CommandBase, DriveAutoCommand> {
public:
    DriveAutoCommand(Drivetrain* drivetrain, Odometry* odometry);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
    void changeSetpoint(double x, double y);
    void bezierDrive(Bezier curve, double time);
    Point calculateBezierPosition();
private:
    Drivetrain* c_drivetrain;
    Odometry* c_odometry;
    Bezier m_curve;
    double m_bezierTime;
    bool m_finished = false;

    double m_bezierX;
    double m_bezierY;

    double c_pidTolerance = 0.1;
    frc2::PIDController c_pidXController = frc2::PIDController{0.1, 0, 0};
    frc2::PIDController c_pidYController = frc2::PIDController{0.1, 0, 0};
};