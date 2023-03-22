#include "commands/arm/ArmTeleopCommand.h"
#include "subsystems/arm/arm.h"

#include <frc/XboxController.h>

#include <cmath>

#define JOYSTICK_DEADZONE 0.15
#define DEADZONE(x, min, max) ((std::abs((x)) < JOYSTICK_DEADZONE) \
    ? 0.0 \
    : ((x) + std::copysign(1.0 - JOYSTICK_DEADZONE, (x))) \
        / (1.0 - JOYSTICK_DEADZONE) \
        * ((max) - (min)) \
        + std::copysign((min), (x)))

const double k_maxPointSpeed = 0.005;
const double k_maxPointRotSpeed = (2.0 * M_PI / 10.0) * 0.02;   // radians per second in 20ms.
const double k_maxWristRotSpeed = (2.0 * M_PI / 2.0) * 0.02;
const double k_maxGripSpeed = 0.1;

ArmTeleopCommand::ArmTeleopCommand(ArmSubsystem* arm, frc::XboxController* operatorController) {
    c_operatorController = operatorController;
    c_arm = arm;

    AddRequirements(c_arm);
}

void ArmTeleopCommand::Initialize() {
    // Update target to current point so arm doesn't move unexpectedly.
    m_target = c_arm->getGripPoint();
    c_arm->moveToPoint(m_target);
}

void ArmTeleopCommand::Execute() {
    {
        Vector offsetLX;
        Vector offsetLY;
        Vector offsetRY;
        double gripMag = std::sqrt(std::pow(m_target.x,2)+std::pow(m_target.y,2));
        double gripDir = std::atan2(m_target.x, m_target.y);  // 0 deg == y axis

        // Rotate turret. Speed of rotation is reduced the further the arm reaches.
        double leftX = c_operatorController->GetLeftX();
        // Square input to improve fidelity.
        leftX = DEADZONE(leftX, 0.0, 1.0);
        if (0.0 != leftX) {
            // (+) leftX should move turret clockwise.
            double dir = gripDir + (leftX * k_maxPointRotSpeed) / std::max(gripMag,1.0);
            Point point1(
                gripMag * std::sin(dir),
                gripMag * std::cos(dir),
                m_target.z
            );
            offsetLX = point1 - m_target;
        }

        // Extend/retract gripper from/to turret.
        double leftY = -c_operatorController->GetLeftY(); /* Invert so + is forward */
        // Square input to improve fidelity.
        leftY = DEADZONE(leftY, 0.0, 1.0);
        if (0.0 != leftY) {
            // (+) leftY should move away from turret.
            double mag = gripMag + (leftY * k_maxPointSpeed);
            Point point1(
                mag * std::sin(gripDir),
                mag * std::cos(gripDir),
                m_target.z
            );
            offsetLY = point1 - m_target;
        }

        // Move gripper up or down.
        double rightY = -c_operatorController->GetRightY();    /* Invert so + is up */
        // Square input to improve fidelity.
        rightY = DEADZONE(rightY, 0.0, 1.0);
        if (0.0 != rightY) {
            // (+) rightY should move gripper up.
            offsetRY = Vector(0.0, 0.0, rightY * k_maxPointSpeed);
        }

        Point desiredTarget = m_target + offsetLX + offsetLY + offsetRY;

        // Attempt to move to desired target, and ignore point if deemed unsafe.
        std::optional<Point> optSafePoint = c_arm->moveToPoint(desiredTarget);

        if (optSafePoint) {
            // moveToPoint yielded a safe point that employs safety limits.
            // Reset target to this safe point so target doesn't attempt to travel
            // beyond arm limits.
            m_target = *optSafePoint;
        }
    }

    // Rotate wrist clockwise or counterclockwise.
    {
        double leftTrigger = c_operatorController->GetLeftTriggerAxis();
        double rightTrigger = c_operatorController->GetRightTriggerAxis();
        double trigger = leftTrigger - rightTrigger;
        // Square input to improve fidelity.
        trigger = DEADZONE(trigger, 0.0, 1.0);

        // Use current angle as default so wrist doesn't move wildly upon enable.
        double wristTargetAngle = c_arm->getWristRollAngle();

        if (0.0 != trigger) {
            // (+) trigger should rotate wrist counter-clockwise, looking down forearm.
            wristTargetAngle += trigger * k_maxWristRotSpeed;
        }

        // Move/hold wrist angle.
        c_arm->setWristRollAngle(wristTargetAngle);
    }

    // Open and close gripper.
    {
        double bumper =
            (c_operatorController->GetRightBumper() ? 1.0 : 0.0)
            - (c_operatorController->GetLeftBumper() ? 1.0 : 0.0);

        // Use current position as default so grip doesn't move wildly upon enable.
        double gripTargetPos = c_arm->getGrip();

        if (0.0 != bumper) {
            gripTargetPos += bumper * k_maxGripSpeed;
        }

        // Move/hold grip.
        c_arm->setGrip(gripTargetPos);
    }
}

void ArmTeleopCommand::End(bool interrupted) {
    // Update target to current point so arm stops.
    // Cannot stop motors because shoulder will backdrive and fall.
    m_target = c_arm->getGripPoint();
    c_arm->moveToPoint(m_target);
}

bool ArmTeleopCommand::IsFinished() {
    return false; // dont end because then we wont be able to drive
}

void ArmTeleopCommand::resetTarget(){
    m_target = c_arm->getGripPoint();
}
