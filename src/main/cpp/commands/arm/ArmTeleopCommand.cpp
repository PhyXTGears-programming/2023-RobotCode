#include "commands/arm/ArmTeleopCommand.h"
#include "subsystems/arm/arm.h"

#include <frc/XboxController.h>

#include <cmath>

#define JOYSTICK_DEADZONE 0.1
#define DEADZONE(input) ((std::abs(input) < JOYSTICK_DEADZONE) ? 0.0 : input)

const double k_maxPointSpeed = 0.005;
const double k_maxPointRotSpeed = 2.0 * M_PI / 10.0;   // radians per second in 20ms.
const double k_maxWristRotSpeed = 2.0 * M_PI / 2.0;
const double k_maxGripSpeed = 0.1;

ArmTeleopCommand::ArmTeleopCommand(ArmSubsystem* arm, frc::XboxController* operatorController) {
    c_operatorController = operatorController;
    c_arm = arm;

    AddRequirements(c_arm);
}

void ArmTeleopCommand::Initialize() {
    // Update target to current point so arm doesn't move unexpectedly.
    m_target = c_arm->getGripPoint();
}

void ArmTeleopCommand::Execute() {
    {
        Vector gripVec(m_target.x, m_target.y, m_target.z);
        double gripMag = gripVec.len();
        double gripDir = std::atan2(gripVec.x, gripVec.y);  // 0 deg == y axis

        // Rotate turret. Speed of rotation is reduced the further the arm reaches.
        double leftX = c_operatorController->GetLeftX();
        // Square input to improve fidelity.
        leftX = std::copysign(std::clamp(leftX * leftX, -1.0, 1.0), leftX);
        leftX = DEADZONE(leftX);
        if (0.0 != leftX) {
            // (+) leftX should move turret clockwise.
            gripDir = gripDir + leftX * k_maxPointRotSpeed / gripMag;
            Vector offset(
                gripMag * std::cos(gripDir),
                gripMag * std::sin(gripDir),
                0.0
            );
            m_target = m_target + offset;
        }

        // Extend/retract gripper from/to turret.
        double leftY = -c_operatorController->GetLeftY(); /* Invert so + is forward */
        // Square input to improve fidelity.
        leftY = std::copysign(std::clamp(leftY * leftY, -1.0, 1.0), leftY);
        leftY = DEADZONE(leftY);
        if (0.0 != leftY) {
            // (+) leftY should move away from turret.
            gripMag = gripMag + leftY * k_maxPointSpeed;
            Vector offset(
                gripMag * std::cos(gripDir),
                gripMag * std::sin(gripDir),
                0.0
            );
            m_target = m_target + offset;
        }

        // Move gripper up or down.
        double rightY = -c_operatorController->GetRightY();    /* Invert so + is up */
        // Square input to improve fidelity.
        rightY = std::copysign(std::clamp(rightY * rightY, -1.0, 1.0), rightY);
        rightY = DEADZONE(rightY);
        if (0.0 != rightY) {
            // (+) rightY should move gripper up.
            m_target = m_target + Vector(0.0, 0.0, rightY * k_maxPointSpeed);
        }

        c_arm->moveToPoint(m_target);
    }

    // Rotate wrist clockwise or counterclockwise.
    {
        double leftTrigger = c_operatorController->GetLeftTriggerAxis();
        double rightTrigger = c_operatorController->GetRightTriggerAxis();
        double trigger = leftTrigger - rightTrigger;
        // Square input to improve fidelity.
        trigger = std::copysign(std::clamp(trigger * trigger, -1.0, 1.0), trigger);
        trigger = DEADZONE(trigger);

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
}

bool ArmTeleopCommand::IsFinished() {
    return false; // dont end because then we wont be able to drive
}