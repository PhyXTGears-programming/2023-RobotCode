#include "commands/arm/ArmTeleopCommand.h"
#include "subsystems/arm/arm.h"

#include <frc/XboxController.h>

#include <cmath>

#define JOYSTICK_DEADZONE 0.2
#define DEADZONE(x, min, max)                                                                      \
    ((std::abs((x)) < JOYSTICK_DEADZONE) ? 0.0                                                     \
                                         : ((x) + std::copysign(1.0 - JOYSTICK_DEADZONE, (x)))     \
                                                   / (1.0 - JOYSTICK_DEADZONE) * ((max) - (min))   \
                                               + std::copysign((min), (x)))

ArmTeleopCommand::ArmTeleopCommand(ArmSubsystem * arm, frc::XboxController * operatorController) {
    c_operatorController = operatorController;
    c_arm                = arm;

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
        double armExtension = std::sqrt(std::pow(m_target.x, 2) + std::pow(m_target.y, 2));

        // Rotate turret. Speed of rotation is reduced the further the arm reaches.
        double leftX = c_operatorController->GetLeftX();
        // Square input to improve fidelity.
        leftX = DEADZONE(leftX, 0.0, 1.0);
        if (0.0 != leftX) {
            // (+) leftX should move turret clockwise.
            double rotSpeed =
                (leftX * Constants::Arm::k_maxTurnSpeed) / std::max(armExtension * 2.0, 1.0);

            if (isNearZero(rotSpeed, 0.05)) {
                // Stop turret.
                c_arm->setTurretSpeed(0.0);
            } else {
                c_arm->setTurretSpeed(rotSpeed);
            }
        } else {
            // Stop turret.
            c_arm->setTurretSpeed(0.0);
        }

        // Extend/retract gripper from/to turret.
        double leftY = -c_operatorController->GetLeftY(); /* Invert so + is forward */
        // Square input to improve fidelity.
        leftY = DEADZONE(leftY, 0.0, 1.0);
        if (0.0 != leftY) {
            // (+) leftY should move away from turret.
            offsetLY = Vector(leftY * Constants::Arm::k_maxPointSpeed, 0.0);
        }

        // Move gripper up or down.
        double rightY = -c_operatorController->GetRightY(); /* Invert so + is up */
        // Square input to improve fidelity.
        rightY = DEADZONE(rightY, 0.0, 1.0);
        if (0.0 != rightY) {
            // (+) rightY should move gripper up.
            offsetRY = Vector(0.0, rightY * Constants::Arm::k_maxPointSpeed);
        }

        Point desiredTarget = m_target + offsetLX + offsetLY + offsetRY;
        // Clamp max distance from current position.
        {
            Point currentPos = c_arm->getGripPoint();
            Vector offset    = desiredTarget - currentPos;
            double mag       = offset.len();
            offset           = offset * (std::min(mag, 0.0254 * 4.0 /* 2 inches */) / mag);
            desiredTarget    = currentPos + offset;
        }

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
    if (false) {
        /// TODO: Fix analog sensor
        double leftTrigger  = c_operatorController->GetLeftTriggerAxis();
        double rightTrigger = c_operatorController->GetRightTriggerAxis();
        double trigger      = leftTrigger - rightTrigger;
        // Square input to improve fidelity.
        trigger = DEADZONE(trigger, 0.0, 1.0);

        // Use current angle as default so wrist doesn't move wildly upon enable.
        double wristTargetAngle = c_arm->getWristRollAngle();

        if (0.0 != trigger) {
            // (+) trigger should rotate wrist counter-clockwise, looking down forearm.
            wristTargetAngle += trigger * Constants::Arm::k_maxWristRotSpeed;
        }

        // Move/hold wrist angle.
        c_arm->setWristRollAngle(wristTargetAngle);
    } else {
        // Fallback: manual operation
        double leftTrigger  = c_operatorController->GetLeftTriggerAxis();
        double rightTrigger = c_operatorController->GetRightTriggerAxis();
        double trigger      = rightTrigger - leftTrigger;
        // Square input to improve fidelity.
        trigger = DEADZONE(trigger, 0.0, 0.1);

        c_arm->setWristRollSpeed(trigger);
    }

    // Open and close gripper.
    {
        double bumper = (c_operatorController->GetRightBumper() ? 1.0 : 0.0)
                        - (c_operatorController->GetLeftBumper() ? 1.0 : 0.0);

        // Disabled 2023 (jcc) - Position sensor broke.  Will not replace.
        if (false) {
            // Use current position as default so grip doesn't move wildly upon enable.
            double gripTargetPos = c_arm->getGrip();

            if (0.0 != bumper) {
                gripTargetPos += bumper * Constants::Arm::k_maxGripSpeed;
            }

            // Move/hold grip.
            c_arm->setGrip(gripTargetPos);
        }

        if (NEAR_ZERO_METERS < std::abs(bumper)) {
            c_arm->setGripSpeed(bumper * Constants::Arm::k_maxGripSpeed);
        } else {
            // Stop motor
            c_arm->setGripSpeed(0.0);
        }
    }
}

void ArmTeleopCommand::End(bool interrupted) {
    // Update target to current point so arm stops.
    // Cannot stop motors because shoulder will backdrive and fall.
    m_target = c_arm->getGripPoint();
    c_arm->moveToPoint(m_target);
    // Stop turret.
    c_arm->setTurretSpeed(0.0);
}

bool ArmTeleopCommand::IsFinished() {
    return false; // dont end because then we wont be able to move the arm
}

void ArmTeleopCommand::resetTarget() {
    m_target = c_arm->getGripPoint();
}
