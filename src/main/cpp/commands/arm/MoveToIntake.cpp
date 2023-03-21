#include "commands/arm/MoveToIntake.h"

#include "subsystems/arm/motionPath.h"
#include "subsystems/arm/arm.h"
#include "subsystems/arm/armPose.h"

MoveToIntakeCommand::MoveToIntakeCommand(ArmSubsystem * arm, Point currentPoint) {
    m_arm = arm;
    m_currentPoint = currentPoint;

    AddRequirements(m_arm);
}

void MoveToIntakeCommand::Initialize() {
    m_finalTarget = m_arm->m_pointIntake;
    Point currentPoint = m_arm->getGripPoint();

    m_path.emplace(std::move(m_arm->getPathTo(currentPoint, m_finalTarget)));
    m_target = m_path->getNextPoint();
}

void MoveToIntakeCommand::Execute() {
    if(m_finalTarget.isNear(m_target) && m_arm->isNearPoint(m_finalTarget)) {
        return;
    } else if (m_arm->isNearPoint(m_target)) {
        m_target = m_path->getNextPoint();
    }

    m_arm->moveToPoint(m_target);

}

void MoveToIntakeCommand::End(bool isInterrupted) {
    //heeyaw
    m_arm->stopArm();
}

bool MoveToIntakeCommand::IsFinished() {
    return !m_arm->isPointSafe(m_target)
        || (m_finalTarget.isNear(m_target) && m_arm->isNearPoint(m_finalTarget));
}
