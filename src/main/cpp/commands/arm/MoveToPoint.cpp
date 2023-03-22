#include "commands/arm/MoveToPoint.h"

#include "subsystems/arm/motionPath.h"
#include "subsystems/arm/arm.h"
#include "subsystems/arm/armPose.h"

MoveToPointCommand::MoveToPointCommand(ArmSubsystem * arm, Point finalPoint) {
    m_arm = arm;
    m_finalTarget = finalPoint;

    AddRequirements(m_arm);
}

void MoveToPointCommand::Initialize() {
    Point currentPoint = m_arm->getGripPoint();

    m_path.emplace(m_arm->getPathTo(currentPoint, m_finalTarget));
    m_target = m_path->getNextPoint();
}

void MoveToPointCommand::Execute() {
    if(m_finalTarget.isNear(m_target) && m_arm->isNearPoint(m_finalTarget)) {
        return;
    } else if (m_arm->isNearPoint(m_target)) {
        m_target = m_path->getNextPoint();
    }

    m_arm->moveToPoint(m_target);

}

void MoveToPointCommand::End(bool isInterrupted) {
    //heeyaw
    m_arm->stopArm();
}

bool MoveToPointCommand::IsFinished() {
    return !m_arm->isPointSafe(m_target)
        || (m_finalTarget.isNear(m_target) && m_arm->isNearPoint(m_finalTarget));
}
