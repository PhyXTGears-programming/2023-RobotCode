#include "commands/arm/MoveToPoint.h"

#include "subsystems/arm/motionPath.h"
#include "subsystems/arm/arm.h"
#include "subsystems/arm/armPose.h"

MoveToPointCommand::MoveToPointCommand(ArmSubsystem * arm, Point finalPoint) {
    c_arm = arm;
    c_finalTarget = finalPoint;

    AddRequirements(c_arm);
}

void MoveToPointCommand::Initialize() {
    Point currentPoint = c_arm->getGripPoint();

    m_path.emplace(c_arm->getPathTo(currentPoint, c_finalTarget));
    m_target = m_path->getNextPoint();
}

void MoveToPointCommand::Execute() {
    if(c_finalTarget.isNear(m_target) && c_arm->isNearPoint(c_finalTarget)) {
        return;
    } else if (c_arm->isNearPoint(m_target)) {
        m_target = m_path->getNextPoint();
    }

    c_arm->moveToPoint(m_target);

}

void MoveToPointCommand::End(bool isInterrupted) {
    //heeyaw
    c_arm->stopArm();
}

bool MoveToPointCommand::IsFinished() {
    return !c_arm->isPointSafe(m_target)
        || (c_finalTarget.isNear(m_target) && c_arm->isNearPoint(c_finalTarget));
}
