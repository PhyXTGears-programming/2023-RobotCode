#include "commands/auto/AccelerateCommand.h"
#include "util/math.h"

#include <cmath>

AccelerateCommand::AccelerateCommand(double forward, double strafe, double percent, Drivetrain * drive) {
    c_drive = drive;

    c_finalForward = forward;
    c_finalStrafe = strafe;

    c_percent = percent;

    AddRequirements(c_drive);
}

void AccelerateCommand::Initialize() {
    m_prevForward = c_drive->getForward();
    m_prevStrafe = c_drive->getStrafe();
}

void AccelerateCommand::Execute() {
    m_prevForward = (c_percent * c_finalForward) + ((1.0 - c_percent) * m_prevForward);
    m_prevStrafe = (c_percent * c_finalStrafe) + ((1.0 - c_percent) * m_prevStrafe);

    c_drive->setMotion(m_prevStrafe, m_prevForward, 0.0);
}

void AccelerateCommand::End(bool isInterrupted) {
    // Hopefully, a subsequent command can restore the momentum.
    c_drive->setMotion(0.0, 0.0, 0.0);
}

bool AccelerateCommand::IsFinished() {
    return isNearZero(std::abs(c_finalForward - m_prevForward))
        && isNearZero(std::abs(c_finalStrafe - m_prevStrafe));
}