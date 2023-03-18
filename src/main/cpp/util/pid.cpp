#include "util/pid.h"

#include <algorithm>
#include <iostream>

PID::PID(double p, double i, double d, double ff, double acceptableError, double minOutput,
    double maxOutput, double izone, units::second_t period) {
    m_proportional = p;
    m_integral     = i;
    m_derivation   = d;
    m_feedForward  = ff;

    m_acceptableError = acceptableError;

    m_iZone     = izone;
    m_minOutput = minOutput;
    m_maxOutput = maxOutput;

    if (0_s <= period) {
        std::cerr << "PID period must be above zero" << std::endl;
        m_period = 20_ms;
    } else {
        m_period = period;
    }
}

double PID::calculate(double measuredValue) {
    double output = 0.0;
    double error  = m_target - measuredValue;

    m_velocityError = (error - m_previousError) / m_period.value();

    m_previousError = error;

    if (std::abs(error) <= m_acceptableError) {
        return 0.0;
    }

    if (std::abs(error) <= m_iZone) {
        m_accumulator += error * m_period.value();
    }

    output += error * m_proportional;
    output += m_accumulator * m_integral;
    output += m_velocityError * m_derivation;
    output += std::copysign(m_feedForward, output);

    return std::clamp(output, m_minOutput, m_maxOutput);
}

void PID::setTarget(double target) {
    m_target = target;
}

void PID::setP(double p) {
    m_proportional = p;
}

void PID::setI(double i) {
    m_integral = i;
}

void PID::setD(double d) {
    m_derivation = d;
}

void PID::setFeedForward(double ff) {
    m_feedForward = ff;
}

void PID::reset() {
    m_previousError = 0.0;
    m_accumulator   = 0.0;
}

double PID::getError() {
    return m_previousError;
}

double PID::getVelocityError() {
    return m_velocityError;
}
