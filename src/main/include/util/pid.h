#pragma once

#include <cmath>
#include <units/time.h>

class PID {
public:
    PID(double p = 0.0, double i = 0.0, double d = 0.0, double ff = 0.0,
        double acceptableError = 0.1, double minOutput = -1.0, double maxOutput = 1.0,
        double izone = INFINITY, units::second_t period = 20_ms);

    PID(PID const &) = default;

    double calculate(double measuredValue);

    void setTarget(double target);
    void setP(double p);
    void setI(double i);
    void setD(double d);
    void setFeedForward(double ff);

    void reset();

    double getError();
    double getVelocityError();

private:
    double m_proportional;
    double m_integral;
    double m_derivation;
    double m_feedForward;

    double m_target;

    double m_previousError = 0.0;
    double m_velocityError = 0.0;

    double m_accumulator = 0.0;
    double m_acceptableError;
    double m_iZone = INFINITY;

    double m_minOutput = -1.0;
    double m_maxOutput = 1.0;

    units::second_t m_period = 20_ms;
};
