#pragma once

#include <functional>

#include <wpinet/uv/Loop.h>
#include <wpinet/uv/Udp.h>

struct SetVelocityPayload {
    double turretVelocity;
    double lowJointVelocity;
    double midJointVelocity;
    double wristRollVelocity;
    double gripperVelocity;
};

struct VelocityState {
    double velocity;
    uint32_t ticksUntilReset;
};

struct CalibrationState {
    VelocityState turret;
    VelocityState lowJoint;
    VelocityState midJoint;
    VelocityState wristRoll;
    VelocityState gripper;
};

struct TelemetryData {
    double posTurret;
    double posLowJoint;
    double posMidJoint;
    double posWristRoll;
    double posGripper;
};

class RemoteCalibrationService {
public:
    static std::shared_ptr<RemoteCalibrationService>
    Create(const std::shared_ptr<wpi::uv::Loop> & loop);

    std::function<void(SetVelocityPayload &&)> onSetVelocityRequest = nullptr;
    std::function<TelemetryData && ()> onRequestTelemetry           = nullptr;

private:
    std::shared_ptr<wpi::uv::Udp> cServer = nullptr;
};
