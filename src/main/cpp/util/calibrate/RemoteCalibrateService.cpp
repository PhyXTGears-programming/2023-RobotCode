
#include "Mandatory.h"

#include "util/calibrate/RemoteCalibrateService.h"

#include <frc/DriverStation.h>

#include <wpinet/uv/Buffer.h>

#include <bit>
#include <concepts>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>

#define COMMAND_REQUEST 0x1
#define COMMAND_SET_VELOCITY 0x2

#define SUBSYSTEM_ARM 0x1

#define ARM_LOW_JOINT 0x1
#define ARM_MID_JOINT 0x2
#define ARM_WRIST_ROLL 0x3
#define ARM_GRIPPER 0x4

static inline double htond(double);
static inline double ntohd(double);
static constexpr double byteswap(double value) noexcept;

std::shared_ptr<RemoteCalibrationService>
RemoteCalibrationService::Create(const std::shared_ptr<wpi::uv::Loop> & loop) {
    auto self = std::make_shared<RemoteCalibrationService>();

    self->cServer = wpi::uv::Udp::Create(loop);
    self->cServer->Bind("0.0.0.0", 1720);

    self->cServer->received.connect(
        [svc = self.get(
         )](wpi::uv::Buffer buffer, const struct sockaddr & sock, unsigned int _flags) {
            // Do nothing if robot is not enabled in teleop mode.
            if (!frc::DriverStation::IsTeleopEnabled()) {
                return;
            }

            // Do nothing if remote calibration command wasn't sent over ipv4.
            if (AF_INET != sock.sa_family) {
                return;
            }

            struct sockaddr_in * addr = (struct sockaddr_in *)&sock.sa_data;
            char addr_str[16]         = {0};

            inet_ntop(AF_INET, addr, addr_str, sizeof(addr_str) - 1);

            std::cout << "Remote arm calibration packet from '" << addr_str << "'" << std::endl;

            // Parse the datagram.
            // Protocol:
            //    Request(max 255)
            //
            //    Request := CommandRequest (RequestArmTelemetry)
            //            |  CommandSetVelocity LowJointVelocity MidJointVelocity WristRollVelocity
            //               GripperVelocity
            //
            //    CommandRequest     := 0x1
            //    CommandSetVelocity := 0x2
            //
            //    RequestArmTelemetry := 0x1
            //
            //    *Velocity := <64-bit big-endian double between -1.0..1.0>

            auto request     = buffer.data();
            auto requestType = *request.subspan<0, 1>().data();

            switch (requestType) {
                case COMMAND_REQUEST: {
                    // Do nothing if no callback set.
                    if (nullptr == svc->onRequestTelemetry) {
                        return;
                    }

                    auto telemetry = svc->onRequestTelemetry();

                    break;
                }

                case COMMAND_SET_VELOCITY: {
                    // Do nothing if no callback set.
                    if (nullptr == svc->onSetVelocityRequest) {
                        return;
                    }

                    SetVelocityPayload payload{
                        ntohd(*(double *)request.subspan(1, 8).data()) /* turret */,
                        ntohd(*(double *)request.subspan(9, 8).data()) /* low joint */,
                        ntohd(*(double *)request.subspan(17, 8).data()) /* mid joint */,
                        ntohd(*(double *)request.subspan(25, 8).data()) /* wrist roll */,
                        ntohd(*(double *)request.subspan(33, 8).data()) /* gripper */,
                    };

                    svc->onSetVelocityRequest(std::move(payload));

                    break;
                }

                default:
                    std::cerr << "remote calibration service: unknown request type: " << requestType
                              << std::endl;
            }
        }
    );

    return self;
}

static inline double htond(double d) {
    double ret;

    if (std::endian::native == std::endian::little) {
        ret = byteswap(d);
    } else {
        ret = d;
    }

    return ret;
}

static inline double ntohd(double d) {
    double ret;

    if (std::endian::native == std::endian::little) {
        ret = byteswap(d);
    } else {
        ret = d;
    }

    return ret;
}

static constexpr double byteswap(double value) noexcept {
    auto value_representation = std::bit_cast<std::array<std::byte, sizeof(double)>>(value);
    std::ranges::reverse(value_representation);
    return std::bit_cast<double>(value_representation);
}
