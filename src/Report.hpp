#ifndef REPORT_HPP
#define REPORT_HPP
#include "ezC2X/facility/cam/Cam.pb.h"
#include "ezC2X/core/geographic/Origin.hpp"
#include <EgoPos.hpp>

struct ReadableCam
{
    /*
    Many of these values are stored as a value/confidence pair in the actual CAM.
    In order to simplify things we only use values, not the confidence.
    */

    uint32_t id;
    EgoPos pos;
    double speed;
    double heading;
    uint8_t driveDirection;
    uint64_t generationDeltaTime;   //generationDeltaTime = TimestampIts mod 65 536. 
                                    //TimestampIts represents an integer value in milliseconds since
                                    // 2004-01-01T00:00:00:000Z as defined in ETSI TS 102 894-2.
    double longitudinalAcceleration;
    double curvature;
    double curvatureCalculationMode;
    double yawRate;
    //The following parameters are optional in CA messages
    uint8_t accelerationControl;
    double lanePosition;
    double steeringWheelAngle;
    double lateralAcceleration;
    double verticalAcceleration;
    bool attacking;
    size_t fingerprint;
};

struct MetaData
{
    uint64_t timeOnReceive;
    EgoPos posOnReceieve;
    std::string id;

};

class Report{
public:
    Report();

    ~Report();

    Report(ezC2X::Cam cam, MetaData meta);

    ReadableCam getCam();

    MetaData getMetaData();

    uint8_t accelerationControlValue(ezC2X::cdd::AccelerationControl ac);

    size_t fingerprint(ReadableCam rc);

    std::string concatenateValues();

private:
    ReadableCam cam_;
    MetaData metaData_;
};
#endif //REPORT_HPP