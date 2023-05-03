#include "ezC2X/facility/cam/Cam.pb.h"


struct ReadableCam
{
    /*
    Many of these values are stored as a value/confidence pair in the actual CAM.
    In order to simplify things we only use values, not the confidence.
    */

    uint32_t id;
    std::tuple<double,double> pos;
    double speed;
    double heading;
    uint8_t driveDirection;
    uint16_t generationDeltaTime;   //generationDeltaTime = TimestampIts mod 65 536. 
                                    //TimestampIts represents an integer value in milliseconds since
                                    // 2004-01-01T00:00:00:000Z as defined in ETSI TS 102 894-2.
    int longitudinalAcceleration;
    int curvature;
    int curvatureCalculationMode;
    int yawRate;
    //The following parameters are optional in CA messages
    uint8_t accelerationControl;
    int lanePosition;
    int steeringWheelAngle;
    int lateralAcceleration;
    int verticalAcceleration;
    size_t fingerprint;
};

struct MetaData
{
    uint16_t timeOnReceive;
    std::tuple<double,double> positionOnReceieve;
    std::string id;

};

class Report{
public:
    Report();

    ~Report();

    Report(ezC2X::Cam cam, MetaData meta);

    ReadableCam getCam();

    MetaData getMetaData();

    void fingerprint(ReadableCam rc);

private:
    ReadableCam cam_;
    MetaData metaData_;
};
