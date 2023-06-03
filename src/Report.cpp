#include <Report.hpp>
#include <sstream>
#include <iomanip>
#include "ezC2X/core/geographic/LocalCartesianTransform.hpp"

Report::Report() {}

Report::~Report(){}

Report::Report(ezC2X::Cam cam, MetaData meta){
    //Moves relevant info from cam to readablecam
    auto hfb = cam.payload().containers().high_frequency_container().basic_vehicle_container_high_frequency();
    auto refPos = cam.payload().containers().basic_container().reference_position();
    auto wgsPos = ezC2X::Wgs84Position::wrap(refPos.latitude().value(), refPos.longitude().value());
    auto origin = ezC2X::Wgs84Position::wrap(ORIGIN_LAT,ORIGIN_LONG);
    ezC2X::LocalCartesianTransform transformer(origin);
    auto vCoords = transformer.toCartesian(wgsPos);
    cam_.id = cam.header().station_id(); //this is not the sumo id
    cam_.pos.wgsPos = wgsPos;
    cam_.pos.cartPos = vCoords;
    cam_.speed = hfb.speed().value().value();
    cam_.heading = hfb.heading().value().value();
    cam_.driveDirection = hfb.drive_direction().value();
    cam_.generationDeltaTime = cam.payload().generation_time(); //TODO fix time
    cam_.longitudinalAcceleration = hfb.longitudinal_acceleration().value().value(); //Note: converts double to int
    cam_.curvature = hfb.curvature().value().value();
    cam_.curvatureCalculationMode = hfb.curvature_calculation_mode().value();
    cam_.yawRate = hfb.yaw_rate().value().value();

    //The following parameters are optional
    if(hfb.has_acceleration_control()){cam_.accelerationControl = accelerationControlValue(hfb.acceleration_control());}
    if(hfb.has_lane_position()){cam_.lanePosition = hfb.lane_position().value();}
    if(hfb.has_steering_wheel_angle()){cam_.lanePosition = hfb.steering_wheel_angle().value().value();}
    if(hfb.has_lateral_acceleration()){cam_.lanePosition = hfb.lateral_acceleration().value().value();}
    if(hfb.has_vertical_acceleration()){cam_.lanePosition = hfb.vertical_acceleration().value().value();}

    //todo: not sure if this object always exists, so need to check
    if(cam.payload().containers().special_vehicle_container().safety_car_container().has_light_bar_siren_in_use()){
        cam_.attacking = cam.payload().containers().special_vehicle_container().safety_car_container().light_bar_siren_in_use().light_bar_activated();
    }
    cam_.fingerprint = fingerprint(cam_);
    //Save metadata
    metaData_ = meta;
}

uint8_t
Report::accelerationControlValue(ezC2X::cdd::AccelerationControl ac){
    int val = 0;
    if(ac.speed_limiter_engaged()){val++;}
    val = val << 1;
    if(ac.cruise_control_engaged()){val++;}
    val = val << 1;
    if(ac.acc_engaged()){val++;}
    val = val << 1;
    if(ac.collision_warning_engaged()){val++;}
    val = val << 1;
    if(ac.speed_limiter_engaged()){val++;}
    val = val << 1;
    if(ac.emergency_brake_engaged()){val++;}
    val = val << 1;
    if(ac.gas_pedal_engaged()){val++;}
    val = val << 1;
    if(ac.brake_pedal_engaged()){val++;}

    return (uint8_t)val; //Should always be a positive int
}

// Define the hash function for the struct
size_t
Report::fingerprint (ReadableCam rc){
    std::string hash_val = "";
    hash_val += std::to_string(rc.pos.wgsPos.getLatitude().value()) + ";";
    hash_val += std::to_string(rc.pos.wgsPos.getLongitude().value()) + ";";
    hash_val += std::to_string(rc.speed) + ";";
    hash_val += rc.id + ";";
    hash_val += std::to_string(rc.generationDeltaTime) + ";";
    return std::hash<std::string>()(hash_val);
}

std::string
Report::concatenateValues() {
    std::stringstream ss;

    ss << cam_.id << ",";
    ss << std::fixed << std::setprecision(8);
    ss << (cam_.pos.wgsPos.getLongitude().value()) << "," << (cam_.pos.wgsPos.getLatitude().value()) << ",";
    ss << (cam_.pos.cartPos.x) << "," << (cam_.pos.cartPos.y) << ",";
    ss << std::defaultfloat;
    ss << cam_.speed << ",";
    ss << cam_.heading << ",";
    ss << static_cast<int>(cam_.driveDirection) << ",";
    ss << cam_.generationDeltaTime << ",";
    ss << cam_.longitudinalAcceleration << ",";
    ss << cam_.curvature << ",";
    ss << cam_.curvatureCalculationMode << ",";
    ss << cam_.yawRate << ",";
    ss << static_cast<int>(cam_.accelerationControl) << ",";
    ss << cam_.lanePosition << ",";
    ss << cam_.steeringWheelAngle << ",";
    ss << cam_.lateralAcceleration << ",";
    ss << cam_.verticalAcceleration << ",";
    ss << metaData_.timeOnReceive << ",";
    ss << std::fixed << std::setprecision(8);
    ss << (metaData_.posOnReceieve.wgsPos.getLongitude().value()) << "," << (metaData_.posOnReceieve.wgsPos.getLatitude().value()) << ",";
    ss << (metaData_.posOnReceieve.cartPos.x) << "," << (metaData_.posOnReceieve.cartPos.y) << ",";
    ss << metaData_.id << ",";
    ss << cam_.attacking << ",";
    ss << cam_.fingerprint;
    return ss.str();
}


ReadableCam 
Report::getCam(){
    return cam_;
}

MetaData
Report::getMetaData(){
    return metaData_;
}

