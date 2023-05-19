#include "idsse.hpp"

#include <ctime>
#include <chrono>
#include <cmath>
#include <math.h>
#include <iostream>
#include <vector>
#include <time.h>
#include <boost/tokenizer.hpp>
#include <boost/cast.hpp>
#include <boost/range/algorithm/count.hpp>
//#include <boost/python.hpp>

#include <ezC2X/core/component/Aggregatable.hpp>
#include <ezC2X/core/component/Bundle.hpp>
#include <ezC2X/core/component/RunUtil.hpp>
#include <ezC2X/core/property/Mapper.hpp>
//#include "ezC2X/security/attacker/AttackTypes.hpp"

#include <iostream>
#include <fstream>

#include <ezC2X/facility/cam/CaBasicService.hpp>
#include <ezC2X/facility/denm/DenBasicService.hpp>
#include "ezC2X/facility/denm/DenEventType.hpp"
#include <ezC2X/core/geographic/Distance.hpp>
#include <ezC2X/core/geographic/VehicleCoordinateTransform.hpp>
#include <ezC2X/core/time/ItsClock.hpp>
#include <ezC2X/core/time/ItsTimestamp.hpp>
#include "ezC2X/facility/denm/DenTriggerParameters.hpp"

namespace ezC2X
{

idsse::idsse() : state_(State::NotRunning), log_("idsse"){}

idsse::~idsse(){
    log_.info() << "Shutting down " << vehicleId_;
    if(isReporter_){saveReports();}
    triggerEvent_.cancel();
    rerouteEvent_.cancel();
    speedAdapterEvent_.cancel();
}

std::string 
idsse::getId(){
    log_.info() << "Getting ID";
    if(vehicleId_ == ""){
        //auto vehicleControl_ = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::getId");
        vehicleId_ = (vehicleControl_->getId());
        log_.debug() << "I am ns-3 vehicle: " << vehicleId_;        
    }
    return vehicleId_;
}

void
idsse::configure(boost::property_tree::ptree const& properties)
{
    log_.info() << "idsse is configuring";
    property::Mapper pm;
    pm.addProperty("TriggerStart", &triggerStart_, false);
    pm.addProperty("RerouteDelay", &rerouteDelay_, false);
    pm.addProperty("SpeedAdapterStart", &speedAdapterStart_,false);
    pm.addProperty("SpeedAdapterPeriod", &speedAdapterPeriod_,false);
    log_.info() << "Configuration completed";
}

uint8_t
idsse::isReporter(std::string id){
    return (id.find("reporter") != std::string::npos);

}

uint8_t
idsse::isAttacker(std::string id){
    return (id.find("attacker") != std::string::npos);
}

void
idsse::triggerEvent(){
    log_.info() << "Triggering event!";
    if(isAttacker_){ //will non-attacker even reach this?
        isAttacking_ = true;
        switch (attackType_){
            case spoofing:
                caService_->spoof();
                break;
            default:
                break;
        }
    }
}

uint64_t 
idsse::getCurrentCertificate(){
    auto cm = deps_.getOrThrow<PseudonymManager, component::MissingDependency>("PseudonymManager", "idsse::getCurrentCertificate");
    return cm->getCurrentPseudonymId().value();
}

void
idsse::attackStart(){
    log_.info() << "Running attack start";
    auto es = deps_.getOrThrow<EventScheduler, component::MissingDependency>("EventScheduler", "idsse::attackStart");
    triggerEvent_ = es->schedule([this] () { triggerEvent();}, std::chrono::milliseconds(triggerStart_));
    log_.info() << "Attack start completed";

}

void
idsse::normalStart(){
    log_.info() << "Running normal start";
    //auto vehicleControl_ = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::normalStart");
    auto es = deps_.getOrThrow<EventScheduler, component::MissingDependency>("EventScheduler", "idsse:normalStart");
    rerouteEvent_ = es->schedule([this] () {rerouter();},std::chrono::milliseconds(rerouteDelay_));
    speedAdapterEvent_ = es->schedule([this] () {speedAdapter();},std::chrono::milliseconds(speedAdapterStart_), std::chrono::milliseconds(speedAdapterPeriod_));
    //Schedule event for reroute

    /*ScopedEvent triggerEvent_;
        Settings for all normal vehicles
    */
    //vehicleControl_->setColor(238,255,230,255);
    //vehicleControl_->setSpeed(defaultSpeed); //max speed on the road. '
    // if(getId()== vehicleIdOppositeDir){
    //     vehicleControl_->setRoute(route_otherway);
    //     std::srand(static_cast<unsigned int>(std::time(nullptr)));
    //     double oSpeed = std::rand() % defaultSpeed+1;
    //     log_.info() << "Vehicle " << getId() << ": Setting speed to " << oSpeed <<"m/s (randomly decided)";
    //     vehicleControl_->setSpeed(oSpeed+2);
    // }else{
    //     vehicleControl_->setColor(238,255,230,255);
    //     vehicleControl_->setSpeed(defaultSpeed); //max speed on the road. 
    // }
    log_.info() << "Normal start completed";
}


void
idsse::start(component::Bundle const& framework)
{
    log_.info() << "Application started";
    state_ = State::Running;
    deps_.setFromAggregationIfNotSet(framework);
    auto cm = deps_.getOrThrow<PseudonymManager, component::MissingDependency>("PseudonymManager", "idsse::start");
    auto vehicleControl_ = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::start");
    auto timeProvider_ = deps_.getOrThrow<TimeProvider, component::MissingDependency>("TimeProvider", "idsse::start");
    //Enable CAM
    try
    {
        log_.info() << "Acquiring CaBasicService from framework";
        caService_ = framework.get<CaBasicService>();
        log_.info() << "Enabling CAM subscription";
        camReceptionConnection_ = caService_->subscribeOnCam([this](Cam const& cam) { handleReceivedCam(cam); });


    }
    catch (component::NotFoundInBundle const& e)
    {
        //log_.error() << "Couldn't load PseudonymManager dependency, therefore, no CAM subscription";
        throw(MissingDependency(e.what()));
    }

    //Vehicle type branching
    log_.info() << "Assigning id";
    std::string id = getId();
    //log_.info() << "Determining if vehicle (" << id << ") is attacker based on ID";
    if(isAttacker(id)){
        isAttacker_ = true;
        log_.info() << "Vehicle (" << id << ") is attacker, running attackStart()";
        attackStart();
    }else{
        log_.info() << "Vehicle (" << id << ") is normal, running normalStart()";
        normalStart();
        //log_.info() << "Determining if vehicle (" << id << ") is reporter based on ID";
        if(isReporter(id)){
            log_.info() << "Vehicle (" << id << ") is reporter, setting reporter flag to true";
            isReporter_ = true;
        }
    }
    //Schedule event for speed-adapter
    log_.info() << "Startup completed";
}

void
idsse::handleReceivedCam(Cam const& cam)
{
    log_.info() << "handleReceivedCam";
    auto cm = deps_.getOrThrow<PseudonymManager, component::MissingDependency>("PseudonymManager", "idsse::handleRecievedCam");
    //auto vehicleControl_ = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::handleRecievedCam");

    if(isAttacking_){ //Stop listening to CAMs while actively attacking 
        return;
    }else {
        //Create Report... send in cam and Meta data...
        //Send the Report into the car_ids
        //Send Report to network_ids (ensure this is a central shared network_ids)
        //Send report to route_decider
    }
    if(isReporter_){ //Logging
        //reporter_collection.push_back(report) //This is for collecting a msg_dump per reporter car
        //log_.info() << "Vehicle " << getId() << ":  Received CAM: " << cam.DebugString();
        MetaData meta;
        meta.id = vehicleId_;
        auto lat = vehicleControl_->getCenterPosition().getLatitude().value();
        auto lon = vehicleControl_->getCenterPosition().getLongitude().value();
        std::tuple<double,double> pos = std::tuple<double,double>(lat,lon);
        meta.positionOnReceieve = pos;
        meta.timeOnReceive = makeItsTimestamp(timeProvider_->now()); //Divide by 65536 or no?
        reporter_collection.push_back(Report(cam,meta));
        
    }
    
}


void
idsse::stop() noexcept
{
    log_.info() << "Application stopped";
    triggerEvent_.cancel();
    /*
        It will globally disable CAM and DENM reception, not just within this application. Therefore, non-attack vehicles who stop the "malware" would also automatically disable V2V communication.
    */
    camReceptionConnection_.disconnect();
    denmReceptionConnection_.disconnect();
    state_ = State::NotRunning;
}

idsse::State
idsse::state() const
{
    return state_;
}

void idsse::saveReports (){
    std::ofstream myfile;
    std::string filename = "car_dump_" + std::to_string(std::chrono::system_clock::to_time_t((std::chrono::system_clock::now())));
    myfile.open(filename);
    for(auto report: reporter_collection) {
        myfile << (report.concatenateValues() + "\n");
    }
    myfile.close();
}


void idsse::speedAdapter(){
    log_.info() << "Running speed adapter";
    //This event should be scheduled like every second...
    //Variables is just fetching current timestamp, car x pos, and car y pos
    //auto vehicleControl_ = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::speedAdapter");
    auto lat = vehicleControl_->getCenterPosition().getLatitude().value();
    auto lon = vehicleControl_->getCenterPosition().getLongitude().value();
    std::tuple<double,double> pos = std::tuple<double,double>(lat,lon);
    uint64_t time;
    if(caService_->getLatestCam().has_value()){
        time = caService_->getLatestCam().value().payload().generation_time();
    }else{
        time = 0;
    }
    vehicleControl_->setSpeed(routeDecider.new_speed(std::get<0>(pos), std::get<1>(pos), routeDecider.MAX_SPEED, time));
    log_.info() << "SA: finished";
}

void idsse::rerouter(){
    log_.info() << "Running rerouter";
    //auto vehicleControl_ = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::rerouter");
    if (!routeDecider.continue_on_main(routeDecider.MAX_SPEED, routeDecider.MAX_SPEED)) {
        vehicleControl_->setRoute(side_route);//is it really accessing the const
    }
}

} // namespace ezC2X
