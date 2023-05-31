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
    if(isReporter_){
        auto timestamp = std::to_string(std::chrono::system_clock::to_time_t((std::chrono::system_clock::now())));
        std::string reporterFile = "report_" + getId() + "_" + timestamp + ".csv";
        std::string misbehaviorFile = "misbehaving_" + getId() + "_" + timestamp + ".csv";
        saveReports(reportCollection_, reporterFile);
        saveReports(cIDS_.getMisbehavedMessages(), misbehaviorFile);
    }
    triggerEvent_.cancel();
    rerouteEvent_.cancel();
    speedAdapterEvent_.cancel();
}

std::string 
idsse::getId(){
    log_.info() << "Getting ID";
    if(vehicleId_ == ""){
        auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::getId");
        vehicleId_ = (vehicleControl->getId());
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
    auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::attackStart");
    auto es = deps_.getOrThrow<EventScheduler, component::MissingDependency>("EventScheduler", "idsse::attackStart");
    triggerEvent_ = es->schedule([this] () { triggerEvent();}, std::chrono::milliseconds(triggerStart_));
    vehicleControl->setSpeed(15.00);
    log_.info() << "Attack start completed";

}

void
idsse::normalStart(){
    log_.info() << "Running normal start";
    auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::normalStart");
    auto es = deps_.getOrThrow<EventScheduler, component::MissingDependency>("EventScheduler", "idsse:normalStart");
    log_.info() << "Scheduling reroute with delay: " << rerouteDelay_;
    //rerouteEvent_ = es->schedule([this] () {rerouter();},std::chrono::milliseconds(rerouteDelay_));
    log_.info() << "Scheduling speedAdapter with delay: " << speedAdapterStart_ << " and period: " << speedAdapterPeriod_;
    speedAdapterEvent_ = es->schedule([this] () {speedAdapter();},std::chrono::milliseconds(speedAdapterStart_), std::chrono::milliseconds(speedAdapterPeriod_));
    log_.info() << "Normal start completed";
    vehicleControl->disableAutomaticSafeDriving();
}


void
idsse::start(component::Bundle const& framework)
{
    log_.info() << "Application started";
    state_ = State::Running;
    deps_.setFromAggregationIfNotSet(framework);
    auto cm = deps_.getOrThrow<PseudonymManager, component::MissingDependency>("PseudonymManager", "idsse::start");
    auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::start");
    //auto timeProvider = deps_.getOrThrow<TimeProvider, component::MissingDependency>("TimeProvider", "idsse::start");
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
    std::string id = getId();
    if(isAttacker(id)){
        isAttacker_ = true;
        attackStart();
    }else{
        normalStart();
        if(isReporter(id)){
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
    auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::handleRecievedCam");

    if(isAttacking_){ //Stop listening to CAMs while actively attacking 
        return;
    }else {
        MetaData meta;
        auto timeProvider = deps_.getOrThrow<TimeProvider, component::MissingDependency>("TimeProvider","idsse::handleReceivedCam");
        meta.id = vehicleId_;
        auto lon = vehicleControl->getCenterPosition().getLongitude().value();
        auto lat = vehicleControl->getCenterPosition().getLatitude().value();
        std::tuple<double,double> pos = std::tuple<double,double>(lon,lat);
        meta.positionOnReceieve = pos;
        meta.timeOnReceive = makeItsTimestamp(timeProvider->now()); //modolu 65536 or no?  Cam doesn't seem to have it so hold off for now
        
        //Send report to routeDecider
        auto report = Report(cam,meta);
        routeDecider_.collectLatest(report);

        //Send the Report into the carIDs
        cIDS_.carIDS(report);

        if(isReporter_){ //Logging
            reportCollection_.push_back(report);
        }
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

void idsse::saveReports (std::vector<Report> reports, std::string filename){
    std::ofstream myfile;
    myfile.open(filename);
    myfile << "sendId,xpos,ypos,speed,heading,driveDir,genDeltaTime,longAcc,curvature,curvCalcMode,yawRate,accControl,lanePos,steeringWheelAngle,latAcc,vertAcc,receiveTime,receiveXPos,receiveYPos,myID,attacking\n";
    for(auto report: reports) {
        myfile << (report.concatenateValues() + "\n");
    }
    myfile.close();
}


void idsse::speedAdapter(){
    auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::speedAdapter");
    auto lon = vehicleControl->getCenterPosition().getLongitude().value();
    auto lat = vehicleControl->getCenterPosition().getLatitude().value();
    std::tuple<double,double> pos = std::tuple<double,double>(lon,lat);
    uint64_t time;
    if(caService_->getLatestCam().has_value()){
        time = caService_->getLatestCam().value().payload().generation_time();
    }else{
        time = 0;
    }
    auto newSpeed = routeDecider_.newSpeed(std::get<0>(pos), std::get<1>(pos), routeDecider_.MAX_SPEED, time);
    log_.info() << "SA: " << getId() << "Setting new speed to " << newSpeed;
    vehicleControl->setSpeed(newSpeed);
    log_.info() << "SA: finished";
}

void idsse::rerouter(){
    auto timeProvider = deps_.getOrThrow<TimeProvider, component::MissingDependency>("TimeProvider","idsse::rerouter");
    log_.info() << "Running rerouter at t:" << makeItsTimestamp(timeProvider->now());
    auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse::rerouter");
    if (!routeDecider_.continueOnMain(routeDecider_.MAX_SPEED, routeDecider_.MAX_SPEED)) {
        log_.info() << "Attempting to set new route";
        vehicleControl->setRoute(sideRoute_);
    }
}

} // namespace ezC2X
