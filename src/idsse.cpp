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
#include "ezC2X/facility/denm/DenTriggerParameters.hpp"

namespace ezC2X
{

idsse::idsse() : state_(State::NotRunning), log_("idsse"){}

idsse::~idsse(){
    triggerEvent_.cancel();
}

std::string 
idsse::getId(){
    if(vehicleId_ == ""){
        auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse");
        log_.debug() << "I am ns-3 vehicle: " << vehicleControl->getId();
        vehicleId_ = (vehicleControl->getId());
    }
    return vehicleId_;
}

void
idsse::configure(boost::property_tree::ptree const& properties)
{
    property::Mapper pm;
    pm.addProperty("TriggerStart", &triggerStart_, false);
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
    auto cm = deps_.getOrThrow<CertificateManager, component::MissingDependency>("CertificateManager", "idsse");
    return cm->getCurrentPseudonymId().value();
}

void
idsse::attackStart(){
    auto es = deps_.getOrThrow<EventScheduler, component::MissingDependency>("EventScheduler", "idsse");
    triggerEvent_ = es->schedule([this] () { triggerEvent();}, std::chrono::milliseconds(triggerStart_));

}

void
idsse::normalStart(){
    auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse");

    /*
        Settings for all normal vehicles
    */
    //vehicleControl->setColor(238,255,230,255);
    vehicleControl->setSpeed(defaultSpeed); //max speed on the road. 
    // if(getId()== vehicleIdOppositeDir){
    //     vehicleControl->setRoute(route_otherway);
    //     std::srand(static_cast<unsigned int>(std::time(nullptr)));
    //     double oSpeed = std::rand() % defaultSpeed+1;
    //     log_.info() << "Vehicle " << getId() << ": Setting speed to " << oSpeed <<"m/s (randomly decided)";
    //     vehicleControl->setSpeed(oSpeed+2);
    // }else{
    //     vehicleControl->setColor(238,255,230,255);
    //     vehicleControl->setSpeed(defaultSpeed); //max speed on the road. 
    // }
}


void
idsse::start(component::Bundle const& framework)
{
    log_.info() << "Application started";
    state_ = State::Running;
    auto cm = deps_.getOrThrow<CertificateManager, component::MissingDependency>("CertificateManager", "idsse");
    auto vehicleControl = deps_.getOrThrow<VehicleControlInterface, component::MissingDependency>("VehicleControlInterface", "idsse");

    //Enable CAM
    try
    {
        log_.info() << "Enabling CAM subscription";
        caService_ = framework.get<IdsseCaBasicService>();
        camReceptionConnection_ = caService_->subscribeOnCam([this](Cam const& cam) { handleReceivedCam(cam); });

    }
    catch (component::NotFoundInBundle const& e)
    {
        log_.error() << "Couldn't load CertificateManager dependency, therefore, no CAM subscription";
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
}

void
idsse::handleReceivedCam(Cam const& cam)
{
    auto cm = deps_.getOrThrow<CertificateManager, component::MissingDependency>("CertificateManager", "idsse");
    if(isAttacking_){return;} //Stop listening to CAMs while actively attacking 
    if(isReporter_){ //Logging
        //Create Report...
        //reporter_collection.push_back(report)
        log_.info() << "Vehicle " << getId() << ":  Received CAM: " << cam.DebugString();
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

void dump_file (){
    std::ofstream myfile;
    std::string filename = 'car_dump_' + std::to_string(std::chrono::system_clock::to_time_t((std::chrono::system_clock::now())));
    myfile.open(file_name);
    for(auto report: reporter_collection) {
        myfile << (report.concatenateValues() + "\n");
    }
    myfile.close();
}

} // namespace ezC2X
