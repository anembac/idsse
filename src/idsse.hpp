/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/* 
 * Based on the ezCar2x plugin template.
 * Copyright (c) [2022] Thomas Rosenstatter, RISE Research Institutes of Sweden, All rights reserved.
 * Author: Thomas Rosenstatter
 */

/*!
 * @brief Attacker application class
 * @file idsse.hpp
 */
#ifndef EZC2X_IDSSE_APPLICATION_HPP
#define EZC2X_IDSSE_APPLICATION_HPP
#include <cstdint>
#include <vector>
#include <boost/circular_buffer.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>

#include <ezC2X/core/component/Aggregatable.hpp>
#include <ezC2X/core/component/Handle.hpp>
#include <ezC2X/core/component/Runnable.hpp>
#include <ezC2X/core/component/Configurable.hpp>
#include <ezC2X/core/event/EventScheduler.hpp>
#include <ezC2X/core/event/ScopedEvent.hpp>
#include <ezC2X/core/logging/Logger.hpp>

#include <ezC2X/facility/vehicle_control/VehicleControlInterface.hpp>
#include "ezC2X/security/attacker/AttackTypes.hpp"
#include <ezC2X/framework/Application.hpp>
#include <ezC2X/facility/cam/Cam.pb.h>
#include <ezC2X/facility/cam/CaBasicService.hpp>
//#include <ezC2X/facility/cam/IdsseCaBasicService.hpp>

#include <ezC2X/security/pseudonym/PseudonymManager.hpp>
//#include <ezC2X/security/Certificate/PseudonymManager.hpp>
#include <ezC2X/facility/denm/Denm.pb.h>
#include "ezC2X/facility/denm/DenEventType.hpp"
#include <ezC2X/facility/denm/EtsiDenBasicService.hpp>

#include <ezC2X/core/time/TimeProvider.hpp>

#include <ezC2X/network/btp/Btp.hpp>
#include <ezC2X/facility/data_presentation/DataPresentation.hpp>
#include <map>

#include <bits/stdc++.h>
#include <boost/algorithm/string.hpp>

#include <Report.hpp>
#include <RouteDecider.hpp>
#include <CarIDS.hpp>
#include <EgoPos.hpp>

namespace ezC2X
{
/*!
 * @brief The application class
 */
class idsse : public Application, public component::Configurable
{
public:
    //! Dependencies of the application
    using Dependencies = component::Handle<Btp, EventScheduler, PseudonymManager, VehicleControlInterface, TimeProvider, DataPresentation>;
    // using Dependencies = component::Handle<Btp, EventScheduler, PseudonymManager, VehicleControlInterface, TimeProvider, DataPresentation, component::Optional<ns3::EzC2XNodeAccessor>>;

    //! Create default instance
    idsse();

    //! Define destructor
    ~idsse();

    /*!
     * @brief Returns the vehicle identifier according to SUMO. Note that this does not need to correspond with the station id/Certificate.
     */
    std::string
    getId();

    /*
     * Configurable API
     */

    /*!
     * @brief Configure the maneuver application
     *
     * @param properties    Properties with which this component should be configured
     *
     * @throws PropertyError        If parsing the properties fails
     * @throws std::runtime_error   If something else happens that the configuration cannot recover from
     */
    void
    configure(boost::property_tree::ptree const& properties) override;

    /*
     * Application API
     */

    /*!
     * @brief Start the maneuver application with the provided component bundle.
     *
     * When the application starts it schedules a function to trigger a pre-configured maneuver periodically.
     * Also this is where application layer handlers which will be called by the maneuver service in response to certain
     * events e.g. reception of a request message, are registered with the maneuver service.
     *
     * @param framework    Bundle of components provided as potential dependencies to the application
     * @throws StartupError or subclasses in case of any issues during startup
     */
    void
    start(component::Bundle const& framework) override;

    /*!
     * @brief Stop the maneuver application.
     *
     * @note The application must not throw any exception in this method.
     */
    void
    stop() noexcept override;

    /*!
     * @brief Get the current state of the maneuver application.
     * @return State of the application
     */
    State
    state() const override;

private:

    /*!attackStep
     * @brief Executes periodically and triggers a pre-configured maneuver.
     * @param remoteID   ID of the remote vehicle for this maneuver
     * @note TriggerStation configuration parameter controls which station triggers the maneuver
     */
    void
    triggerEvent();


    /*!
     * @brief Resets the vehicle's default speed after the DENM warning message times out. Current implementation, drive 20% slower than the default speed since the vehicles are still driving more carefully
     * @param no parameters
    */
    void
    denmResetEvent();

    /*!
     * @brief Handler for receiving DENM messages from other vehicles.
     * @param DenEventType  event type
     * @param ActionId  the action id of the corresponding denm message
     * @param denm	  the received DENM message
     */
    
    void
    handleReceivedDenm(ezC2X::DenEventType eventtype, cdd::ActionId actionid, Denm const *denm);
    
    /*!
     * @brief Handler for receiving CAM messages from other vehicles. Further runs "handleReceivedCamNormalApp" or "handleReceivedCamReporter" depending on vehicle type (*reporter* or *normal*).
     * @param cam	  the received CAM message
     */
    void
    handleReceivedCam(Cam const& cam);

    /*!
     * @brief Get the Certificate identifier of the vehicle.
    */
    uint64_t 
    getCurrentCertificate();

    /*!
     * @brief compute the bearing or heading of the direct line from position 1 to 2
     */
    double
    getBearing(Wgs84Position position1, Wgs84Position position2);

    double
    getHeadingDiff(double head1, double head2);

    /*!
     * @brief extracts the speed profile from the XML configuration (e.g., -1,-2,-6,-6). The values from the speed profile are used in the speed spoofing attack and will be added to the attacker's actual speed.
     * @param string  speed profile as a comma separated string
     */
    uint8_t
    extractSpeedProfile(std::string);

    /*!
     * @brief extracts attack specific parameters from the XML configuration (e.g., which car is the attacker, setting the maximum random speed and extract the spoofed speed profile)
     */
    void 
    attackAppConfiguration();

    /*!
     * @brief Starting/Configuring the normal vehicles, including setting default speed, define routes and set color.
     * @param component::Bundle framework   the vehicle's framework is needed in order to get access to the VehicleControl Interface.
     */
    void 
    normalStart();

    void 
    attackStart();

    void
    reporterLog(std::string msg);

    /*!
     * @brief Returns true if vehicle is of type *reporter*
     */
    uint8_t
    isReporter(std::string id);

    /*!
     * @brief Returns true if vehicle is of type *attacker*
     */
    uint8_t
    isAttacker(std::string id);

    void saveReports (std::vector<Report> reports, std::string filename);

    void speedAdapter();

    void rerouter();

    std::vector<std::string>
    trimRoute(std::vector<std::string> route, std::string roadID);

    EgoPos getEgoPos();

    // void 
    // addNearbyVehicle(int _id, int _type);

    // int 
    // numberVehicleWarnings(int _type);

    // False means event with dynamically triggered vehicle control
    // True means static predefined maneuver event, periodically executed
    //! Flag for periodic based maneuver trigger
    // bool periodicBasedTrigger_ = true;
    
    //! Seed for generating random number using boost library
    boost::random::mt19937 gen;

    //! State of the application
    State state_;

    //! Logger for this component
    Logger log_;
    std::string camString_;

    //! Dependency handle
    Dependencies deps_;

    //! Pointer to TraCi Vehicle for sending messages
    // ns3::Ptr<ns3::traci::Vehicle> m_vehicle;

    //! Handle for the maneuver trigger event
    ScopedEvent triggerEvent_;
    ScopedEvent rerouteEvent_;
    ScopedEvent speedAdapterEvent_;

    //Handle for reseting the speed after emergency break or similar event has happened.
    ScopedEvent denmResetEvent_;

    //! Connection for receiving CAMs
    boost::signals2::scoped_connection camReceptionConnection_;
    //! Connection for receiving DENMs
    boost::signals2::scoped_connection denmReceptionConnection_;

    enum attackType: int {
        spoofing,
        replay
    };


    // std::list<attack> attackList;
    // std::list<attack> parseAttackInfo(std::string _attackInfo);
    //int isAttacker = 0;

    // Properties configureable through XML file
    //! Time to wait (ms) before the first triggering of the maneuver
    std::uint32_t triggerStart_ = 3000;
    std::uint32_t rerouteDelay_ = 8000;
    std::uint32_t reroutePeriod_ = 200;
    std::uint32_t speedAdapterStart_ = 0;
    std::uint32_t speedAdapterPeriod_ = 100;
    const double stopReroutePos = -320;
    //! Time period (ms) with which triggering of the maneuver recurs
    //todo can we set to infinite/non-recurring??
    std::uint32_t triggerInterval_ = 5;
    //! attacker id
    std::string attackInfo_ = "";
    //std::map <int,int> attackList_;
    std::uint8_t isAttacker_ = false;
    std::uint8_t isAttacking_ = false;
    std::uint8_t isReporter_ = false;
    //const double maxAttackStep_ = 1.05;
    int attackSteps_;
    int attackType_;

    std::string vehicleId_ = "";
    std::shared_ptr<DenBasicService> denService_;
    std::shared_ptr<CaBasicService> caService_;
    // std::shared_ptr<VehicleControlInterface> vehicleControl_;
    // std::shared_ptr<TimeProvider> timeProvider_;

    int defaultSpeed_ = 12;
    //int vehicleIdOppositeDir = 1;
    std::vector<std::string> sideRoute_ = {"R1", "R2", "side1", "side2", "side3", "side4", "side5", "R5", "R6"};
    std::vector<std::string> mainRoute_ = {"R1", "R2", "R3", "R4", "R5", "R6"};

    std::unordered_map<int,int> nearbyVehicles;
    std::vector<Report> reportCollection_;
    RouteDecider* routeDecider_;
    CarIDS cIDS_;
    const bool IDSDisabled_ = false;
    uint64_t startTime_;

};

}  // namespace ezC2X

#endif  // EZC2X_IDSSE_APPLICATION_HPP
