/*
 * Copyright (c) [2019] Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * acting on behalf of its Fraunhofer Institute for Cognitive Systems IKS. All rights reserved.
 *
 * This Software is licensed under the terms and conditions of the MIT License.
 * See LICENSE file in the project root for details.
 */

/*!
 * @brief Implementation of the CA Basic Service according to ETSI EN 302 637-2 V1.4.1 (2019-04)
 * @author Yagmur Sevilmis
 * @author Thomas Rosenstatter, adapted to accomodate cyber attacks
 * @file IdsseCaBasicService.hpp
 */
#ifndef EZC2X_FACILITY_CAM_ETSI_CA_BASIC_SERVICE_HPP
#define EZC2X_FACILITY_CAM_ETSI_CA_BASIC_SERVICE_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <boost/optional.hpp>
#include <boost/signals2/connection.hpp>

#include "ezC2X/cdd/types/StationType.pb.h"

#include "ezC2X/core/component/Aggregatable.hpp"
#include "ezC2X/core/component/Configurable.hpp"
#include "ezC2X/core/component/Handle.hpp"
#include "ezC2X/core/component/Runnable.hpp"
#include "ezC2X/core/datahub/DataHub.hpp"
#include "ezC2X/core/event/EventScheduler.hpp"
#include "ezC2X/core/event/ScopedEvent.hpp"
#include "ezC2X/core/geographic/Wgs84Position.hpp"
#include "ezC2X/core/logging/Logger.hpp"
#include "ezC2X/core/position/EgoPositionProvider.hpp"
#include "ezC2X/core/position/PositionVector.hpp"
#include "ezC2X/core/time/TimePoint.hpp"
#include "ezC2X/core/time/TimeProvider.hpp"

#include "ezC2X/facility/cam/CaBasicService.hpp"
#include "ezC2X/facility/cam/CaBasicServiceManagement.hpp"
#include "ezC2X/facility/cam/Cam.pb.h"
#include "ezC2X/facility/cam/etsi/EtsiCaBasicServicePaths.hpp"
#include "ezC2X/facility/cam/etsi/EtsiProtocolParameters.hpp"
#include "ezC2X/facility/cam/tracing/EventTracer.hpp"
#include "ezC2X/facility/data_presentation/DataPresentation.hpp"

#include "ezC2X/network/btp/Btp.hpp"

#include "ezC2X/security/pki/CertificateManager.hpp"
#include "ezC2X/security/attacker/AttackTypes.hpp"

#include <bits/stdc++.h>
#include <boost/algorithm/string.hpp>


namespace ezC2X
{
/*!
 * @brief Implementation of the CA Basic Service according to ETSI EN 302 637-2 V1.4.1 (2019-04)
 *
 * Missing functionalities according to the latest CAM standard are listed below for reference:
 * -Does not support special vehicle containers
 * -Security constraints (clause 6.2.2)
 *
 * @ingroup cam
 */
class IdsseCaBasicService : public CaBasicService,
                           public CaBasicServiceManagement,
                           public component::Aggregatable,
                           public component::Configurable,
                           public component::Runnable,
                           public cam::tracing::EventTracer
{
public:
    //! External dependencies of this component
    using Dependencies =
        component::Handle<Btp, ezC2X::component::Optional<DataHub>, DataPresentation,
                          ezC2X::component::Optional<EventScheduler>, ezC2X::component::Optional<TimeProvider>,
                          ezC2X::component::Optional<CertificateManager>>;

    //! Create instance with default parameters
    IdsseCaBasicService();

    /*!
     * @brief Set a specific dependency
     * @param d     Pointer to the dependency instance to use
     * @tparam T    Type of the dependency (must match exactly)
     */
    template<typename T>
    void
    setDependency(std::weak_ptr<T> d)
    {
        deps_.set<T>(d);
    }

    /*
     * CaBasicService API
     */

    boost::signals2::connection
    subscribeOnCam(SignalOnCam::slot_type const& handler) override;
    /*
     * CaBasicServiceManagement API
     */

    void
    setTGenCamDcc(std::chrono::milliseconds tGenCamDcc) override;

    /*
     * Configurable API
     */

    void
    configure(boost::property_tree::ptree const& properties) override;

    /*
     * Runnable API
     */

    void
    run(Stage stage) override;

    void
    shutdown() noexcept override;

    /*!
     * @brief Get the currently configured paths (i.e. DataHub paths used for populating CAM fields)
     * @return Current path configuration
     */
    cam::Paths
    getPaths() const;

    /*!
     * @brief Set the DataHub paths to use for populating CAM fields
     * @param paths     Paths to set
     */
    void
    setPaths(cam::Paths const& paths);


    /*!
     * @brief Get the current station ID the CamService is using;
     * @param paths     Paths to set
     */
    std::uint32_t 
    getStationID();

    void setAttackType(int) override;
    bool isSuppressCAMs() override;
    // uint8_t getAttackType() override;    
    void setAttack1(std::vector<double> a1IrregularSpeedProfile) override;
    void setAttack2(std::uint32_t a2PositionOffset) override;
    double getAttack1(int _attackStep) override;
    void triggerAttack() override;
    void setAttackActive(bool active) override;
    bool isAttackActive() override;
    int getAttackStep() override;
    void setSuppressCAMs(bool _suppressCams) override;

    // uint8_t getAttackStep() override;

    /*!
    * @brief Returns a pointer to the service's position provider.
    *
    */
    boost::optional<ezC2X::PositionVector>
    getPositionVector();

    /*!
    * @brief Returns the latest sent cam
    */
    boost::optional<Cam>
    getLatestCam();

    //! Last heading value
    boost::optional<double> getlastHeading();

    //! Last position data
    boost::optional<Wgs84Position> getlastPosition();

    //! Last speed value
    boost::optional<double> getlastSpeed();


    std::chrono::milliseconds getTimeSinceLastCam();

private:
    /*!
     * @brief Get the current tGenCamDcc value (set by the management entity)
     * @return current tGenCamDcc value
     */
    std::chrono::milliseconds
    tGenCamDcc() const;

    /*!
     * @brief Handler for the packets received by the BTP layer
     *
     * Functionality:
     * - Decode received CAM
     * - Make CAM data available by e.g. passing it to the ITS application layer or to the LDM
     * - End of operation, wait for the next CAM reception
     *
     * @param destinationPort Destination port of the received packet
     * @param sourcePort  Source port of the received packet (if available)
     * @param destinationInfo Destination info of the received packet (if available)
     * @param geonetParams GeoNetworking parameters associated with the received packet
     * @param data Payload of the received packet (should contain the encoded CAM)
     */
    void
    handleBtpPacket(std::uint16_t destinationPort, boost::optional<std::uint16_t> sourcePort,
                    boost::optional<std::uint16_t> destinationInfo, geonet::DataIndication const& gnParams,
                    Packet const& data);

    /*!
     * @brief Starts the CAM generation process
     *
     * Functionality:
     * - If the property "manualCamGenerationInterval" is set, generateCamStaticMode is scheduled repeatedly every
     * "manualCamGenerationInterval" milliseconds i.e. a CAM will be generated every manualCamGenerationInterval
     * milliseconds
     * - If the property "manualCamGenerationInterval" is not set, generateCamStandardMode is scheduled repeatedly every
     * "tCheckCamGen" milliseconds i.e. CAM trigger conditions will be checked and standard CAM generation
     * rules will apply. It should be noted that RSU CAMs will be generated with a static frequency of 1Hz.
     *
     * @throws component::MissingDependency if there is no event scheduler.
     */
    void
    start();

    /*!
     * @brief Increments the bookkeeping counters and creates/sends a CAM
     * @note This function does not check the CAM trigger conditions (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.3)
     * specified in the standard
     */
    void
    generateStaticMode();

    /*!
     * @brief Increments the bookkeeping counters, checks CAM trigger conditions (ETSI EN 302 637-2 V1.4.1 -
     * chapter 6.1.3) and creates/sends a CAM if the conditions are satisfied
     * @note This function is also responsible for CAM frequency management bookkeeping specified in ETSI EN 302 637-2
     * V1.4.1 - chapter 6.1.3
     */
    void
    generateStandardMode();

    /*!
     * @brief Sends a CAM by passing it to the BTP layer
     *
     * Errors are logged internally, therefore it is safe to ignore the return value unless
     * further action is required.
     *
     * Functionality:
     * - Encodes the CAM
     * - Sets GeoNetworking request parameters
     * - Hands the CAM to the BTP layer
     *
     * @param cam Message to be sent
     * @return true if successful, false otherwise e.g null message, encoding error, missing dependency etc.
     */
    bool
    send(boost::optional<Cam> cam);

    //! Interval between two subsequent invocations of CAM generation functions (i.e. generateStaticMode or
    //! generateStandardMode) is called one check interval
    using CheckIntervalType = std::uint32_t;

    /*!
     * @brief Calculates a duration from a number of check intervals
     * @param checkIntervals Number of check intervals
     * @return Duration corresponding to the number of check intervals
     */
    std::chrono::milliseconds
    checkIntervalsToDuration(CheckIntervalType checkIntervals) const;

    /*!
     * @brief Checks whether a low frequency container should be generated or not
     * @param checkIntervalsSinceLastLfContainer Number of check intervals since the last low frequency container
     * @return True if a low frequency container should be generated, false otherwise
     */
    bool
    shouldGenerateLowFrequencyContainer(CheckIntervalType checkIntervalsSinceLastLfContainer) const;

    /*!
     * @brief Increments the counters that keep track of the number of check intervals elapsed since the last CAM and
     * low frequency container generation
     */
    void
    incrementCheckIntervals();

    /*!
     * @brief Checks whether the heading change since last CAM generation exceeds the specified threshold or not
     *
     * @param now Current heading
     * @param prev Heading included in the CAM previously transmitted
     *
     * @return True if the absolute difference between the current and the previous heading exceeds
     * the threshold specified, false otherwise
     */
    bool
    checkHeadingDelta(double now, double prev) const;

    /*!
     * @brief Checks whether the position change since last CAM generation exceeds the specified threshold or not
     *
     * @param now Current position
     * @param prev Position included in the CAM previously transmitted
     *
     * @return True if the distance between the current and the previous position exceeds
     * the threshold specified, false otherwise
     */
    bool
    checkPositionDelta(Wgs84Position const& now, Wgs84Position const& prev) const;

    /*!
     * @brief Checks whether the speed change since last CAM generation exceeds the specified threshold or not
     *
     * @param now Current speed
     * @param prev Speed included in the CAM previously transmitted
     *
     * @return True if the absolute difference between the current and the previous speed exceeds
     * the threshold specified, false otherwise
     */
    bool
    checkSpeedDelta(double now, double prev) const;

    /*!
     * @brief Constructs a CAM message
     *
     * Functionality:
     * - Adds Its Pdu header, generation time, basic container, high frequency container (vehicle or rsu depending on
     * the station type) as they are mandatory
     * - Adds low frequency container for vehicle stations if it should be included (it is always included in the first
     * CAM)
     *
     * @note Generation time is set as follows: Time when the position information was captured if the station is of
     * type vehicle, message generation time if the station is of type rsu (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.5.2)
     *
     * @return CAM if successful, null object if CAM creation failed due to position being unavailable, missing
     * dependencies or errors while message population e.g. accessing the DataHub
     */
    boost::optional<Cam>
    cam();

    /*!
     * @brief Populates the fields of a CAM basic container and records the position that will be used when
     * checking CAM trigger conditions in the next generation cycle
     *
     * @param cam Message for which the basic container should be populated
     * @param posData Reference to the position related data (for setting latitude, longitude, altitude, altitude
     * confidence and position confidence)
     */
    void
    addBasicContainer(Cam& cam, PositionVector const& posData);

    /*!
     * @brief Populates the fields of a CAM vehicle high frequency container and records the speed, heading
     * values that will be used when checking CAM trigger conditions in the next generation cycle
     *
     * @param cam Message for which the high frequency container should be populated
     * @param posData Reference to the position related data (for setting heading, speed)
     * @param hub Reference to the DataHub used for setting other container data elements
     * @throws datahub::ObjectInvalid if there is an error accessing the DataHub
     */
    void
    addHighFrequencyContainer(Cam& cam, PositionVector const& posData, ezC2X::DataHub const& hub);

    /*!
     * @brief Populates the fields of a CAM vehicle low frequency container
     *
     * @param cam Message for which the low frequency container should be populated
     * @param hub Reference to the DataHub used for setting container data elements (i.e. vehicle role, exterior lights)
     * @throws datahub::ObjectInvalid if there is an error accessing the DataHub
     */
    void
    addLowFrequencyContainer(Cam& cam, ezC2X::DataHub const& hub) const;

    /*!
     * @brief Calculates the total time required for a CAM generation and logs a warning if it is greater than or equal
     * to 50 ms (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.5.1)
     *
     * @param start Time at which CAM generation was triggered
     * @param end Time at which CAM generation was completed
     */
    void
    reportCamGenerationTime(TimePoint start, TimePoint end);

    //! Handle to the dependencies of this component
    Dependencies deps_;

    //! Logger for this component
    Logger log_;

    //! Object providing position related data
    EgoPositionProvider positionProvider_;

    //! DataHub paths used for populating CAM fields
    cam::Paths paths_;

    //! Configuration of this protocol instance
    cam::EtsiProtocolParameters params_;

    //! Signal to inform higher layers about received CAMs
    SignalOnCam onCam_;

    //! Connection for receiving packets from the BTP layer
    boost::signals2::scoped_connection btpConnection_;

    //! Connection for pseudonym updates
    boost::signals2::scoped_connection pseudonymUpdateConnection_;

    //! Recurring event that checks CAM generation rules
    ScopedEvent checkEvent_;

    /*
     * CAM frequency management parameters
     */

    //! Currently valid upper limit of the CAM generation interval, default value of T_GenCam shall be T_GenCamMax (ETSI
    //! EN 302 637-2 V1.4.1 - chapter 6.1.3)
    std::chrono::milliseconds tGenCam_;

    //! The minimum time interval between two consecutive CAM generations, T_GenCamMin ≤ T_GenCam_DCC ≤ T_GenCamMax
    //! (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.3)
    std::chrono::milliseconds tGenCamDcc_;

    //! Mutex to serialize access to tGenCamDcc_
    mutable std::mutex tGenCamDccMutex_;

    //! Number of CAMs to send before resetting T_GenCam to T_GenCamMax
    std::uint16_t nGenCam_;

    //! Number of check intervals since the generation of the last CAM
    CheckIntervalType checkIntervalsSinceLastCam_;

    //! Number of check intervals since the generation of a CAM with a low frequency container
    CheckIntervalType checkIntervalsSinceLastLfContainer_;

    //! Last heading value
    boost::optional<double> lastHeading_;

    //! Last position data
    boost::optional<Wgs84Position> lastPosition_;

    //! Last speed value
    boost::optional<double> lastSpeed_;

    //! Type of the ITS station (assumed to be static throughout operation)
    cdd::StationType_Value stationType_;

    //! Station ID (derived from the pseudonym ID and gets updated with pseudonym updates)
    std::atomic<std::uint32_t> stationId_;

    //! Type of attack which is executed when triggered
    int attackType = 0;
    //! list of the speed profile used in attack 1 when spoofing the speed
    std::vector<double> a1IrregularSpeedProfile;   
    //! maximum speed difference when randomly spoofing the attacker's speed
    std::uint32_t a2MaxRandomSpeed = 0;
    //! indicate whether the attack is currently executed
    bool attackActive = false;
    //! Attack progress, e.g., when used for speed spoofing, a different speed is spoofed at each step
    int attackStep = 0;
    //! Indicator whether CAM messages are suppressed/not being sent. Needed for Sybil attacks where the "virtual" vehicles are implemented as actual NS-3/SUMO vehicles.
    bool suppressCAMs = false;

    boost::optional<Cam> latestCam_;
};

}  // namespace ezC2X

#endif  // EZC2X_FACILITY_CAM_ETSI_CA_BASIC_SERVICE_HPP
