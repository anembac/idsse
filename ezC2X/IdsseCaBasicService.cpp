/*
 * Copyright (c) [2019] Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * acting on behalf of its Fraunhofer Institute for Cognitive Systems IKS. All rights reserved.
 *
 * This Software is licensed under the terms and conditions of the MIT License.
 * See LICENSE file in the project root for details.
 */

#include "IdsseCaBasicService.hpp"

#include <cmath>
#include <limits>
#include <time.h>
#include <boost/cast.hpp>
#include <boost/range/algorithm/count.hpp>

#include "ezC2X/cdd/conversion/AltitudeConfidence.hpp"
#include "ezC2X/cdd/util/StationTypeProperty.hpp"

#include "ezC2X/core/component/RunErrors.hpp"
#include "ezC2X/core/component/RunUtil.hpp"
#include "ezC2X/core/datahub/DataHubErrors.hpp"
#include "ezC2X/core/geographic/Distance.hpp"
#include "ezC2X/core/numeric/StreamAsNumber.hpp"
#include "ezC2X/core/property/Mapper.hpp"
#include "ezC2X/core/time/DurationStreaming.hpp"
#include "ezC2X/core/time/ItsClock.hpp"

#include "ezC2X/network/geonet/common/DataRequest.hpp"
#include "ezC2X/security/profile/CamProfile.hpp"

namespace
{
void
generateDataRequestParameters(ezC2X::geonet::DataRequest& gnParams, ezC2X::Cam const& cam,
                              ezC2X::cam::EtsiProtocolParameters const& params)
{
    gnParams.transportType = params.gnPacketTransportType;
    gnParams.communicationProfile = params.gnCommunicationProfile;
    gnParams.trafficClass = params.gnTrafficClass;
    gnParams.maximumLifetime = params.gnMaximumPacketLifetime;

    // Generate security profile if it is enabled
    if (params.gnSecurityProfile)
    {
        // get the generation time from the CAM
        ezC2X::TimePoint generationTime = ezC2X::ItsClock::to_sys(
            ezC2X::ItsClock::ItsTimePoint(std::chrono::milliseconds(cam.payload().generation_time())));

        // set the profile
        gnParams.securityProfile = std::make_shared<ezC2X::security::CamProfile>(generationTime);
    }
}

std::uint32_t
stationIdFromPseudonym(ezC2X::PseudonymId const& pseudonymId)
{
    auto constexpr byteMask = 0xFFFFFFFFU;
    return pseudonymId.value() & byteMask;
}

}  // namespace

namespace ezC2X
{
IdsseCaBasicService::IdsseCaBasicService()
    : log_("IdsseCaBasicService"),
      positionProvider_(deps_),

      // Initialize CA Basic Service frequency management parameters
      tGenCam_(params_.tGenCamMax),
      tGenCamDcc_(params_.tGenCamMin),
      nGenCam_(params_.nGenCam.value()),
      checkIntervalsSinceLastCam_(std::numeric_limits<CheckIntervalType>::max()),
      checkIntervalsSinceLastLfContainer_(std::numeric_limits<CheckIntervalType>::max()),
      stationType_(cdd::StationType_Value_UNKNOWN),
      stationId_(0)
      {
        // CaBasicService::log_("CaBasicService");
      }

/*
 * CaBasicService API
 */

boost::signals2::connection
IdsseCaBasicService::subscribeOnCam(SignalOnCam::slot_type const& handler)
{
    return onCam_.connect(handler);
}

/*
 * CaBasicServiceManagement API
 */

void
IdsseCaBasicService::setTGenCamDcc(std::chrono::milliseconds tGenCamDcc)
{
    auto newGenCamDccValue = tGenCamDcc;

    // T_GenCamMin ≤ T_GenCam_DCC ≤ T_GenCamMax (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.3)
    if (tGenCamDcc < params_.tGenCamMin)
    {
        newGenCamDccValue = params_.tGenCamMin;
    }
    else if (tGenCamDcc > params_.tGenCamMax)
    {
        newGenCamDccValue = params_.tGenCamMax;
    }

    // mutex is necessary as the the management entity can set this value at random
    // times thus it can write to tGenCamDcc_ at the same time the IdsseCaBasicService is reading from it
    std::lock_guard<std::mutex> l(tGenCamDccMutex_);
    tGenCamDcc_ = newGenCamDccValue;
}

/*
 * Configurable API
 */

void
IdsseCaBasicService::configure(boost::property_tree::ptree const& properties)
{
    property::Mapper pm;

    // the functionality of the EtsiCaService depends on the station type which can be set using this property
    pm.addProperty("StationType", &stationType_, false);

    // map protocol parameters to sub node
    auto paramPm = cam::mapProperties(params_);
    pm.addMapper("ProtocolParameters", paramPm);
    // TODO: add separate attack parameters mapper
    // pm.addMapper("AttackParameters", )
    // mapper that will hold sub-mappers for each path group
    property::Mapper pathsPm;

    // add mappers for each path group to pathsPm
    auto pathMappers = cam::getPathMappers(paths_);
    for (auto& mapper : pathMappers)
    {
        pathsPm.addMapper(mapper.first, mapper.second);
    }

    // map paths to sub node
    pm.addMapper("Paths", pathsPm);

    pm.apply(properties);
    log_.info() << pm;

    // adjust the run-time frequency management parameters according to the configuration values

    // tGenCamMax_ might have changed so set tGenCam_ accordingly as the default value of
    // tGenCam_ should be equal to tGenCamMax_
    tGenCam_ = params_.tGenCamMax;

    // tGenCamMin_ might have changed so set tGenCamDcc_ accordingly as the default value of
    // tGenCamDcc_ should be equal to tGenCamMin_
    tGenCamDcc_ = params_.tGenCamMin;

    // reference nGenCam value might have changed so set the starting nGenCam_ value accordingly
    nGenCam_ = params_.nGenCam.value();
    attackActive = false;
}


/*
 * Runnable API
 */

void
IdsseCaBasicService::run(Stage stage)
{
    switch (stage)
    {
        case Stage::Init:
            deps_.setFromAggregationIfNotSet(*this);
            if (params_.sendingEnabled)
            {
                // include optional dependencies in the check
                component::throwOnMissingDependencies(deps_, "IdsseCaBasicService", true);

                // position, heading and speed should be available in order to send CAMs
                positionProvider_.enforceFields({EgoPositionProvider::DataField::Position,
                                                 EgoPositionProvider::DataField::Heading,
                                                 EgoPositionProvider::DataField::Speed});
            }
            else
            {
                // for the listener mode we only need the mandatory dependencies i.e. Btp and DataPresentation
                component::throwOnMissingDependencies(deps_, "IdsseCaBasicService");
            }

            break;
        case Stage::Normal:
            // subscribe to packets sent to the port that is used for CAMs
            if (auto btp = deps_.get<Btp>())
            {
                btpConnection_ = btp->subscribeOnPacket(
                    [this](std::uint16_t destinationPort, boost::optional<std::uint16_t> sourcePort,
                           boost::optional<std::uint16_t> destinationInfo, geonet::DataIndication const& gnParams,
                           Packet const& data) {
                        handleBtpPacket(destinationPort, sourcePort, destinationInfo, gnParams, data);
                    },
                    params_.destinationPort);
            }
            else
            {
                throw component::MissingDependency("IdsseCaBasicService is missing Btp!");
            }

            if (params_.sendingEnabled)
            {
                positionProvider_.start();

                if (auto pm = deps_.get<CertificateManager>())
                {

                
                    // get the current station id
                    stationId_ = stationIdFromPseudonym(pm->getCurrentPseudonymId());

                    // subscribe for updates
                    /*
                     * Note: The current implementation ignores the two-phase commit and simply switches
                     *       to the new identifier in the stage commit. To ensure the coordinated change of
                     *       pseudonyms across the stack, additional measures have to be implemented
                     */
                    pseudonymUpdateConnection_ = pm->subscribeOnPseudonymChange(
                        [this](security::pseudonym::UpdateStage stage, PseudonymId const& newPseudonymId) {
                            if (stage == security::pseudonym::UpdateStage::Commit)
                            {
                                stationId_ = stationIdFromPseudonym(newPseudonymId);
                            }
                            return true;
                        });
                }
                else
                {
                    throw component::MissingDependency("IdsseCaBasicService is missing CertificateManager!");
                }
            }

            break;
        case Stage::Final:
            // start the CA Basic Service if sending is enabled via configuration
            if (params_.sendingEnabled)
            {
                start();
            }
            break;
        default:
            break;
    }
}

void
IdsseCaBasicService::shutdown() noexcept
{
    positionProvider_.stop();
    btpConnection_.disconnect();
    pseudonymUpdateConnection_.disconnect();
    checkEvent_.cancel();
}

cam::Paths
IdsseCaBasicService::getPaths() const
{
    return paths_;
}

void
IdsseCaBasicService::setPaths(cam::Paths const& paths)
{
    paths_ = paths;
}

uint32_t
IdsseCaBasicService::getStationID(){
    uint32_t _id = stationId_.load(std::memory_order_relaxed);
    return _id;
}


std::chrono::milliseconds
IdsseCaBasicService::tGenCamDcc() const
{
    std::lock_guard<std::mutex> l(tGenCamDccMutex_);
    return tGenCamDcc_;
}

void
IdsseCaBasicService::handleBtpPacket(std::uint16_t destinationPort, boost::optional<std::uint16_t> /*sourcePort*/,
                                    boost::optional<std::uint16_t> /*destinationInfo*/,
                                    geonet::DataIndication const& /*gnParams*/, Packet const& data)
{
    // sanity check
    if (destinationPort != params_.destinationPort)
    {
        log_.warn() << "Received packet with a destination port (" << streamAsNumber(destinationPort)
                    << ") different from the configured BTP port (" << streamAsNumber(params_.destinationPort)
                    << ") - ignoring packet";
        return;
    }

    // trace reception of a CAM
    onCamPacketRx_(data);

    // placeholder for the missing security check (will be added later)

    if (onCam_.empty())
    {
        log_.info() << "No subscriptions for the received CAM. The message will not be decoded!";
        return;
    }

    Cam decodedCam;

    // decode the CAM
    if (auto dp = deps_.get<DataPresentation>())
    {
        try
        {
            decodedCam = dp->decodeCam(data);
        }
        catch (DataPresentation::DecodeFailed const& e)
        {
            log_.error() << e.what();
            return;
        }
    }
    else
    {
        log_.error() << "Data presentation is missing, cannot decode the packet!";
        return;
    }

    // trace successful decode
    onCamDecoded_(decodedCam);

    // Hand the received CAM to the ITS application or LDM
    onCam_(decodedCam);
}

void
IdsseCaBasicService::start()
{
    auto es =
        deps_.getOrThrow<EventScheduler, component::MissingDependency>("IdsseCaBasicService is missing EventScheduler!");

    if (params_.manualCamGenerationInterval)
    {
        checkEvent_ = es->schedule([this]() { generateStaticMode(); }, std::chrono::milliseconds::zero(),
                                   *params_.manualCamGenerationInterval);

        log_.debug() << "Standard CAM generation rules were disabled. A CAM will be generated every "
                     << streamWithUnit(*params_.manualCamGenerationInterval);
    }
    else
    {
        // standard mentions vehicle and RSU ITS-S but what can be classified as a vehicle is not mentioned
        // here everything besides RSUs was decided to be considered as a vehicle
        if (stationType_ != cdd::StationType_Value_ROAD_SIDE_UNIT)
        {
            checkEvent_ = es->schedule([this]() { generateStandardMode(); }, std::chrono::milliseconds::zero(),
                                       params_.tCheckCamGen);

            log_.debug() << "Standard CAM generation rules will apply. CAM generation rules will be checked every "
                         << streamWithUnit(params_.tCheckCamGen);
        }
        else  // schedule CAM generation with the configured RSU CAM generation frequency (1 Hz by default)
        {
            checkEvent_ = es->schedule([this]() { generateStaticMode(); }, std::chrono::milliseconds::zero(),
                                       params_.rsuCamGenerationInterval);

            log_.debug() << "A RSU CAM will be generated every " << streamWithUnit(params_.rsuCamGenerationInterval);
        }
    }
}

void
IdsseCaBasicService::generateStaticMode()
{
    // incrementing the counter for the low frequency container is necessary to provide the correct inclusion semantics
    // incrementing checkIntervalsSinceLastCam_ is superfluous but does not cause any harm since it is never checked
    incrementCheckIntervals();
    send(cam());
}

void
IdsseCaBasicService::generateStandardMode()
{
    // increment the counters for CAM generation bookkeeping
    incrementCheckIntervals();

    // current position is accessible through the position provider
    auto posData = positionProvider_.position();
    if (!posData)
    {
        // WARN is preferred here since having no valid position information is not an error
        // but possibly a temporary condition, e.g. no GPS fix. Thus, we might recover from it at
        // a later point in time.
        log_.warn() << "Cannot check CAM generation rules. No position information!";
        return;
    }

    // missing values for any of heading, speed and position means we have not sent a CAM before so send the first one
    if (!lastHeading_ || !lastPosition_ || !lastSpeed_)
    {
        send(cam());
        return;
    }

    // variables to hold current values for heading, position, speed
    auto heading = posData->heading;
    auto speed = posData->speed;
    auto position = posData->position;


    // check heading and speed since they are optional in the PositionVector returned by the position provider
    if (!heading || !speed)
    {
        log_.error() << "Cannot check CAM generation rules. No heading and speed information!";
        return;
    }

    // if we have sent a CAM before, a decision should be made based on the trigger conditions
    auto timeSinceLastCam = checkIntervalsToDuration(checkIntervalsSinceLastCam_);
    auto currentTGenCamDccValue = tGenCamDcc();

    // check CAM trigger conditions
    if (timeSinceLastCam >= currentTGenCamDccValue)
    {
        // condition 1 (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.3 - p.18)
        if (checkHeadingDelta(*heading, *lastHeading_) || checkSpeedDelta(*speed, *lastSpeed_) ||
            checkPositionDelta(position, *lastPosition_))
        {
            // T_GenCam shall be set to the time elapsed since the last CAM generation,
            // if a CAM is triggered due to condition 1 (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.3)
            tGenCam_ = timeSinceLastCam;

            // reset N_GenCam to its reference value
            nGenCam_ = params_.nGenCam.value();

            send(cam());
        }
        // condition 2 (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.3 - p.18)
        else if (timeSinceLastCam >= tGenCam_)
        {
            if (--nGenCam_ <= 0)
            {
                // T_GenCam has to be set to T_GenCamMax after triggering N_GenCam consecutive CAMs due to condition 2
                // (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.3)
                tGenCam_ = params_.tGenCamMax;
            }
            else
            {
                // NOTE: textual description ((ETSI EN 302 637-2 V1.4.1 - p.17) does nothing but the flowchart (ETSI EN
                // 302 637-2 V1.4.1 - p.41) sets tGenCam_ = std::max(timeSinceLastCam, currentGenCamDccValue) Because
                // tGenCam_ should hold the currently valid upper limit of the CAM generation interval the flowchart
                // version was selected. std::max is in fact not necessary as at this point timeSinceLastCam >=
                // currentGenCamDccValue always holds
                tGenCam_ = std::max(timeSinceLastCam, currentTGenCamDccValue);
            }

            send(cam());
        }
    }
}

bool
IdsseCaBasicService::send(boost::optional<Cam> cam)
{

    if (!cam)
    {
        log_.debug() << "Send Failed: Message to send does not exist!";
        return false;
    }

    //update latestCam once we know that cam isn't empty
    latestCam_ = cam;

    // trace generation event
    onCamGenerated_(*cam);

    Packet encodedCam;

    // encode the CAM
    if (auto dp = deps_.get<DataPresentation>())
    {
        try
        {
            encodedCam = dp->encodeCam(*cam);
        }
        catch (DataPresentation::EncodeFailed const& e)
        {
            log_.error() << e.what();
            return false;
        }
    }
    else
    {
        log_.error() << "Send Failed: Data presentation is missing, cannot encode the CAM!";
        return false;
    }

    // trace successful decode
    onCamEncoded_(encodedCam);

    // pass the encoded CAM to ITS Networking & Transport Layer
    if (auto btp = deps_.get<Btp>())
    {
        geonet::DataRequest gnParams{};
        generateDataRequestParameters(gnParams, *cam, params_);

        try
        {
            // pass the encoded CAM to the BTP layer (can throw)
            btp->sendNonInteractive(params_.destinationPort, params_.destinationPortInfo, gnParams,
                                    std::move(encodedCam));

            // send successful
            log_.trace() << "Sent CAM: " << cam->ShortDebugString();
            return true;
        }
        catch (std::exception const& e)
        {
            log_.error() << "Send failed: " << e.what();
            return false;
        }
        catch (...)
        {
            log_.error() << "Send Failed: Unknown error from lower layers";
            return false;
        }
    }
    else
    {
        log_.error() << "Send Failed: Cannot send CAM. Btp is missing!";
        return false;
    }
}

std::chrono::milliseconds
IdsseCaBasicService::checkIntervalsToDuration(CheckIntervalType checkIntervals) const
{
    if (params_.manualCamGenerationInterval)
    {
        return (*params_.manualCamGenerationInterval) * checkIntervals;
    }
    return params_.tCheckCamGen * checkIntervals;
}

bool
IdsseCaBasicService::shouldGenerateLowFrequencyContainer(CheckIntervalType checkIntervalsSinceLastLfContainer) const
{
    return checkIntervalsToDuration(checkIntervalsSinceLastLfContainer) >=
           params_.lowFrequencyContainerGenerationThreshold;
}

void
IdsseCaBasicService::incrementCheckIntervals()
{
    if (checkIntervalsSinceLastCam_ != std::numeric_limits<CheckIntervalType>::max())
    {
        checkIntervalsSinceLastCam_++;
    }

    if (checkIntervalsSinceLastLfContainer_ != std::numeric_limits<CheckIntervalType>::max())
    {
        checkIntervalsSinceLastLfContainer_++;
    }
}

bool
IdsseCaBasicService::checkHeadingDelta(double now, double prev) const
{
    return std::abs(now - prev) > params_.headingThreshold;
}

bool
IdsseCaBasicService::checkPositionDelta(Wgs84Position const& now, Wgs84Position const& prev) const
{
    return distance(now, prev) > params_.distanceThreshold;
}

bool
IdsseCaBasicService::checkSpeedDelta(double now, double prev) const
{
    return std::abs(now - prev) > params_.speedThreshold;
}

boost::optional<Cam>
IdsseCaBasicService::cam()
{

    if(isSuppressCAMs()){
        log_.debug() << "Supressing CAM messages";
        return boost::none;
    }
    // check the dependencies necessary to populate the required fields of a CAM message

    // current position is accessible through the position provider
    auto posData = positionProvider_.position();
    if (!posData)
    {
        // WARN is preferred here since having no valid position information is not a setup error
        // but possibly a temporary condition, e.g. no GPS fix. Thus, we might recover from it at
        // a later point in time.
        log_.warn() << "CAM creation failed: No position information!";
        return boost::none;
    }

    auto tp = deps_.get<TimeProvider>();
    if (!tp)
    {
        log_.error() << "CAM creation failed: No TimeProvider! Cannot set generation time for RSU stations and cannot "
                        "calculate time required for CAM generation!";
        return boost::none;
    }

    auto hub = deps_.get<DataHub>();
    if (!hub)
    {
        log_.error() << "CAM creation failed: No DataHub!";
        return boost::none;
    }

    auto cm = deps_.get<CertificateManager>();
    if (!cm)
    {
        log_.error() << "CAM creation failed: No CertificateManager! Cannot set station ID!";
        return boost::none;
    }

    // record the time when CAM generation is triggered
    TimePoint start = tp->now();

    Cam cam;
    cam.Clear();

    try
    {
        // Its Pdu header
        cdd::ItsPduHeader* header = cam.mutable_header();
        header->set_protocol_version(2);
        header->set_message_id(cdd::ItsPduHeader_MessageId_CAM);
        header->set_station_id(stationId_);

        // time stamp given in the vehicle ITS-S CAM shall correspond to the time at which
        // the reference position of the originating ITS-S given in this CAM was determined,
        // for RSUs the time stamp given in the RSU ITS-S CAM shall be the time of generation
        TimePoint generationTime;

        if (stationType_ != cdd::StationType_Value_ROAD_SIDE_UNIT)
        {
            // get the position capture time
            generationTime = posData->captureTime;
        }
        else
        {
            generationTime = tp->now();
        }

        // Set generation time (it's the real time here, not just a remainder)
        // can throw ItsClock::InvalidTime
        auto convertedTs =
            std::chrono::duration_cast<std::chrono::milliseconds>(ItsClock::from_sys(generationTime).time_since_epoch())
                .count();

        // can throw boost::numeric::bad_numeric_cast
        cam.mutable_payload()->set_generation_time(boost::numeric_cast<std::uint64_t>(convertedTs));

        /* 
        Ground Truth Data collection 
        */
        //GANlog
        log_.info() << "/TRUTH/;" << convertedTs << ";" << "G" << ";" << stationId_ << ";" << posData->speed.get_value_or(-1) << ";" << posData->heading.get_value_or(-1) << ";" << posData->position.getLatitude().value()<< ";" << posData->position.getLongitude().value();
   
        /*
        ATTACKS happen here 
        */

        //GANlog
        //log_.info() << "/GAN/;" << convertedTs << ";" << attackLabel << ";" << stationId_ << ";" << posData->speed.get_value_or(-1) << ";" << posData->heading.get_value_or(-1) << ";" << posData->position.getLatitude().value()<< ";" << posData->position.getLongitude().value();

        // Basic container
        addBasicContainer(cam, *posData);

        if (stationType_ != cdd::StationType_Value_ROAD_SIDE_UNIT)
        {
            // can throw datahub::ObjectInvalid
            addHighFrequencyContainer(cam, *posData, *hub);

            if (shouldGenerateLowFrequencyContainer(checkIntervalsSinceLastLfContainer_))
            {
                // can throw datahub::ObjectInvalid
                addLowFrequencyContainer(cam, *hub);
                checkIntervalsSinceLastLfContainer_ = 0;
            }
        }
        else  // station is a RSU
        {
            // it is assumed that high frequency container is mandatory for RSU ITSs but
            // because all values are optional it could be empty (a component providing protected communication zones is
            // not available)
            cam.mutable_payload()
                ->mutable_containers()
                ->mutable_high_frequency_container()
                ->mutable_rsu_container_high_frequency();
        }
    }
    catch (boost::numeric::bad_numeric_cast const& e)
    {
        log_.error() << "CAM creation failed: " << e.what();
        return boost::none;
    }
    catch (datahub::ObjectInvalid const& e)
    {
        log_.error() << "CAM creation failed: " << e.what();
        return boost::none;
    }
    catch (ItsClock::InvalidTime const& e)
    {
        log_.error() << "CAM creation failed: " << e.what();
        return boost::none;
    }
    catch (...)
    {
        log_.error() << "CAM creation failed!";
        return boost::none;
    }

    // update the cam generation counter
    checkIntervalsSinceLastCam_ = 0;

    // record the time at which the CAM is about to be delivered to networking transport layer
    TimePoint end = tp->now();

    // time required for CAM generation shall be less than 50ms (ETSI EN 302 637-2 V1.4.1 - chapter 6.1.5.1)
    reportCamGenerationTime(start, end);
    //seems like we don't have acceleration info
    //<< cam.payload().containers().high_frequency_container().basic_vehicle_container_high_frequency().longitudinal_acceleration().value().value() << ";" 


    return cam;
}

void
IdsseCaBasicService::addBasicContainer(Cam& cam, PositionVector const& posData)
{
    // Basic container
    cam::BasicContainer* basicContainer = cam.mutable_payload()->mutable_containers()->mutable_basic_container();
    basicContainer->mutable_station_type()->set_value(static_cast<std::uint32_t>(stationType_));

    auto pos = posData.position;
    cdd::ReferencePosition* rp = basicContainer->mutable_reference_position();
    rp->mutable_latitude()->set_value(pos.getLatitude().value());
    rp->mutable_longitude()->set_value(pos.getLongitude().value());

    // record the position included in the CAM as it will be used in checking the CAM trigger conditions
    lastPosition_ = pos;

    if (posData.altitude)
    {
        rp->mutable_altitude()->mutable_value()->set_value(*posData.altitude);

        if (posData.altitudeConfidence)
        {
            rp->mutable_altitude()->set_confidence(cdd::toAltitudeConfidence(*posData.altitudeConfidence));
        }
        else
        {
            rp->mutable_altitude()->set_confidence(cdd::Altitude_Confidence_UNAVAILABLE);
        }
    }

    auto pce = posData.positionConfidence;
    if (pce)
    {
        rp->mutable_pos_confidence_ellipse()->mutable_semi_major_confidence()->set_value(pce->semiMajorConfidence);
        rp->mutable_pos_confidence_ellipse()->mutable_semi_minor_confidence()->set_value(pce->semiMinorConfidence);
        rp->mutable_pos_confidence_ellipse()->mutable_semi_major_orientation()->set_value(pce->semiMajorOrientation);
    }
}

void
IdsseCaBasicService::addHighFrequencyContainer(Cam& cam, PositionVector const& posData, ezC2X::DataHub const& hub)
{
    // High frequency container
    cam::BasicVehicleContainerHighFrequency* hfContainer = cam.mutable_payload()
                                                               ->mutable_containers()
                                                               ->mutable_high_frequency_container()
                                                               ->mutable_basic_vehicle_container_high_frequency();

    // heading
    if (posData.heading)
    {
        hfContainer->mutable_heading()->mutable_value()->set_value(*posData.heading);

        // record the heading included in the CAM as it will be used in checking the CAM trigger conditions
        lastHeading_ = *posData.heading;

        if (posData.headingConfidence)
        {
            hfContainer->mutable_heading()->mutable_confidence()->set_value(*posData.headingConfidence);
        }
    }
    else
    {
        // reset the last heading value
        lastHeading_ = boost::none;
    }
    
    // speed
    if (posData.speed)
    {
        hfContainer->mutable_speed()->mutable_value()->set_value(*posData.speed);

        // record the speed included in the CAM as it will be used in checking the CAM trigger conditions
        lastSpeed_ = *posData.speed;

        if (posData.speedConfidence)
        {
            hfContainer->mutable_speed()->mutable_confidence()->set_value(*posData.speedConfidence);
        }
    }
    else
    {
        // reset the last speed value
        lastSpeed_ = boost::none;
    }

    // vehicle status fields
    if (hub.has(paths_.status.root))
    {
        datahub::Reader st = hub.reader(paths_.status.root);

        // drive direction
        if (st.hasValue<cdd::DriveDirection_Value>(paths_.status.driveDirection))
        {
            hfContainer->mutable_drive_direction()->set_value(
                st.get<cdd::DriveDirection_Value>(paths_.status.driveDirection));
        }

        // lane position
        if (st.hasValue<std::int32_t>(paths_.status.lanePosition))
        {
            hfContainer->mutable_lane_position()->set_value(st.get<std::int32_t>(paths_.status.lanePosition));
        }

        // performance class
        if (st.hasValue<std::uint32_t>(paths_.status.performanceClass))
        {
            hfContainer->mutable_performance_class()->set_value(st.get<std::uint32_t>(paths_.status.performanceClass));
        }
    }

    // vehicle features fields
    if (hub.has(paths_.features.root))
    {
        datahub::Reader ft = hub.reader(paths_.features.root);

        // length
        if (ft.hasValue<double>(paths_.features.length))
        {
            hfContainer->mutable_vehicle_length()->mutable_value()->set_value(ft.get<double>(paths_.features.length));

            // length confidence indication
            if (ft.hasValue<cdd::VehicleLength_ConfidenceIndication>(paths_.features.lengthConfidenceIndication))
            {
                hfContainer->mutable_vehicle_length()->set_confidence_indication(
                    ft.get<cdd::VehicleLength_ConfidenceIndication>(paths_.features.lengthConfidenceIndication));
            }
            else
            {
                hfContainer->mutable_vehicle_length()->set_confidence_indication(
                    cdd::VehicleLength_ConfidenceIndication_UNAVAILABLE);
            }
        }

        // width
        if (ft.hasValue<double>(paths_.features.width))
        {
            hfContainer->mutable_vehicle_width()->set_value(ft.get<double>(paths_.features.width));
        }
    }

    // vehicle rotation fields
    if (hub.has(paths_.rotation.root))
    {
        datahub::Reader rt = hub.reader(paths_.rotation.root);

        // curvature
        if (rt.hasValue<double>(paths_.rotation.curvature))
        {
            hfContainer->mutable_curvature()->mutable_value()->set_value(rt.get<double>(paths_.rotation.curvature));

            // curvature confidence
            if (rt.hasValue<cdd::Curvature_Confidence>(paths_.rotation.curvatureConfidence))
            {
                hfContainer->mutable_curvature()->set_confidence(
                    rt.get<cdd::Curvature_Confidence>(paths_.rotation.curvatureConfidence));
            }
            else
            {
                hfContainer->mutable_curvature()->set_confidence(cdd::Curvature_Confidence_UNAVAILABLE);
            }

            // curvature calculation mode
            if (rt.hasValue<cdd::CurvatureCalculationMode_Value>(paths_.rotation.curvatureCalculationMode))
            {
                hfContainer->mutable_curvature_calculation_mode()->set_value(
                    rt.get<cdd::CurvatureCalculationMode_Value>(paths_.rotation.curvatureConfidence));
            }
        }

        // yaw
        if (rt.hasValue<double>(paths_.rotation.yaw))
        {
            hfContainer->mutable_yaw_rate()->mutable_value()->set_value(rt.get<double>(paths_.rotation.yaw));

            // yaw confidence
            if (rt.hasValue<cdd::YawRate_Confidence>(paths_.rotation.yawConfidence))
            {
                hfContainer->mutable_yaw_rate()->set_confidence(
                    rt.get<cdd::YawRate_Confidence>(paths_.rotation.yawConfidence));
            }
            else
            {
                hfContainer->mutable_yaw_rate()->set_confidence(cdd::YawRate_Confidence_UNAVAILABLE);
            }
        }
    }

    // vehicle acceleration fields
    if (hub.has(paths_.acceleration.root))
    {
        datahub::Reader acc = hub.reader(paths_.acceleration.root);

        // longitudinal
        if (acc.hasValue<double>(paths_.acceleration.longitudinal))
        {
            hfContainer->mutable_longitudinal_acceleration()->mutable_value()->set_value(
                acc.get<double>(paths_.acceleration.longitudinal));

            if (acc.hasValue<double>(paths_.acceleration.longitudinalConfidence))
            {
                hfContainer->mutable_longitudinal_acceleration()->mutable_confidence()->set_value(
                    acc.get<double>(paths_.acceleration.longitudinalConfidence));
            }
        }

        // lateral
        if (acc.hasValue<double>(paths_.acceleration.lateral))
        {
            hfContainer->mutable_lateral_acceleration()->mutable_value()->set_value(
                acc.get<double>(paths_.acceleration.lateral));

            if (acc.hasValue<double>(paths_.acceleration.lateralConfidence))
            {
                hfContainer->mutable_lateral_acceleration()->mutable_confidence()->set_value(
                    acc.get<double>(paths_.acceleration.lateralConfidence));
            }
        }

        // vertical
        if (acc.hasValue<double>(paths_.acceleration.vertical))
        {
            hfContainer->mutable_vertical_acceleration()->mutable_value()->set_value(
                acc.get<double>(paths_.acceleration.vertical));

            if (acc.hasValue<double>(paths_.acceleration.verticalConfidence))
            {
                hfContainer->mutable_vertical_acceleration()->mutable_confidence()->set_value(
                    acc.get<double>(paths_.acceleration.verticalConfidence));
            }
        }
    }

    // vehicle acceleration control fields
    if (hub.has(paths_.accelerationControl.root))
    {
        datahub::Reader ac = hub.reader(paths_.accelerationControl.root);
        cdd::AccelerationControl* accControl = hfContainer->mutable_acceleration_control();

        accControl->set_acc_engaged(ac.hasValue<bool>(paths_.accelerationControl.accEngaged)
                                        ? ac.get<bool>(paths_.accelerationControl.accEngaged)
                                        : false);
        accControl->set_brake_pedal_engaged(ac.hasValue<bool>(paths_.accelerationControl.brakePedalEngaged)
                                                ? ac.get<bool>(paths_.accelerationControl.brakePedalEngaged)
                                                : false);
        accControl->set_collision_warning_engaged(ac.hasValue<bool>(paths_.accelerationControl.collisionWarningEngaged)
                                                      ? ac.get<bool>(paths_.accelerationControl.collisionWarningEngaged)
                                                      : false);
        accControl->set_cruise_control_engaged(ac.hasValue<bool>(paths_.accelerationControl.cruiseControlEngaged)
                                                   ? ac.get<bool>(paths_.accelerationControl.cruiseControlEngaged)
                                                   : false);
        accControl->set_emergency_brake_engaged(ac.hasValue<bool>(paths_.accelerationControl.emergencyBrakeEngaged)
                                                    ? ac.get<bool>(paths_.accelerationControl.emergencyBrakeEngaged)
                                                    : false);
        accControl->set_gas_pedal_engaged(ac.hasValue<bool>(paths_.accelerationControl.gasPedalEngaged)
                                              ? ac.get<bool>(paths_.accelerationControl.gasPedalEngaged)
                                              : false);
        accControl->set_speed_limiter_engaged(ac.hasValue<bool>(paths_.accelerationControl.speedLimiterEngaged)
                                                  ? ac.get<bool>(paths_.accelerationControl.speedLimiterEngaged)
                                                  : false);
    }

    // vehicle steering wheel fields
    if (hub.has(paths_.steeringWheel.root))
    {
        datahub::Reader sw = hub.reader(paths_.steeringWheel.root);

        // angle
        if (sw.hasValue<double>(paths_.steeringWheel.angle))
        {
            hfContainer->mutable_steering_wheel_angle()->mutable_value()->set_value(
                sw.get<double>(paths_.steeringWheel.angle));

            // angle confidence
            if (sw.hasValue<double>(paths_.steeringWheel.angleConfidence))
            {
                hfContainer->mutable_steering_wheel_angle()->mutable_confidence()->set_value(
                    sw.get<double>(paths_.steeringWheel.angleConfidence));
            }
        }
    }

    // cen dsrc tolling zone
    if (hub.has(paths_.cenDsrcTollingZone.root))
    {
        datahub::Reader cdtz = hub.reader(paths_.cenDsrcTollingZone.root);

        // longitude, latitude
        if (cdtz.hasValue<Wgs84Position>(paths_.cenDsrcTollingZone.position))
        {
            auto pos = cdtz.get<Wgs84Position>(paths_.cenDsrcTollingZone.position);
            hfContainer->mutable_cen_dsrc_tolling_zone()->mutable_protected_zone_latitude()->set_value(
                pos.getLatitude().value());
            hfContainer->mutable_cen_dsrc_tolling_zone()->mutable_protected_zone_longitude()->set_value(
                pos.getLongitude().value());
        }

        // ID
        if (cdtz.hasValue<std::uint32_t>(paths_.cenDsrcTollingZone.id))
        {
            hfContainer->mutable_cen_dsrc_tolling_zone()->mutable_cen_dsrc_tolling_zone_id()->set_value(
                cdtz.get<std::uint32_t>(paths_.cenDsrcTollingZone.id));
        }
    }
}

void
IdsseCaBasicService::addLowFrequencyContainer(Cam& cam, ezC2X::DataHub const& hub) const
{
    // Low frequency container
    cam::BasicVehicleContainerLowFrequency* lfContainer = cam.mutable_payload()
                                                              ->mutable_containers()
                                                              ->mutable_low_frequency_container()
                                                              ->mutable_basic_vehicle_container_low_frequency();

    // vehicle status fields
    if (hub.has(paths_.status.root))
    {
        datahub::Reader st = hub.reader(paths_.status.root);

        // vehicle role
        if (st.hasValue<cdd::VehicleRole_Value>(paths_.status.vehicleRole))
        {
            lfContainer->mutable_vehicle_role()->set_value(st.get<cdd::VehicleRole_Value>(paths_.status.vehicleRole));
        }
    }

    // vehicle exterior lights fields
    if (hub.has(paths_.exteriorLights.root))
    {
        datahub::Reader exteriorLights = hub.reader(paths_.exteriorLights.root);
        cdd::ExteriorLights* extLights = lfContainer->mutable_exterior_lights();

        extLights->set_daytime_running_lights_on(
            exteriorLights.hasValue<bool>(paths_.exteriorLights.daytimeRunningLightsOn)
                ? exteriorLights.get<bool>(paths_.exteriorLights.daytimeRunningLightsOn)
                : false);
        extLights->set_fog_light_on(exteriorLights.hasValue<bool>(paths_.exteriorLights.fogLightOn)
                                        ? exteriorLights.get<bool>(paths_.exteriorLights.fogLightOn)
                                        : false);
        extLights->set_high_beam_headlights_on(
            exteriorLights.hasValue<bool>(paths_.exteriorLights.highBeamHeadlightsOn)
                ? exteriorLights.get<bool>(paths_.exteriorLights.highBeamHeadlightsOn)
                : false);
        extLights->set_left_turn_signal_on(exteriorLights.hasValue<bool>(paths_.exteriorLights.leftTurnSignalOn)
                                               ? exteriorLights.get<bool>(paths_.exteriorLights.leftTurnSignalOn)
                                               : false);
        extLights->set_low_beam_headlights_on(exteriorLights.hasValue<bool>(paths_.exteriorLights.lowBeamHeadlightsOn)
                                                  ? exteriorLights.get<bool>(paths_.exteriorLights.lowBeamHeadlightsOn)
                                                  : false);
        extLights->set_parking_lights_on(exteriorLights.hasValue<bool>(paths_.exteriorLights.parkingLightsOn)
                                             ? exteriorLights.get<bool>(paths_.exteriorLights.parkingLightsOn)
                                             : false);
        extLights->set_reverse_light_on(exteriorLights.hasValue<bool>(paths_.exteriorLights.reverseLightOn)
                                            ? exteriorLights.get<bool>(paths_.exteriorLights.reverseLightOn)
                                            : false);
        extLights->set_right_turn_signal_on(exteriorLights.hasValue<bool>(paths_.exteriorLights.rightTurnSignalOn)
                                                ? exteriorLights.get<bool>(paths_.exteriorLights.rightTurnSignalOn)
                                                : false);
    }

    // a component providing the path history is not available so skip it
}

void
IdsseCaBasicService::reportCamGenerationTime(TimePoint start, TimePoint end)
{
    auto totalGenTime = std::chrono::duration_cast<std::chrono::milliseconds>(start - end);

    // warn if the configured CAM generation time is exceeded
    if (totalGenTime >= params_.maxCamGenerationTime)
    {
        log_.warn() << "CAM generation took too long (i.e. >= " << streamWithUnit(params_.maxCamGenerationTime)
                    << ")! : " << streamWithUnit(totalGenTime);
    }
}

boost::optional<ezC2X::PositionVector>
IdsseCaBasicService::getPositionVector()
{
    return positionProvider_.position();
}

void IdsseCaBasicService::setAttackType(int _attackType){
    // IdsseCaBasicService::log_.info() << "Changing to attack type to " << _attackType;
    attackType = _attackType;
}

void
IdsseCaBasicService::setAttack1(std::vector<double> _profile){
    log_.info() << "Updating A1 irregular attack profile with size " << _profile.size();
    a1IrregularSpeedProfile = _profile;
}

void 
IdsseCaBasicService::setAttack2(std::uint32_t _a2MaxRandomSpeed){
    log_.info() << "Updating A2 random speed attack with maximum value " << _a2MaxRandomSpeed;
    a2MaxRandomSpeed = _a2MaxRandomSpeed;
}

void 
IdsseCaBasicService::triggerAttack(){
    if(!isAttackActive())
        attackActive = true;
    attackStep++;
}

bool
IdsseCaBasicService::isAttackActive(){
    return attackActive;
}

void 
IdsseCaBasicService::setAttackActive(bool active){
    attackActive = active;
}

int
IdsseCaBasicService::getAttackStep(){
    return attackStep;
}
double
IdsseCaBasicService::getAttack1(int _attackStep){
    // log_.info() << "Profile size: " << a1IrregularSpeedProfile.size() << " current step " << _attackStep;
    if(static_cast<long unsigned int>(_attackStep) < a1IrregularSpeedProfile.size())
        return a1IrregularSpeedProfile[_attackStep];    
    else{
        setAttackActive(false);
        return 0.0;
    }
}

void 
IdsseCaBasicService::setSuppressCAMs(bool _suppressCAMs){
    suppressCAMs = _suppressCAMs;
}
bool
IdsseCaBasicService::isSuppressCAMs(){
    return suppressCAMs;
}

boost::optional<Cam>
IdsseCaBasicService::getLatestCam(){
    return latestCam_;
}

boost::optional<double>
IdsseCaBasicService::getlastHeading()
{
    return boost::optional<double>();
}


boost::optional<Wgs84Position>
IdsseCaBasicService::getlastPosition()
{
    return boost::optional<Wgs84Position>();
}

boost::optional<double>
IdsseCaBasicService::getlastSpeed()
{
    return boost::optional<double>();
}

std::chrono::milliseconds
IdsseCaBasicService::getTimeSinceLastCam(){
return checkIntervalsToDuration(checkIntervalsSinceLastCam_);
}

}  // namespace ezC2X
