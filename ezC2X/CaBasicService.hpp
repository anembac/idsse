/*
 * Copyright (c) [2019] Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * acting on behalf of its Fraunhofer Institute for Cognitive Systems IKS. All rights reserved.
 *
 * This Software is licensed under the terms and conditions of the MIT License.
 * See LICENSE file in the project root for details.
 */

/*!
 * @brief ITS application or LDM interface for the CA Basic Service.
 * @author Yagmur Sevilmis
 * @file CaBasicService.hpp
 */
#ifndef EZC2X_FACILITY_CAM_CA_BASIC_SERVICE_HPP
#define EZC2X_FACILITY_CAM_CA_BASIC_SERVICE_HPP

#include <boost/signals2.hpp>
#include "ezC2X/facility/cam/Cam.pb.h"
#include "ezC2X/core/logging/Logger.hpp"
#include "ezC2X/security/attacker/AttackTypes.hpp"
#include "ezC2X/core/position/PositionVector.hpp"
#include "ezC2X/core/geographic/Wgs84Position.hpp"


namespace ezC2X
{
/*!
 * @brief ITS application or LDM interface for CA Basic Service.
 *
 * The interface follows the specification for ETSI Cooperative Awareness Basic Service in ETSI EN 302 637-2 V1.4.1
 * (2019-04) (chapter 5.3.1) i.e. CAM reception management (IF.CAM).
 *
 * @ingroup cam
 */
class CaBasicService
{
public:
    //! Virtual destructor for proper subclassing
    virtual ~CaBasicService() = default;

    /*!
     * @brief Signal for CAMs.
     *
     * @param ezC2X::CAM received CAM packet
     */
    using SignalOnCam = boost::signals2::signal<void(ezC2X::Cam const&)>;

    /*!
     * @brief Subscribe for received CAMs.
     * @param handler   Handler to call for each received CAM.
     * @return Connection object for the signal-slot setup.
     */
    virtual boost::signals2::connection
    subscribeOnCam(SignalOnCam::slot_type const& handler) = 0;

    /*!
     * @brief Set the type of attack that shall be executed.
     * @param int   values greater 0 are assigned to various attacks.
    */
    virtual void setAttackType(int) = 0;
    
    /*!
        * @brief Set additional parameters for attack type 1, i.e., set the speed profile that should be spoofed.
        * @param std::vector<double>   A list of the speed differences to the attackers actual speed.
    */
    virtual void setAttack1(std::vector<double> a1IrregularSpeedProfile) = 0;

    /*!
     * @brief Return the current speed difference that is being applied in the given attack step. 
     * @return Current speed difference between actual and spoofed speed.
     */
    virtual double getAttack1(int _attackStep) = 0;

    /*!
    * @brief Set additional parameters for attack type 2, i.e., maximum value for random speed spoofing.
    * @param std::uint32_t Maximum speed difference allowed for in this attack. 
    */
    virtual void setAttack2(std::uint32_t a2MaxRandomSpeed) = 0;

    /*!
    * @brief Trigger the attack or increase attack step by one.
    */
    virtual void triggerAttack() = 0;

    /*!
     * @brief Function to active or deactivate an attack.
     * @param bool  set attack active (true) or disable it (false)
     */
    virtual void setAttackActive(bool active) = 0;

    /*!
     * @brief Get a bool telling whether the attack is active or disabled.
     * @return True if attack is active, otherwise false.
     */
    virtual bool isAttackActive() = 0;

    /*!
     * @brief Get the current attack step (attack progress) needed by some attacks to know when to proceed with next step, e.g., next value when spoofing the speed.
     * @return Current attack step.
     */
    virtual int getAttackStep() = 0;

    /*!
     * @brief Suppress to send CAM messages. Needed in this simulation environment for simulating some of the attacks, e.g., Sybil attacks.
     * @param bool  True if NO CAM messages should be send, false if CAMs should be sent.
     */
    virtual void setSuppressCAMs(bool _sendCams) = 0;

    /*!
     * @brief Query whether the sending of CAM messages is suppressed.
     * @return True if no CAM messages are being sent, false if CAM messages are being sent.
     */
    virtual bool isSuppressCAMs() = 0;

        /*!
    * @brief Returns a pointer to the service's position provider.
    *
    */
    virtual boost::optional<ezC2X::PositionVector>
    getPositionVector() = 0;

    virtual void spoof() = 0;

    //! Last heading value
    virtual boost::optional<double> getlastHeading() = 0;

    //! Last position data
    virtual boost::optional<Wgs84Position> getlastPosition() = 0;

    //! Last speed value
    virtual boost::optional<double> getlastSpeed() = 0;


    virtual std::chrono::milliseconds getTimeSinceLastCam() = 0;

private: 

};

}  // namespace ezC2X

#endif  // EZC2X_FACILITY_CAM_CA_BASIC_SERVICE_HPP
