/*
 * Copyright (c) [2019] Fraunhofer-Gesellschaft zur Foerderung der angewandten Forschung e.V.
 * acting on behalf of its Fraunhofer Institute for Cognitive Systems IKS. All rights reserved.
 *
 * This Software is licensed under the terms and conditions of the MIT License.
 * See LICENSE file in the project root for details.
 */

/*!
 * @brief defines interface for passing commands to vehicle control.
 * @author Henning Schepker
 * @file VehicleControlInterface.hpp
 */

#ifndef EZC2X_FACILITY_VEHICLE_CONTROL_VEHICLE_CONTROL_INTERFACE_HPP
#define EZC2X_FACILITY_VEHICLE_CONTROL_VEHICLE_CONTROL_INTERFACE_HPP

#include <string>
#include <vector>
#include <ezC2X/core/geographic/CartesianPosition.hpp>
#include <ezC2X/core/geographic/Wgs84Position.hpp>

namespace ezC2X
{
/*!
 * @brief abstract class which defines interface for passing commands to vehicle control.
 *
 * @ingroup vehicle_control
 */
typedef unsigned char UByte;


class VehicleControlInterface
{
public:
    //! Virtual destructor for proper subclassing
    virtual ~VehicleControlInterface() = default;
    /*
    virtual int16_t 
    getID() = 0;
    */

    virtual void
    setColor(int r, int g, int b, int a) = 0;

    virtual std::string
    getId() = 0;

    virtual double
    getSpeed() = 0;

    /*!
     * @brief get the current lateral speed of the vehicle.
     */
    virtual double
    getSpeedLateral() = 0;

    /*!
     * @brief get maximum speed of the vehicle.
     */
    virtual double
    getMaxSpeed() = 0;

    /*!
     * @brief get maximum lateral speed of the vehicle.
     */
    virtual double
    getMaxSpeedLateral() = 0;

    /*!
     * @brief get the current acceleration of the vehicle.
     */
    virtual double
    getAcceleration() = 0;

    /*!
     * @brief get the maximum acceleration of the vehicle.
     */
    virtual double
    getMaxAcceleration() = 0;

    /*!
     * @brief get the maximum deceleration of the vehicle.
     */
    virtual double
    getMaxDeceleration() = 0;

    /*!
     * @brief get the current front bumper position of the vehicle.
     */
    virtual Wgs84Position
    getFrontBumperPosition() = 0;

    /*!
     * @brief get the current front bumper position of the vehicle.
     */
    virtual CartesianPosition
    getFrontBumperPositionXY() = 0;

    /*!
     * @brief get the current center position of the vehicle.
     */
    virtual Wgs84Position
    getCenterPosition() = 0;

    /*!
     * @brief get the current center position of the vehicle.
     */
    virtual CartesianPosition
    getCenterPositionXY() = 0;

    /*!
     * @brief get the current lateral position of the vehicle.
     */
    virtual double
    getPositionLateral() = 0;

    /*!
     * @brief get the position of the vehicle in traveling direction along the current road (or edge in SUMO).
     */
    virtual double
    getPositionLongitudinal() = 0;

    /*!
     * @brief get total traveled distance of the vehicle.
     */
    virtual double
    getDistance() = 0;

    /*!
     * @brief get the current heading of the vehicle.
     */
    virtual double
    getHeading() = 0;

    /*!
     * @brief get the index of the lane the vehicle is in.
     */
    virtual int
    getLane() = 0;

    /*!
     * @brief get the ID of the lane the vehicle is in.
     */
    virtual std::string
    getLaneId() = 0;

    /*!
     * @brief get the relative position with the lane the vehicle is in (0.0 is right edge and 1.0 is left edge).
     */
    virtual double
    getRelativeLanePosition() = 0;

    /*!
     * @brief get the total number of lanes of the current road.
     */
    virtual int
    getNumLanes() = 0;

    /*!
     * @brief get the road id of the current road.
     */
    virtual std::string
    getRoadId() = 0;

    /*!
     * @brief get the route id of the current road.
     */
    virtual std::string
    getRouteId() = 0;

    /*!
     * @brief get the route of a vehicle.
     */
    virtual std::vector<std::string>
    getRoute() = 0;

    /*!
     * @brief set the route of a vehicle
     */
    virtual void setRoute(std::vector<std::string>) = 0;

    /*!
     * @brief set stop for a vehicle
     */
    virtual void
    setStop(std::string edgeID, double endpos, int lane, double stopduration, int stopflag, double startpos,
            double stopuntil) = 0;

    /*!
     * @brief reroute parkingArea of a vehicle
     */
    virtual void reroutePark(std::string) = 0;

    /*!
     * @brief get the length of the vehicle.
     */
    virtual double
    getLength() = 0;

    /*!
     * @brief get the width of the vehicle.
     */
    virtual double
    getWidth() = 0;

    /*!
     * @brief get the height of the vehicle.
     */
    virtual double
    getHeight() = 0;

    /*!
     * @brief disables lower layer functions which ensure automatic safe driving, enabling safe driving via vehicle
     * control interface automatic safe driving should be enabled by default at lower layers
     */
    virtual void
    disableAutomaticSafeDriving() = 0;

    /*!
     * @brief enables lower layer functions which ensure automatic safe driving
     *        automatic safe driving should be enabled by default at lower layers
     */
    virtual void
    enableAutomaticSafeDriving() = 0;

    /*!
     * @brief change lane over given duration.
     *
     * Parameters:
     * - ID of the lange to change to
     * - duration for the change
     */
    virtual void
    changeLane(int laneID, double duration) = 0;

    /*!
     * @brief set new speed.
     *
     * Parameters:
     * - new speed to set
     */
    virtual void
    setSpeed(double speed) = 0;

    /*!
     * @brief set maximum lateral speed of the vehicle.
     *
     * Parameters:
     * - new max speed to set
     */
    virtual void
    setMaxSpeedLateral(double max_speed) = 0;

    /*!
     * @brief gradually change the speed over given stringduration.
     *
     * Parameters:
     * - new speed to set
     * - duration for the change
     */
    virtual void
    slowDown(double speed, double duration) = 0;

    /*!
     * @brief set new position for vehicle.
     *
     * Parameters:
     * - ID of the road
     * - ID of the lange to change to
     * - new position
     * - heading (if default value, then use the angle of the edge)
     */
    virtual void
    moveTo(std::string edgeID, int lane, Wgs84Position newPosition, double heading = -1000000.0) = 0;

    /*!
     * @brief set new position for vehicle.
     *
     * Parameters:
     * - ID of the road
     * - ID of the lange to change to
     * - new position
     * - heading (if default value, then use the angle of the edge)
     */
    virtual void
    moveToXY(std::string edgeID, int lane, CartesianPosition newPosition, double heading = -1000000.0) = 0;


};

}  // namespace ezC2X

#endif  // EZC2X_FACILITY_VEHICLE_CONTROL_VEHICLE_CONTROL_INTERFACE_HPP
