#ifndef EGOPOS
#define EGOPOS
#include "ezC2X/core/geographic/CartesianPosition.hpp"
#include "ezC2X/core/geographic/Wgs84Position.hpp"

    struct EgoPos {
        ezC2X::CartesianPosition cartPos;
        ezC2X::Wgs84Position wgsPos;
    };

#endif