#include <iostream>
#include <CarIDS.hpp>
#include <cmath>
#include "ezC2X/core/geographic/Distance.hpp"

CarIDS::CarIDS() : log_("CarIDS"){}

CarIDS::~CarIDS(){}

bool 
CarIDS::rangePlausability(Report msg) {
    auto pos1 = msg.getCam().pos;
    auto pos2 = msg.getMetaData().positionOnReceieve;
    auto wgsPos1 = ezC2X::Wgs84Position::wrap(std::get<1>(pos1), std::get<0>(pos1)); 
    auto wgsPos2 = ezC2X::Wgs84Position::wrap(std::get<1>(pos2), std::get<0>(pos2)); 
    double dist = ezC2X::distance(wgsPos1,wgsPos2);
    return dist < PLAUSABILITY_RANGE;
}

bool 
CarIDS::speedPlausability(Report msg) {
    return msg.getCam().speed <= MAX_SPEED;
}

bool 
CarIDS::positionConsistency(Report old_msg, Report new_msg, double time, double max_acc, double max_dec) {
    double old_speed = old_msg.getCam().speed;
    double v_diff = new_msg.getCam().speed - old_speed;

    if (v_diff < 0) {
        std::swap(max_acc, max_dec);
    }

    double t_acc = (-max_dec * time + v_diff) / (-(max_dec) + max_acc);
    double t_dec = time - t_acc;

    double v_tmp_acc = old_speed + max_acc * t_acc;
    double v_tmp_dec = old_speed + max_dec * t_dec;

    double bound_upper = (old_speed * t_acc + max_acc * pow(t_acc, 2)) +
                         (v_tmp_acc * t_dec + max_dec * pow(t_dec, 2));
    double bound_lower = (old_speed * t_dec + max_dec * pow(t_dec, 2)) +
                         (v_tmp_dec * t_acc + max_acc * pow(t_acc, 2));

    if (v_diff < 0) {
        std::swap(bound_lower, bound_upper);
    }
    auto pos1 = old_msg.getCam().pos;
    auto pos2 = new_msg.getCam().pos;
    auto wgsPos1 = ezC2X::Wgs84Position::wrap(std::get<1>(pos1), std::get<0>(pos1)); 
    auto wgsPos2 = ezC2X::Wgs84Position::wrap(std::get<1>(pos2), std::get<0>(pos2)); 
    double dist = ezC2X::distance(wgsPos1,wgsPos2);

    //Test print to see values for bounds and dist
    //std::cout << "Lower: " << bound_lower << ", Upper: " << bound_upper << ", Dist: " << dist << std::endl;

    return bound_lower < dist && dist < bound_upper;
}

bool 
CarIDS::speedConsistency(double speed_diff, double time) {
    return MAX_DEC * time < speed_diff && speed_diff < MAX_ACC * time;
}

bool 
CarIDS::compareMsgConsistency(Report old_msg, Report new_msg) {
    double t_diff = double(new_msg.getCam().generationDeltaTime - old_msg.getCam().generationDeltaTime) / 1000.0;
    bool  plausableRange = rangePlausability(new_msg);
    bool plausableSpeed = speedPlausability(new_msg);
    bool consistentSpeed = speedConsistency(old_msg.getCam().speed - new_msg.getCam().speed, t_diff);
    bool consistentPosition = positionConsistency(old_msg, new_msg, t_diff, MAX_ACC, MAX_DEC);
    log_.info() << "Plausable Range: " << plausableRange << ", plausableSpeed: " << plausableSpeed << ", consistentSpeed: " << consistentSpeed << ", consistentPosition: " << consistentPosition;

    return plausableRange &&
           plausableSpeed &&
           consistentSpeed &&
           consistentPosition;

}

bool 
CarIDS::carIDS(Report msg_latest) {
    uint32_t id = msg_latest.getCam().id;
    bool detected = false;
    if (!msg_stacks_[id].empty()) {
        Report msg_prev = msg_stacks_[id].back();
        if (!compareMsgConsistency(msg_prev, msg_latest)) {
            misbehaved_.push_back(msg_latest);
            detected = true;
        }
    }
    msg_stacks_[id].push_back(msg_latest);
    return detected;
}

std::vector<Report>
CarIDS::getMisbehavedMessages()
{
    return misbehaved_;
}

std::map<uint32_t, std::vector<Report>>
CarIDS::getMsgStacks()
{
    return msg_stacks_;
}
