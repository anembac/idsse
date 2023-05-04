#include <iostream>
#include <car_ids.hpp>
#include <cmath>

bool range_plausability(Report msg) {
    double dist = distance(msg.getCam().pos, msg.getMetaData().positionOnReceieve);
    return dist < PLAUSABILITY_RANGE;
}

bool speed_plausability(Report msg) {
    return msg.getCam().speed <= MAX_SPEED;
}

bool position_consistency(Report old_msg, Report new_msg, double time, double max_acc, double max_dec) {
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

    double dist = distance(old_msg.getCam().pos, new_msg.getCam().pos);

    //Test print to see values for bounds and dist
    //std::cout << "Lower: " << bound_lower << ", Upper: " << bound_upper << ", Dist: " << dist << std::endl;

    return bound_lower < dist && dist < bound_upper;
}

bool speed_consistency(double speed_diff, double time) {
    return MAX_DEC * time < speed_diff && speed_diff < MAX_ACC * time;
}

bool cmp_msg_consistency(Report old_msg, Report new_msg) {
    double t_diff = (new_msg.getCam().generationDeltaTime - old_msg.getCam().generationDeltaTime) / 1000;

    return range_plausability(new_msg) &&
           speed_plausability(new_msg) &&
           speed_consistency(old_msg.getCam().speed - new_msg.getCam().speed, t_diff)  &&
           position_consistency(old_msg, new_msg, t_diff, MAX_ACC, MAX_DEC);

}

bool car_ids(Report msg_latest) {
    uint32_t id = msg_latest.getCam().id;
    if (!msg_stacks[id].empty()) {
        Report msg_prev = msg_stacks[id].back();
        if (!cmp_msg_consistency(msg_prev, msg_latest)) {
            misbehaved.push_back(msg_latest);
        }
    }
    msg_stacks[id].push_back(msg_latest);
    return false;
}

double distance(std::tuple<double,double> pos1, std::tuple<double,double> pos2) {
    return sqrt(pow((std::get<0>(pos2) - std::get<0>(pos1)), 2) + 
                pow((std::get<1>(pos2) - std::get<1>(pos1)), 2));
}
