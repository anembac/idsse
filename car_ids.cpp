#include <cmath>
#include <vector>
#include <iostream>

const int PLAUSABILITY_RANGE = 500;
const int MAX_SPEED = 20;
const double MAX_ACC = 2.6;
const double MAX_DEC = -4.5;

struct CarMessage {
    double timestamp;
    std::pair<double, double> position;
    double speed;
};

std::vector<CarMessage> msg_stack;

bool range_plausability(std::pair<double, double> car_position, std::pair<double, double> mycar_position) {
    double dx = car_position.first - mycar_position.first;
    double dy = car_position.second - mycar_position.second;
    double dist = std::sqrt(dx * dx + dy * dy);
    return dist < PLAUSABILITY_RANGE;
}

bool speed_plausability(CarMessage msg) {
    return msg.speed <= MAX_SPEED;
}

bool position_consistency(CarMessage old_msg, CarMessage new_msg, double time, double max_acc, double max_dec) {
    double v_diff = new_msg.speed - old_msg.speed;

    if (v_diff < 0) {
        std::swap(max_acc, max_dec);
    }

    double t_acc = (-max_dec * time + v_diff) / (-(max_dec) + max_acc);
    double t_dec = time - t_acc;

    double v_tmp_acc = old_msg.speed + max_acc * t_acc;
    double v_tmp_dec = old_msg.speed + max_dec * t_dec;

    double bound_upper = (old_msg.speed * t_acc + max_acc * pow(t_acc, 2)) +
                         (v_tmp_acc * t_dec + max_dec * pow(t_dec, 2));
    double bound_lower = (old_msg.speed * t_dec + max_dec * pow(t_dec, 2)) +
                         (v_tmp_dec * t_acc + max_acc * pow(t_acc, 2));

    if (v_diff < 0) {
        std::swap(bound_lower, bound_upper);
    }

    double dist = std::sqrt(std::pow(new_msg.position.first  - old_msg.position.first, 2) +
                            std::pow(new_msg.position.second - old_msg.position.second, 2));

    std::cout << "Lower: " << bound_lower << ", Upper: " << bound_upper << ", Dist: " << dist << std::endl;

    return bound_lower < dist && dist < bound_upper;
}

bool speed_consistency(double speed_diff, double time) {
    return MAX_DEC * time < speed_diff && speed_diff < MAX_ACC * time;
}

bool cmp_all(CarMessage old_msg, CarMessage new_msg) {
    double t_diff = new_msg.timestamp - old_msg.timestamp;

    return range_plausability(old_msg.position, new_msg.position) &&
           speed_plausability(new_msg) &&
           position_consistency(old_msg, new_msg, t_diff, MAX_ACC, MAX_DEC);
}

bool car_ids(CarMessage msg_latest, std::pair<double, double> mycar_position) {
    if (!msg_stack.empty()) {
        CarMessage msg_prev = msg_stack.back();
        bool misbehaved = !cmp_all(msg_prev, msg_latest);
        if (misbehaved) {
            return true;
        }
    }
    msg_stack.push_back(msg_latest);
    return false;
}

double distance(std::pair<double, double> pos1, std::pair<double, double> pos2) {
    double dx = pos2.first - pos1.first;
    double dy = pos2.second - pos1.second;
    return std::sqrt(dx * dx + dy * dy);
}

int main() {
    CarMessage old = {0, {10, 0}, 15};
}
