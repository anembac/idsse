#include <cmath>
#include <vector>
#include <iostream>

const int PLAUSABILITY_RANGE = 500;
const int MAX_SPEED = 30;
const int MAX_ACC = 5;
const int MAX_DEC = -15;

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

bool position_consistency(CarMessage old, CarMessage new_, double time, int max_acc, int max_dec) {
    double v_diff = new_.speed - old.speed;

    if (v_diff < 0) {
        std::swap(max_acc, max_dec);
    }

    double t_acc = (-max_dec * time + v_diff) / (-(max_dec) + max_acc);
    double t_dec = time - t_acc;

    double v_tmp_acc = old.speed + max_acc * t_acc;
    double v_tmp_dec = old.speed + max_dec * t_dec;

    double bound_upper = (old.speed * t_acc + max_acc * pow(t_acc, 2)) +
                         (v_tmp_acc * t_dec + max_dec * pow(t_dec, 2));
    double bound_lower = (old.speed * t_dec + max_dec * pow(t_dec, 2)) +
                         (v_tmp_dec * t_acc + max_acc * pow(t_acc, 2));

    if (v_diff < 0) {
        std::swap(bound_lower, bound_upper);
    }

    double dist = std::sqrt(std::pow(new_.position.first - old.position.first, 2) +
                            std::pow(new_.position.second - old.position.second, 2));

    std::cout << "Lower: " << bound_lower << ", Upper: " << bound_upper << ", Dist: " << dist << std::endl;

    return bound_lower < dist && dist < bound_upper;
}

bool speed_consistency(double speed_diff, double time) {
    return MAX_DEC * time < speed_diff && speed_diff < MAX_ACC * time;
}

bool cmp_all(CarMessage old, CarMessage new_) {
    double t_diff = new_.timestamp - old.timestamp;

    return range_plausability(old.position, new_.position) &&
           speed_plausability(new_) &&
           position_consistency(old, new_, t_diff, MAX_ACC, MAX_DEC);
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
   
