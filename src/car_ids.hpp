#include <Report.hpp>
#include <map>
#include <vector>

const int PLAUSABILITY_RANGE = 500;
const int MAX_SPEED = 20;
const double MAX_ACC = 2.6;
const double MAX_DEC = -4.5;

hash_map<uint32_t, std::vector<Report>> msg_stacks;
std::vector<Report>> misbehaved;

bool range_plausability(Report msg);

bool speed_plausability(Report msg);

bool position_consistency(Report old_msg, Report new_msg, double time, double max_acc, double max_dec);

bool speed_consistency(double speed_diff, double time);

bool cmp_msg_consistency(Report old_msg, Report new_msg);

bool car_ids(Report msg_latest);

double distance(std::tuple<int> pos1, std::tuple<int> pos2);
