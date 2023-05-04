#include <Report.hpp>
#include <map>
#include <vector>

//This should be updated when the testing determines reasonable bounds
const int LOWER_BOUND = 0;
const int UPPER_BOUND = 100;

std::vector<Report> misbehaving;
std::map<size_t, std::vector<Report>> messages;


double distance(std::tuple<int> pos1, std::tuple<int> pos2);

bool detect_misbehavior(std::vector<Report> reports);

void misbehaving_msgs();

void collect_messages(std::vector<Report> reports);

void collect_single_msg(Report report);

void dump_file ();
