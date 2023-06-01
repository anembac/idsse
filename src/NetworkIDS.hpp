#include <Report.hpp>
#include <map>
#include <vector>

//This should be updated when the testing determines reasonable bounds
const int LOWER_BOUND = 0;
const int UPPER_BOUND = 100;

std::vector<size_t> misbehaving;
std::map<size_t, std::vector<Report>> messages;


double distance(std::tuple<double,double> pos1, std::tuple<double,double> pos2);

bool detectMisbehavior(std::vector<Report> reports);

void misbehavingMsgs();

void collectMessages(std::vector<Report> reports);

void collectSingleMsg(Report report);

void saveToFile ();
