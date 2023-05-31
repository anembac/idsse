#include <Report.hpp>
#include <map>
#include <vector>

const int PLAUSABILITY_RANGE = 500;
const int MAX_SPEED = 20;
const double MAX_ACC = 2.6;
const double MAX_DEC = -4.5;

class CarIDS{
public:
    CarIDS();

    ~CarIDS();

    bool rangePlausability(Report msg);

    bool speedPlausability(Report msg);

    bool positionConsistency(Report old_msg, Report new_msg, double time, double max_acc, double max_dec);

    bool speedConsistency(double speed_diff, double time);

    bool compareMsgConsistency(Report old_msg, Report new_msg);

    bool carIDS(Report msg_latest);

    double distance(std::tuple<double,double> pos1, std::tuple<double,double> pos2);

    std::vector<Report> getMisbehavedMessages();

    std::map<uint32_t, std::vector<Report>> getMsgStacks();


private:
    std::map<uint32_t, std::vector<Report>> msg_stacks_;
    std::vector<Report> misbehaved_;
};
