#include <map>
#include <stdint.h>
#include <Report.hpp>
#include <ezC2X/core/logging/Logger.hpp>

class RouteDecider
{
public:
    RouteDecider();

    ~RouteDecider();

    const double SIDE_ROUTE = 598.34;
    const double MAIN_ROUTE = 698.26;
    const int MAX_SPEED = 20;
    const int XPOS_START = -300; 
    const int XPOS_END = 150;    
    const int YPOS_BELOW = 40; 

    std::map<uint32_t, Report> latest_msgs;

    double new_speed(double mypos_x, double mypos_y, double speed, uint64_t time);

    bool continue_on_main(double side_speed, double main_speed);

    void collect_latest(Report report);

    void clear_old_reports(uint64_t time);

private:
    ezC2X::Logger log_;
};