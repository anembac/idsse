#include <map>
#include <stdint.h>
#include <Report.hpp>
#include <EgoPos.hpp>
#include <ezC2X/core/logging/Logger.hpp>

class RouteDecider
{
public:
    RouteDecider();

    ~RouteDecider();

    const double MAIN_ROUTE = 598.34;
    const double SIDE_ROUTE = 698.26;
    const int MAX_SPEED = 20;
    const int XPOS_START = -300; 
    const int XPOS_END = 150;    
    const double YPOS_BELOW = 0; 

    std::map<uint32_t, Report> latest_msgs;

    double newSpeed(EgoPos myPos, double speed, uint64_t time);

    bool continueOnMain(double side_speed, double main_speed);

    void collectLatest(Report report);

    void clearOldReports(uint64_t time);

private:
    ezC2X::Logger log_;
};