#include <RouteDecider.hpp>
#include <vector>
#include "ezC2X/core/geographic/Distance.hpp"


RouteDecider::RouteDecider(): log_("routeDecider"){}

RouteDecider::~RouteDecider() {}

/* Might need extra funcitonality, currently does
not handle if cars have speed zero and speeding up to catch up...*/
double 
RouteDecider::newSpeed(EgoPos myPos, double speed, uint64_t time){
    log_.info() << "Calculating new speed";
    clearOldReports(time);
    double x_diff = 2000;
    double lat; //longitude position of other car - to simplfy not needing to fetch CAM info multiple times
    double lon; //latitude position of other car - to simplfy not needing to fetch CAM info multiple times
    double y; //y position of other car - to simplfy not needing to fetch CAM info multiple times
    double dist;
        if(myPos.cartPos.y < YPOS_BELOW){
            //we are driving on sideroad
            for(auto& msg :latest_msgs){
                lat = msg.second.getCam().pos.wgsPos.getLatitude().value();
                lon = msg.second.getCam().pos.wgsPos.getLongitude().value();
                y = msg.second.getCam().pos.cartPos.y;
                dist = ezC2X::distance(myPos.wgsPos, ezC2X::Wgs84Position::wrap(lat,lon));
                log_.info() << "Distance: " << dist;
                if(y < YPOS_BELOW 
                    && lon > (myPos.wgsPos.getLongitude().value()) 
                    && x_diff > dist
                    && msg.second.getCam().speed < speed) 
                {
                    //x_diff = x - mypos_x;
                    speed = msg.second.getCam().speed;
                    log_.info() << "sideroad updating speed to " << speed;
                }
            }
        } else {
             //we are driving on mainroad
             for(auto& msg :latest_msgs){
                lat = msg.second.getCam().pos.wgsPos.getLatitude().value();
                lon = msg.second.getCam().pos.wgsPos.getLongitude().value();
                y = msg.second.getCam().pos.cartPos.y;
                dist = ezC2X::distance(myPos.wgsPos, ezC2X::Wgs84Position::wrap(lat,lon));
                if(y >= YPOS_BELOW 
                    && lon > (myPos.wgsPos.getLongitude().value()) 
                    && x_diff > dist
                    && msg.second.getCam().speed < speed) {
                    //x_diff = x - mypos_x;
                    speed = msg.second.getCam().speed;
                    log_.info() << "mainroad updating speed to " << speed;
                }
             }
        }
    return speed;
}
/* This is called once, when our position is between (-430,50) and (-330,50)
 * then we have a bool which in the class calling this function which keeps track if route has been
 * decided 
 */
bool
RouteDecider::continueOnMain(double side_speed, double main_speed){
    double x;
    double y;
    double car_speed;
    for (auto& msg : latest_msgs){
        x = msg.second.getCam().pos.cartPos.x;
        y = msg.second.getCam().pos.cartPos.y;
        car_speed = msg.second.getCam().speed;
        if(XPOS_START < x && x < XPOS_END) {
            if(y < YPOS_BELOW && car_speed < side_speed){ 
                side_speed = car_speed;
            } else if(car_speed < main_speed){ 
                main_speed = car_speed;
            }
        }
    }
    log_.info() << "side_speed: " << side_speed << ", side time: " << (SIDE_ROUTE / side_speed)  << ", main_speed: " << main_speed << ", main time: " << (MAIN_ROUTE / main_speed);
    bool goMain = (SIDE_ROUTE / side_speed) > (MAIN_ROUTE / main_speed);
    std::string pickedRoute = goMain ? "main" : "side";
    log_.info() << "Continuing on: " << pickedRoute;
    return goMain;
}

void
RouteDecider::collectLatest(Report report){
    latest_msgs[report.getCam().id] = report;
}

void
RouteDecider::clearOldReports(uint64_t time){
    std::vector<uint32_t> ids;
    for(auto& msg: latest_msgs){
        log_.info() << "genDeltaT: " << msg.second.getCam().generationDeltaTime << ", t-500: "<< (time-500);
        if(msg.second.getCam().generationDeltaTime < (time - 500)){
            ids.push_back(msg.first);
        } 
    }
    for(auto id : ids){
        latest_msgs.erase(id);
    }
}