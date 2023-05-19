#include <RouteDecider.hpp>
#include <vector>

RouteDecider::RouteDecider(){}

RouteDecider::~RouteDecider() {}

/* Might need extra funcitonality, currently does
not handle if cars have speed zero and speeding up to catch up...*/
double 
RouteDecider::new_speed(double mypos_x, double mypos_y, double speed, uint64_t time){
    
    if(time!=0){clear_old_reports(time);}
    double x_diff = 100;
    double x; //x position of other car - to simplfy not needing to fetch CAM info multiple times
    double y; //y position of other car - to simplfy not needing to fetch CAM info multiple times
        if(mypos_y < YPOS_BELOW){
            //we are driving on sideroad
            for(auto& msg :latest_msgs){
                x = std::get<0>(msg.second.getCam().pos);
                y = std::get<1>(msg.second.getCam().pos);
                if(y < YPOS_BELOW && x > mypos_x && x_diff > x - mypos_x) {
                    x_diff = x - mypos_x;
                    speed = msg.second.getCam().speed;
                }
            }
        } else {
             //we are driving on mainroad
             for(auto& msg :latest_msgs){
                x = std::get<0>(msg.second.getCam().pos);
                y = std::get<1>(msg.second.getCam().pos);
                if(y > YPOS_BELOW && x > mypos_x && x_diff > x - mypos_x ) {
                    x_diff = x - mypos_x;
                    speed = msg.second.getCam().speed;
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
RouteDecider::continue_on_main(double side_speed, double main_speed){
    double x;
    double y;
    double car_speed;
    for (auto& msg : latest_msgs){
        x = std::get<0>(msg.second.getCam().pos);
        y = std::get<1>(msg.second.getCam().pos);
        car_speed = msg.second.getCam().speed;
        if(XPOS_START < x && x < XPOS_END) {
            if(y < YPOS_BELOW && car_speed < side_speed){ 
                side_speed = car_speed;
            } else if(car_speed < main_speed){ 
                main_speed = car_speed;
            }
        }
    }
    return side_speed * SIDE_ROUTE < main_speed * MAIN_ROUTE;
}

void
RouteDecider::collect_latest(Report report){
    latest_msgs[report.getCam().id] = report;
}

void
RouteDecider::clear_old_reports(uint64_t time){
    std::vector<uint32_t> ids;
    for(auto& msg: latest_msgs){
        if(msg.second.getCam().generationDeltaTime < time - 2000){
            ids.push_back(msg.first);
        } 
    }
    for(auto id : ids){
        latest_msgs.erase(id);
    }
}