#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

using namespace std;

const double SHORT_ROUTE = 598.34;
const double LONG_ROUTE = 698.26;
const int MAX_SPEED = 20;

struct LatestMessage {
    string edge;
    double speed;
};

vector<string> edges_short = {"main1"};
vector<string> edges_long = {"side1", "side2", "side3"};

bool continue_on_route(vector<LatestMessage> latest, double short_speed, double long_speed)
//Add an if-check for whether they are before at the start so they actually can decide the route
{
    for (auto message : latest)
    {
        if (find(edges_short.begin(), edges_short.end(), message.edge) != edges_short.end() && message.speed < short_speed)
        {
            short_speed = message.speed;
        }
        if (find(edges_long.begin(), edges_long.end(), message.edge) != edges_long.end() && message.speed < long_speed)
        {
            long_speed = message.speed;
        }
    }

    return short_speed * SHORT_ROUTE < long_speed * LONG_ROUTE;
}

int main()
{
    // Example usage of continue_on_route function
    vector<LatestMessage> latest_messages = {{"main1", 15}, {"side1", 10}, {"side2", 20}};
    bool continue_on_short_route = continue_on_route(latest_messages, MAX_SPEED, MAX_SPEED);
    cout << "Continue on short route? " << (continue_on_short_route ? "Yes" : "No") << endl;

    return 0;
}