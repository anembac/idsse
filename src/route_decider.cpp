#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

using namespace std;

const double SHORT_ROUTE = 598.34;
const double LONG_ROUTE = 698.26;
const int MAX_SPEED = 20;

vector<string> edges_short = {"main1"};
vector<string> edges_long = {"side1", "side2", "side3"};

bool continue_on_route(vector<map<string, double>> latest, double short_speed, double long_speed)
{
    for (auto key : latest)
    {
        if (find(edges_short.begin(), edges_short.end(), key["edge"]) != edges_short.end() && key["speed"] < short_speed)
        {
            short_speed = key["speed"];
        }
        if (find(edges_long.begin(), edges_long.end(), key["edge"]) != edges_long.end() && key["speed"] < long_speed)
        {
            long_speed = key["speed"];
        }
    }

    return short_speed * SHORT_ROUTE < long_speed * LONG_ROUTE;
}