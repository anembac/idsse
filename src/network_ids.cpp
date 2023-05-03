#include <cmath>
#include <iostream>
#include <map>
#include <vector>
#include <Report.hpp>

const int LOWER_BOUND = 0;
const int UPPER_BOUND = 100;

struct Message
{
    std::vector<int> receiver_position;
    std::vector<int> sender_position;
    double msg_arrival_time;
    double timestamp;
    std::string fingerprint;
};

std::map<std::string, std::vector<Message>> messages;
std::vector<std::string> misbehaving;

double
distance(std::tuple<int> pos1, std::tuple<int> pos2)
{
    /*Returns the distance between two positions*/
    return sqrt(pow((std::get<0>(pos2) - std::get<0>(pos1)), 2) + 
                pow((std::get<1>(pos2) - std::get<1>(pos1)), 2));
}

bool
detect_misbehavior(std::vector<Report> reports)
{
    /*Function for determining if a message is suspicious*/
    for (auto report : reports)
    {
        double dist = distance(msg.getMetaData().positionOnReceieve, msg.getCam().pos);
        double transfer_time = msg.getMetaData().timeOnReceive - msg.getCam().generationDeltaTime;
        if (LOWER_BOUND <= dist / transfer_time && dist / transfer_time <= UPPER_BOUND)
        {
            continue;
        }
        return true;
    }
    return false;
}


//Needs updating to accomadate for reports...
void
misbehaving_msgs()
{
    /*Function responsible for going through all messages and adding misbehaving ones to a list*/
    for (auto& [k, v] : messages)
    {
        if (detect_misbehavior(v))
        {
            misbehaving.push_back(k);
        }
    }
}

void
collect_messages(std::vector<Message> collection)
{
    /*A method for collecting messages*/
    for (auto msg : collection)
    {
        std::string fingerprint = msg.fingerprint;
        if (messages.count(fingerprint))
        {
            messages[fingerprint].push_back(msg);
        }
        else
        {
            messages.insert({fingerprint, {msg}});
        }
    }
}

int
main()
{
    /*A main method responsible for handling Intrusion Detection on network level*/
    return 0;
}
