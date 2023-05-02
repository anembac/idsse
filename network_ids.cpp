#include <iostream>
#include <vector>
#include <map>
#include <cmath>

const int LOWER_BOUND = 0;
const int UPPER_BOUND = 100;

struct Message {
    std::vector<int> receiver_position;
    std::vector<int> sender_position;
    double msg_arrival_time;
    double timestamp;
    std::string fingerprint;
};

std::map<std::string, std::vector<Message>> messages;
std::vector<std::string> misbehaving;

double distance (std::vector<int> pos1, std::vector<int> pos2) {
    /*Returns the distance between two positions*/
    return sqrt(pow((pos2[0] - pos1[0]),2) + pow((pos2[1]-pos1[1]),2));
}

bool detect_misbehavior(std::vector<Message> msg_set) {
    /*Function for determining if a message is suspicious*/
    for (auto msg : msg_set) {
        double dist = distance(msg.receiver_position, msg.sender_position);
        double transfer_time = msg.msg_arrival_time - msg.timestamp;
        if (LOWER_BOUND <= dist/transfer_time && dist/transfer_time <= UPPER_BOUND) {
            continue;
        }
        return true;
    }
    return false;
}

void misbehaving_msgs() {
    /*Function responsible for going through all messages and adding misbehaving ones to a list*/
    for (auto [key, value] : messages) {
        if (detect_misbehavior(value)) {
            misbehaving.push_back(key);
        }
    }
}

void collect_messages(std::vector<Message> collection) {
    /*A method for collecting messages*/
    for (auto msg : collection) {
        std::string fingerprint = msg.fingerprint;
        if (messages.count(fingerprint)) {
            messages[fingerprint].push_back(msg);
        } else {
            messages.insert({fingerprint, {msg}});
        }
    }
}

int main() {
    /*A main method responsible for handling Intrusion Detection on network level*/
    return 0;
}
