#include <network_ids.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

double distance(std::tuple<int> pos1, std::tuple<int> pos2) {
    return sqrt(pow((std::get<0>(pos2) - std::get<0>(pos1)), 2) + 
                pow((std::get<1>(pos2) - std::get<1>(pos1)), 2));
}

bool detect_misbehavior(std::vector<Report> reports) {
    for (auto report : reports) {
        double dist = distance(msg.getMetaData().positionOnReceieve, msg.getCam().pos);
        double transfer_time = msg.getMetaData().timeOnReceive - msg.getCam().generationDeltaTime;
        if (LOWER_BOUND <= dist / transfer_time && dist / transfer_time <= UPPER_BOUND) {
            continue;
        }
        return true;
    }
    return false;
}

//Needs updating to accomadate for reports...
void misbehaving_msgs() {
    /*Function responsible for going through all messages and adding misbehaving ones to a list*/
    for (auto& [k, v] : messages) {
        if (detect_misbehavior(v)) {
            misbehaving.push_back(k);
        }
    }
}

void collect_messages(std::vector<Report> reports) {
    for (auto report : reports){
        messages[report.getCam().fingerprint].push_back(report);
    }
}

void collect_single_msg(Report report) {
    messages[report.getCam().fingerprint].push_back(report);
}

void dump_file () {
    ofstream myfile;
    std::string file_name = "dump_" + std::to_string(std::chrono::system_clock::now());
    myfile.open(file_name);
    for(auto& [k, v]: messages) {
        for(auto report: v) {
            myfile << (report.concatenateValues() + "\n")
        }
    }
    myfile.close()
}
