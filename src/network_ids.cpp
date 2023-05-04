#include <network_ids.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>

double distance(std::tuple<double,double> pos1, std::tuple<double,double> pos2) {
    return sqrt(pow((std::get<0>(pos2) - std::get<0>(pos1)), 2) + 
                pow((std::get<1>(pos2) - std::get<1>(pos1)), 2));
}

bool detect_misbehavior(std::vector<Report> reports) {
    for (auto report : reports) {
        double dist = distance(report.getMetaData().positionOnReceieve, report.getCam().pos);
        double transfer_time = report.getMetaData().timeOnReceive - report.getCam().generationDeltaTime;
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
    for (auto& x : messages) {
        if (detect_misbehavior(x.second)) {
            misbehaving.push_back(x.first);
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
    std::ofstream myfile;
    std::string file_name = "net_dump_" + std::to_string(std::chrono::system_clock::to_time_t((std::chrono::system_clock::now())));
    myfile.open(file_name);
    for(auto& x: messages) {
        for(auto report: x.second) {
            myfile << (report.concatenateValues() + "\n");
        }
    }
    myfile.close();
}
