#include <Report.hpp>

//Check through whether all of these are needed
#include <iostream>
#include <tuple>
#include <string>
#include <functional>

Report::Report(){};

Report::Report(ezC2X::Cam cam, MetaData meta){
    //Move relevant info from cam to readablecam
    cam_.generationDeltaTime = cam.payload().generation_time();

    //Save metadata
    metaData_ = meta;
}

// Define the hash function for the struct
  void fingerprint (const ReadableCam& rc){
    string hash_val = "";

    // Hash the tuple of doubles
    hash_val += std::to_string(hash<tuple<double, double>>()(rc.pos));

    // Hash the double speed
    hash_val += std::to_string(<double>()(rc.speed));

    // Hash the string id
    hash_val += std::to_string(hash<string>()(rc.id));

    // Hash the integer time in milliseconds
    hash_val += std::to_string(hash<int>()(rc.generationDeltaTime));

    rc->fingerprint = std::stoull(hash_val);
}

Report::~Report(){};

ReadableCam
Report::getCam(){
    return cam_;
};

MetaData
Report::getMetaData(){
    return metaData_;
};

