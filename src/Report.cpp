#include <Report.hpp>


Report::Report(){};

Report::Report(ezC2X::Cam cam, MetaData meta){
    //Move relevant info from cam to readablecam
    cam_.generationDeltaTime = cam.payload().generation_time();

    //Save metadata
    metaData_ = meta;
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

