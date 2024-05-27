#ifndef SELF_DRIVING_CPP_DRIVE_BOT_H
#define SELF_DRIVING_CPP_DRIVE_BOT_H

#include <opencv2/opencv.hpp>

class Car {
private:
    cv::Mat frames;

public:

    Car(cv::Mat frames);
    void Drive(cv::Mat frames);
};

#endif // SELF_DRIVING_CPP_DRIVE_BOT_H
