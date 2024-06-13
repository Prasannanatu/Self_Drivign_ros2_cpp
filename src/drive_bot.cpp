#include "self_driving_cpp/drive_bot.h"
#include "self_driving_cpp/lane_detection.h"
#include "self_driving_cpp/color_segmentation.h"

Car::Car(cv::Mat frames) {
    this->frames = frames;
}

void Car::Drive(cv::Mat frames) {
    // cv::Rect roi(238, 0, 1042 - 238, 640 - 0);  // x, y, width, height
    cv::Rect roi(238, 0, 1042 - 238, 640 - 0);  // x, y, width, height
    cv::Mat img_cropped = frames(roi);
    cv::Mat img_resized;
    cv::resize(img_cropped, img_resized, cv::Size(320, 240));

    src = img_resized;  // Initialize global src with the resized image
    cv::imshow("Original Image", src); // Display the original image
    // createTrackbars();  // Ensure trackbars are created
    DetectLanes(img_resized);
}
