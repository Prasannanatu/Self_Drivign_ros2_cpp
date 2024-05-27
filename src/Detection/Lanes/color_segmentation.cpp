#include "self_driving_cpp/color_segmentation.h"
#include <iostream>


cv::Mat HLS, src;
int Hue_Low = 0, Lit_Low = 225, Sat_Low = 0;
int Hue_Low_Y = 20, Hue_High_Y = 30, Lit_Low_Y = 100, Sat_Low_Y = 100;

cv::Scalar lower_bound(Hue_Low, Lit_Low, Sat_Low);
cv::Scalar upper_bound(180, 255, 255);
cv::Scalar lower_bound_yellow(Hue_Low_Y, Lit_Low_Y, Sat_Low_Y);
cv::Scalar upper_bound_yellow(Hue_High_Y, 255, 255);

void OnTrackbarChange(int, void*);

void MaskExtract() {
    if (src.empty()) {
        std::cerr << "Source image is empty!" << std::endl;
        return;
    }

    cv::Mat Mask;
    cv::Mat YellowMask;

    // Apply color segmentation
    ClrSegment(src, Mask, lower_bound, upper_bound);
    ClrSegment(src, YellowMask, lower_bound_yellow, upper_bound_yellow);

    // Create mask for yellow
    cv::Mat mask_Y_ = YellowMask != 0;
    cv::Mat dst_Y = cv::Mat::zeros(src.size(), src.type());
    src.copyTo(dst_Y, mask_Y_);

    // Create mask for white
    cv::Mat mask_ = Mask != 0;
    cv::Mat dst = cv::Mat::zeros(src.size(), src.type());
    src.copyTo(dst, mask_);

    // Display results
    cv::imshow("White Region", dst);
    cv::imshow("Yellow Region", dst_Y);
    cv::waitKey(1);
}

cv::Mat ClrSegment(cv::Mat& input_image, cv::Mat& output_image, cv::Scalar lower_bound, cv::Scalar upper_bound) {
    cv::Mat hsv_image;
    cv::Mat dilated_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_image, lower_bound, upper_bound, dilated_image);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(dilated_image, output_image, cv::MORPH_DILATE, kernel);
    return dilated_image;
}

void SegmentLanes(cv::Mat& input_image, int minArea) {
    if (input_image.empty()) {
        std::cerr << "Input image is empty!" << std::endl;
        return;
    }

    cv::Mat Mask;
    cv::Mat YellowMask;
    ClrSegment(input_image, Mask, lower_bound, upper_bound);
    ClrSegment(input_image, YellowMask, lower_bound_yellow, upper_bound_yellow);

    cv::imshow("White Region", Mask);
    cv::imshow("Yellow Region", YellowMask);
    cv::waitKey(1);
}

void OnTrackbarChange(int, void*) {
    lower_bound = cv::Scalar(Hue_Low, Lit_Low, Sat_Low);
    upper_bound = cv::Scalar(180, 255, 255);
    lower_bound_yellow = cv::Scalar(Hue_Low_Y, Lit_Low_Y, Sat_Low_Y);
    upper_bound_yellow = cv::Scalar(Hue_High_Y, 255, 255);

    MaskExtract();
}

void createTrackbars() {
    std::cout << "Creating trackbars..." << std::endl;

    cv::namedWindow("White Region");
    cv::createTrackbar("Hue_L", "White Region", &Hue_Low, 180, OnTrackbarChange);
    cv::createTrackbar("Lit_L", "White Region", &Lit_Low, 255, OnTrackbarChange);
    cv::createTrackbar("Sat_L", "White Region", &Sat_Low, 255, OnTrackbarChange);

    cv::namedWindow("Yellow Region");
    cv::createTrackbar("Hue_L_Y", "Yellow Region", &Hue_Low_Y, 180, OnTrackbarChange);
    cv::createTrackbar("Hue_H_Y", "Yellow Region", &Hue_High_Y, 180, OnTrackbarChange);
    cv::createTrackbar("Lit_L_Y", "Yellow Region", &Lit_Low_Y, 255, OnTrackbarChange);
    cv::createTrackbar("Sat_L_Y", "Yellow Region", &Sat_Low_Y, 255, OnTrackbarChange);
}





// cv::Mat HLS, src;
// int Hue_Low = 0, Lit_Low = 225, Sat_Low = 0;
// int Hue_Low_Y = 30, Hue_High_Y = 33, Lit_Low_Y = 120, Sat_Low_Y = 0;
// cv::Scalar lower_bound(Hue_Low, Lit_Low, Sat_Low);
// cv::Scalar upper_bound(180, 255, 255);
// cv::Scalar lower_bound_yellow(Hue_Low_Y, Lit_Low_Y, Sat_Low_Y);
// cv::Scalar upper_bound_yellow(Hue_High_Y, 255, 255);

// void OnTrackbarChange(int, void*);

// void MaskExtract() {
//     if (src.empty()) {
//         std::cerr << "Source image is empty!" << std::endl;
//         return;
//     }

//     cv::Mat Mask;
//     cv::Mat YellowMask;

//     // Apply color segmentation
//     ClrSegment(src, Mask, lower_bound, upper_bound);
//     ClrSegment(src, YellowMask, lower_bound_yellow, upper_bound_yellow);

//     // Create mask for yellow
//     cv::Mat mask_Y_ = YellowMask != 0;
//     cv::Mat dst_Y = cv::Mat::zeros(src.size(), src.type());
//     src.copyTo(dst_Y, mask_Y_);

//     // Create mask for white
//     cv::Mat mask_ = Mask != 0;
//     cv::Mat dst = cv::Mat::zeros(src.size(), src.type());
//     src.copyTo(dst, mask_);

//     // Display results
//     cv::imshow("White Region", dst);
//     cv::imshow("Yellow Region", dst_Y);
//     cv::waitKey(1);
// }

// cv::Mat ClrSegment(cv::Mat& input_image, cv::Mat& output_image, cv::Scalar lower_bound, cv::Scalar upper_bound) {
//     cv::Mat hsv_image;
//     cv::Mat dilated_image;
//     cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
//     cv::inRange(hsv_image, lower_bound, upper_bound, dilated_image);
//     cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
//     cv::morphologyEx(dilated_image, output_image, cv::MORPH_DILATE, kernel);
//     return dilated_image;
// }

// void SegmentLanes(cv::Mat& input_image, int minArea) {
//     if (input_image.empty()) {
//         std::cerr << "Input image is empty!" << std::endl;
//         return;
//     }

//     cv::Mat Mask;
//     cv::Mat YellowMask;
//     ClrSegment(input_image, Mask, lower_bound, upper_bound);
//     ClrSegment(input_image, YellowMask, lower_bound_yellow, upper_bound_yellow);

//     // createTrackbars();
//     cv::imshow("White Region", Mask);
//     cv::imshow("Yellow Region", YellowMask);
//     cv::waitKey(1);
// }

// void OnTrackbarChange(int, void*) {
//     lower_bound = cv::Scalar(Hue_Low, Lit_Low, Sat_Low);
//     upper_bound = cv::Scalar(255, 255, 255);
//     lower_bound_yellow = cv::Scalar(Hue_Low_Y, Lit_Low_Y, Sat_Low_Y);
//     upper_bound_yellow = cv::Scalar(Hue_High_Y, 255, 255);

//     MaskExtract();
// }

// void createTrackbars() {
//     std::cout << "Creating trackbars..." << std::endl;

//     cv::namedWindow("White Region");
//     cv::createTrackbar("Hue_L", "White Region", &Hue_Low, 255, OnTrackbarChange);
//     cv::createTrackbar("Lit_L", "White Region", &Lit_Low, 255, OnTrackbarChange);
//     cv::createTrackbar("Sat_L", "White Region", &Sat_Low, 255, OnTrackbarChange);

//     cv::namedWindow("Yellow Region");
//     cv::createTrackbar("Hue_L_Y", "Yellow Region", &Hue_Low_Y, 255, OnTrackbarChange);
//     cv::createTrackbar("Hue_H_Y", "Yellow Region",&Hue_High_Y,255, OnTrackbarChange);
//     cv::createTrackbar("Lit_L_Y", "Yellow Region", &Lit_Low_Y, 255, OnTrackbarChange);
//     cv::createTrackbar("Sat_L_Y", "Yellow Region", &Sat_Low_Y, 255, OnTrackbarChange);
// }


