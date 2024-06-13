#include "self_driving_cpp/color_segmentation.h"
#include <iostream>
#include <tuple>
#include "self_driving_cpp/morphology.h"
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

cv::Mat HLS, src;
int Hue_Low = 0, Lit_Low = 225, Sat_Low = 0;
int Hue_Low_Y = 30, Hue_High_Y = 46, Lit_Low_Y = 160, Sat_Low_Y = 0;

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
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HLS_FULL);
    cv::inRange(hsv_image, lower_bound, upper_bound, dilated_image);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(dilated_image, output_image, cv::MORPH_DILATE, kernel);
    return dilated_image;
}

void SegmentLanes(cv::Mat& input_image, int /*minArea*/) {
    if (input_image.empty()) {
        std::cerr << "Input image is empty!" << std::endl;
        return;
    }

    cv::Mat Mask;
    cv::Mat YellowMask;
    ClrSegment(input_image, Mask, lower_bound, upper_bound);
    ClrSegment(input_image, YellowMask, lower_bound_yellow, upper_bound_yellow);
    cv::Mat WhiteyMask = Mask.clone();
    cv::Mat temp_image = input_image.clone();
    cv::imshow("White output mask for mid lanes input", Mask);
    cv::waitKey(1);

    std::tuple<cv::Mat, cv::Mat> midresult = SegmentMidLanes(temp_image, WhiteyMask, 0);
}

void OnTrackbarChange(int, void*) {
    lower_bound = cv::Scalar(Hue_Low, Lit_Low, Sat_Low);
    upper_bound = cv::Scalar(180, 255, 255);
    lower_bound_yellow = cv::Scalar(Hue_Low_Y, Lit_Low_Y, Sat_Low_Y);
    upper_bound_yellow = cv::Scalar(Hue_High_Y, 255, 255);

    MaskExtract();
}

std::pair<cv::Mat, cv::Mat> GetMask_and_Edge_for_largerObject(cv::Mat& input_image, cv::Mat& Mask, int minArea) {
    cv::Mat Gray;
    cv::Mat MaskLargerObject;

    std::cout << "Initial Mask size: " << Mask.size() << ", type: " << Mask.type() << std::endl;
    std::cout << "Initial Input image size: " << input_image.size() << ", type: " << input_image.type() << std::endl;

    // Ensure the sizes are the same
    if (Mask.size() != input_image.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Mask and input image sizes do not match!");
        return std::make_pair(cv::Mat(), cv::Mat());
    }

    // Ensure the mask is single-channel
    if (Mask.channels() != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Mask is not single-channel!");
        return std::make_pair(cv::Mat(), cv::Mat());
    }

    // Convert Mask to the same number of channels as input_image
    cv::Mat temp_mask;
    cv::cvtColor(Mask, temp_mask, cv::COLOR_GRAY2BGR);
    std::cout << "Converted Mask size: " << temp_mask.size() << ", type: " << temp_mask.type() << std::endl;

    // Ensure the types are the same
    if (temp_mask.type() != input_image.type()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Mask and input image types do not match: Mask type = %d, input image type = %d",
                     temp_mask.type(), input_image.type());
        temp_mask.convertTo(temp_mask, input_image.type());
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Converted Mask type to match input image type: %d", temp_mask.type());
    }

    std::cout << "Before bitwise_and: Input image size: " << input_image.size() << ", type: " << input_image.type() << std::endl;
    std::cout << "Before bitwise_and: temp_mask size: " << temp_mask.size() << ", type: " << temp_mask.type() << std::endl;

    // Perform bitwise_and operation
    cv::bitwise_and(input_image, temp_mask, input_image);

    std::cout << "After bitwise_and: Input image size: " << input_image.size() << ", type: " << input_image.type() << std::endl;

    // Convert input_image to grayscale
    cv::cvtColor(input_image, Gray, cv::COLOR_BGR2GRAY);

    std::cout << "After cvtColor: Gray size: " << Gray.size() << ", type: " << Gray.type() << std::endl;

    // Apply BwareaOpen (assuming this function is defined elsewhere)
    BwareaOpen(Gray, MaskLargerObject, minArea);
    std::cout << "After BwareaOpen: MaskLargerObject size: " << MaskLargerObject.size() << ", type: " << MaskLargerObject.type() << std::endl;

    // Perform bitwise_and operation with the larger object mask
    cv::bitwise_and(Gray, MaskLargerObject, Gray);
    std::cout << "After bitwise_and with MaskLargerObject: Gray size: " << Gray.size() << ", type: " << Gray.type() << std::endl;

    // Apply Gaussian blur
    cv::GaussianBlur(Gray, Gray, cv::Size(11, 11), 1);
    std::cout << "After GaussianBlur: Gray size: " << Gray.size() << ", type: " << Gray.type() << std::endl;

    // Perform Canny edge detection
    cv::Canny(Gray, Gray, 50, 150, 3);
    std::cout << "After Canny: Gray size: " << Gray.size() << ", type: " << Gray.type() << std::endl;

    // Display the result
    cv::imshow("Gray", Gray);
    cv::waitKey(1);

    return std::make_pair(MaskLargerObject, Gray);
}


std::tuple<cv::Mat, cv::Mat> SegmentMidLanes(cv::Mat& input_image, cv::Mat& Mask, int /*minArea*/) {
    cv::Mat MidLaneMask, MidLaneEdge;

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mask size: %dx%d, type: %d, Input image size: %dx%d, type: %d", 
    //             Mask.size().width, Mask.size().height, Mask.type(),
    //             input_image.size().width, input_image.size().height, input_image.type());

    // if (Mask.size() != input_image.size()) {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Mask and input image sizes do not match!");
    //     return std::make_tuple(cv::Mat(), cv::Mat());
    // }

    // Ensure the types are the same
    if (Mask.type() != input_image.type()) {
        // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Mask and input image types do not match: Mask type = %d, input image type = %d",
        //              Mask.type(), input_image.type());
        Mask.convertTo(Mask, input_image.type());
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Converted Mask type to match input image type: %d", Mask.type());
    }

    cv::imshow("input_image_segment_midlane", input_image);
    cv::imshow("mask for white in midlane function", Mask);
    cv::waitKey(1);

    // // Uncomment these lines one by one to identify the problematic operation
    std::pair<cv::Mat, cv::Mat> result = GetMask_and_Edge_for_largerObject(input_image, Mask, 0);
    // MidLaneMask = result.first;
    // MidLaneEdge = result.second;

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MidLaneMask size: %dx%d, type: %d, MidLaneEdge size: %dx%d, type: %d", 
    //             MidLaneMask.size().width, MidLaneMask.size().height, MidLaneMask.type(),
    //             MidLaneEdge.size().width, MidLaneEdge.size().heights, MidLaneEdge.type());

    // return std::make_tuple(MidLaneEdge, MidLaneMask);
    return std::make_tuple(cv::Mat(), cv::Mat());
}

std::tuple<cv::Mat, cv::Mat, std::vector<cv::Point>> SegmentOuterLanes(cv::Mat& input_image, cv::Mat& Mask, int minArea) {
    cv::Mat OuterLaneMask, OuterLaneEdge;
    std::vector<cv::Point> lowestedgepoints;
    cv::Mat lanesideseperated;
    cv::imshow("mask for yellow", Mask);
    cv::waitKey(1);

    std::pair<cv::Mat, cv::Mat> result = GetMask_and_Edge_for_largerObject(input_image, Mask, minArea);
    OuterLaneMask = result.first;
    OuterLaneEdge = result.second;

    std::pair<cv::Mat, bool> result_ = RegisterLargestContourOuterLane(OuterLaneMask, minArea);
    bool largest_contour_found = result_.second;
    OuterLaneMask = result_.first;

    if (largest_contour_found) {
        cv::bitwise_and(OuterLaneEdge, OuterLaneMask, OuterLaneEdge);
        cv::imshow("Outer lane Edge", OuterLaneEdge);
        std::pair<cv::Mat, std::vector<cv::Point>> result = RegisterLowestEdgePoints(OuterLaneEdge);
        cv::imshow("lane side seperated", result.first);
        lanesideseperated = result.first;
        lowestedgepoints = result.second;
    } else {
        lanesideseperated = cv::Mat::zeros(input_image.size(), input_image.type());
    }

    return std::make_tuple(OuterLaneEdge, lanesideseperated, lowestedgepoints);
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














//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// #include "self_driving_cpp/color_segmentation.h"
// #include <iostream>
// #include <tuple>
// #include "self_driving_cpp/morphology.h"
// #include <rclcpp/rclcpp.hpp>


// cv::Mat HLS, src;
// int Hue_Low = 0, Lit_Low = 225, Sat_Low = 0;
// int Hue_Low_Y = 30, Hue_High_Y = 46, Lit_Low_Y = 160, Sat_Low_Y = 0;

// cv::Scalar lower_bound(Hue_Low, Lit_Low, Sat_Low);
// cv::Scalar upper_bound(180, 255, 255);
// cv::Scalar lower_bound_yellow(Hue_Low_Y, Lit_Low_Y, Sat_Low_Y);
// cv::Scalar upper_bound_yellow(Hue_High_Y, 255, 255);

// void OnTrackbarChange(int, void*);

// void MaskExtract()
// {
//     if (src.empty()) 
//     {
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

// cv::Mat ClrSegment(cv::Mat& input_image, cv::Mat& output_image, cv::Scalar lower_bound, cv::Scalar upper_bound) 
// {
//     cv::Mat hsv_image;
//     cv::Mat dilated_image;
//     cv::cvtColor(input_image, hsv_image, cv::ColorConversionCodes::COLOR_BGR2HLS_FULL);
//     cv::inRange(hsv_image, lower_bound, upper_bound, dilated_image);
//     cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
//     cv::morphologyEx(dilated_image, output_image, cv::MORPH_DILATE, kernel);
//     return dilated_image;
// }

// // std::tuple<std::tuple<cv::Mat ,cv::Mat>, std::tuple<cv::Mat ,cv::Mat, std::vector<cv::Point>>> SegmentLanes(cv::Mat& input_image, int minArea) {
// void SegmentLanes(cv::Mat& input_image, int minArea) 
// {
//     if (input_image.empty()) 
//     {
//         std::cerr << "Input image is empty!" << std::endl;
//     }
//     cv::Mat Mask;
//     cv::Mat YellowMask;
//     cv::Mat print;
//     // createTrackbars();
//     ClrSegment(input_image, Mask, lower_bound, upper_bound);
//     ClrSegment(input_image, YellowMask, lower_bound_yellow, upper_bound_yellow);


//     cv::imshow("White output mask for mid lanes input", Mask);
//     cv::waitKey(1);
//     // cv::Mat new_src = input_image;
 
//     // cv::bitwise_and(input_image, Mask, print);
//     // cv::imshow ("mask for white", print);
//     // cv::waitKey(1);
//     std::tuple<cv::Mat ,cv::Mat>midresult = SegmentMidLanes(input_image, Mask, minArea);
//     // std::tuple<cv::Mat ,cv::Mat, std::vector<cv::Point>> results = SegmentOuterLanes(new_src, YellowMask, minArea);
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "midresults", std::get<0>((midresult)).size());
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "results", std::get<0>((results)).size());
//     // cv::imshow("Outer lane mask", std::get<0>((results)));
//     // cv::imshow("Outer lane Edge", std::get<1>((results)));
//     // cv::imshow("White Region", Mask);
//     // cv::imshow("Yellow Region", YellowMask);
//     // cv::waitKey(1);
//     // cv::imshow("Mid lane mask", std::get<0>((midresult)));
//     // cv::imshow("Mid lane Edge", std::get<1>((midresult)));
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "successfull imshow calls");

//     // cv::waitKey(1);
//     // return {midresult, results};
// }

// void OnTrackbarChange(int, void*);
//     // return {midresult, results};
// void OnTrackbarChange(int, void*) 
// {
//     lower_bound = cv::Scalar(Hue_Low, Lit_Low, Sat_Low);
//     upper_bound = cv::Scalar(180, 255, 255);
//     lower_bound_yellow = cv::Scalar(Hue_Low_Y, Lit_Low_Y, Sat_Low_Y);
//     upper_bound_yellow = cv::Scalar(Hue_High_Y, 255, 255);

//     MaskExtract();
// }

// std::pair<cv::Mat ,cv::Mat> GetMask_and_Edge_for_largerObject(cv::Mat& input_image, cv::Mat& Mask, int minArea)
// {
//     // RCL 
//     // cv::Mat ip = input_image.clone();

//     cv::Mat Gray;
//     cv::Mat MaskLargerObject;
//     cv::Mat temp_image = input_image.clone();
//     std::cout << Mask.size() << std::endl;
//     std::cout << input_image.size() << std::endl;
    
//     // cv::bitwise_and(temp_image, Mask, temp_image);
//     // cv::cvtColor(temp_image,Gray, cv::COLOR_BGR2GRAY);
//     cv::imshow("Gray", temp_image);
//     cv::waitKey(1);
// //     BwareaOpen(Gray, MaskLargerObject, minArea);
// //     cv::bitwise_and(Gray, MaskLargerObject, Gray);
// //     cv::GaussianBlur(Gray, Gray, cv::Size(11, 11), 1);
// //     cv::Canny(Gray, Gray, 50, 150,3);
// //     // cv::imshow("Gray", Gray);
// //     cv::waitKey(1);    
// //     // return std::make_pair(cv::Mat(), Gray);
// //     return std::make_pair(MaskLargerObject, Gray);
// }

// std::tuple<cv::Mat ,cv::Mat> SegmentMidLanes(cv::Mat& input_image, cv::Mat& Mask, int minArea)
// {
//     cv::Mat MidLaneMask, MidLaneEdge;
//     // cv::imshow("input_image_segment_midlane", input_image);
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), (std::to_string(Mask.size().width) + "x" + std::to_string(Mask.size().height)).c_str());
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), (std::to_string(input_image.size().width) + "x" + std::to_string(input_image.size().height)).c_str());
    
//     // cv::waitKey(1);
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inside SegmentMidLanes");
//     std::pair<cv::Mat, cv::Mat> result = GetMask_and_Edge_for_largerObject(input_image, Mask, minArea);
//     // MidLaneMask = result.first;
//     // MidLaneEdge = result.second;
//     return std::make_tuple(cv::Mat(), cv::Mat());
//     // return std::make_tuple(MidLaneEdge, MidLaneMask);
// }


// std::tuple<cv::Mat ,cv::Mat, std::vector<cv::Point>> SegmentOuterLanes(cv::Mat& input_image, cv::Mat& Mask, int minArea)
// {
//     cv::Mat OuterLaneMask, OuterLaneEdge;
//     std::vector<cv::Point> lowestedgepoints;
//     cv::Mat lanesideseperated;
//     cv::imshow ("mask for yellow", Mask);
//     cv::waitKey(1);
//     std::pair<cv::Mat, cv::Mat> result = GetMask_and_Edge_for_largerObject(input_image, Mask, minArea);
//     OuterLaneMask = result.first;
//     OuterLaneEdge = result.second;
//     std::pair<cv::Mat, bool> result_ = RegisterLargestContourOuterLane(OuterLaneMask, minArea);
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result_.second: %s", result_.second ? "true" : "false");
//     // std::pair<cv::Mat, bool> result_ = RegisterLargestContourOuterLane(OuterLaneMask, minArea);
//     // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result_.second", &result_.second);
//     bool largest_contour_found = result_.second;
//     OuterLaneMask = result_.first;

//     if (largest_contour_found)
//     {
//         cv::bitwise_and(OuterLaneEdge, OuterLaneMask, OuterLaneEdge);
//         cv::imshow("Outer lane Edge", OuterLaneEdge);
//         std::pair< cv::Mat, std::vector<cv::Point>> result =  RegisterLowestEdgePoints(OuterLaneEdge);
//         cv::imshow("lane side seperated", result.first);
//         lanesideseperated = result.first;
//         lowestedgepoints = result.second;
//     }
//     else
//     {
//         lanesideseperated = cv::Mat::zeros(input_image.size(), input_image.type());
//     }
//     return std::make_tuple(OuterLaneEdge, lanesideseperated, lowestedgepoints);
// }





// void createTrackbars() 
// {
//     std::cout << "Creating trackbars..." << std::endl;

//     cv::namedWindow("White Region");
//     cv::createTrackbar("Hue_L", "White Region", &Hue_Low, 180, OnTrackbarChange);
//     cv::createTrackbar("Lit_L", "White Region", &Lit_Low, 255, OnTrackbarChange);
//     cv::createTrackbar("Sat_L", "White Region", &Sat_Low, 255, OnTrackbarChange);

//     cv::namedWindow("Yellow Region");
//     cv::createTrackbar("Hue_L_Y", "Yellow Region", &Hue_Low_Y, 180, OnTrackbarChange);
//     cv::createTrackbar("Hue_H_Y", "Yellow Region", &Hue_High_Y, 180, OnTrackbarChange);
//     cv::createTrackbar("Lit_L_Y", "Yellow Region", &Lit_Low_Y, 255, OnTrackbarChange);
//     cv::createTrackbar("Sat_L_Y", "Yellow Region", &Sat_Low_Y, 255, OnTrackbarChange);
// }

  