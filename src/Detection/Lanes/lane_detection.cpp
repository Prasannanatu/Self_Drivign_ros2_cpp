#include "self_driving_cpp/lane_detection.h"
#include "self_driving_cpp/color_segmentation.h"
#include <rclcpp/rclcpp.hpp>
void DetectLanes(cv::Mat& frames) {
    cv::Rect roi(0, CropHeight_resized, frames.cols, frames.rows - CropHeight_resized);
    cv::Mat cropped_frame = frames(roi);
    // std::tuple<std::tuple<cv::Mat ,cv::Mat>, std::tuple<cv::Mat ,cv::Mat, std::vector<cv::Point>>> final = SegmentLanes(cropped_frame, minArea_resized); 
    SegmentLanes(cropped_frame, minArea_resized);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reached imshow calls");
    // cv::imshow("MId lane mask", std::get<0>(std::get<0>(final)));
    // cv::imshow("MId lane Edge", std::get<1>(std::get<0>(final))); 
    // cv::imshow("Outer lane mask", std::get<0>(std::get<1>(final)));
    // cv::imshow("Outer lane Edge", std::get<1>(std::get<1>(final)));
    // cv::waitKey(1);
}
