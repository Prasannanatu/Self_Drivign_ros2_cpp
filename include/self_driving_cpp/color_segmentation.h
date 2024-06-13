#ifndef COLOR_SEGMENTATION_HPP
#define COLOR_SEGMENTATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>

// Global variables
extern cv::Mat HLS, src;
extern int Hue_Low, Lit_Low, Sat_Low;
extern int Hue_Low_Y, Hue_High_Y, Lit_Low_Y, Sat_Low_Y;
extern cv::Scalar lower_bound, upper_bound;
extern cv::Scalar lower_bound_yellow, upper_bound_yellow;

// Function prototypes
void OnTrackbarChange(int, void*);
void MaskExtract();
void createTrackbars();
cv::Mat ClrSegment(cv::Mat& input_image, cv::Mat& output_image, cv::Scalar lower_bound, cv::Scalar upper_bound);
// std::tuple<std::tuple<cv::Mat ,cv::Mat>, std::tuple<cv::Mat ,cv::Mat, std::vector<cv::Point>>> SegmentLanes(cv::Mat& input_image, int minArea);
void SegmentLanes(cv::Mat& input_image, int minArea);
std::pair<cv::Mat ,cv::Mat> GetMask_and_Edge_for_largerObject(cv::Mat& input_image, cv::Mat& Mask, int minArea);
std::tuple<cv::Mat ,cv::Mat> SegmentMidLanes(cv::Mat& input_image, cv::Mat& Mask, int minArea);
std::tuple<cv::Mat ,cv::Mat, std::vector<cv::Point>> SegmentOuterLanes(cv::Mat& input_image, cv::Mat& Mask, int minArea);
#endif // COLOR_SEGMENTATION_HPP
