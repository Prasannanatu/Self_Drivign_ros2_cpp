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
void SegmentLanes(cv::Mat& input_image,int minArea);

#endif // COLOR_SEGMENTATION_HPP
