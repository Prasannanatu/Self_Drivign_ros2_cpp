#pragma once


#include "self_driving_cpp/color_segmentation.h"
#include "self_driving_cpp/config.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>



void DetectLanes(cv::Mat& frames);