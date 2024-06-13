#pragma once

#include <opencv2/opencv.hpp>


cv::Mat BwareaOpen(cv::Mat& img,cv::Mat& thresh, double minArea);
std::pair<int, int> FindEXtrema(cv::Mat& img);
int FindLowestRow(cv::Mat& img);
cv::Point ExtractPoint(cv::Mat& img, int row) ;
std::pair<cv::Mat, bool>RegisterLargestContour(cv::Mat& Gray_image);
cv::Mat ROI_extractor(cv::Mat& img, cv::Point start_point, cv::Point end_point);
std::pair< cv::Mat, std::vector<cv::Point>> RegisterLowestEdgePoints(cv::Mat& gray_image);
std::pair<cv::Mat, bool>RegisterLargestContourOuterLane(cv::Mat& Gray_image, double min_area);
std::pair<cv::Mat, std::vector<cv::Point>> RegisterLowestEdgePoints(cv::Mat& gray_image);