#include "self_driving_cpp/lane_detection.h"
#include "self_driving_cpp/color_segmentation.h"

void DetectLanes(cv::Mat& frames) {
    cv::Rect roi(0, CropHeight_resized, frames.cols, frames.rows - CropHeight_resized);
    cv::Mat cropped_frame = frames(roi);
    SegmentLanes(cropped_frame, minArea_resized);   
}
