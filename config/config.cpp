#include "self_driving_cpp/config.h"
#include <opencv2/opencv.hpp>
#include <string>

int detect = 1;

bool Testing = true;
bool Profiling = false;
bool write = false;
bool In_write = false;
bool Out_write = false;

bool debugging = true;
bool debugging_Lane = true;
bool debugging_L_ColorSeg = true;
bool debugging_L_Est = true;
bool debugging_L_Cleaning = true;
bool debugging_L_LaneInfoExtraction = true;

bool debugging_Signs = true;
bool debugging_TrafficLights = true;
bool debugging_TL_Config = true;
bool enable_SatNav = false;
bool animate_steering = false;

int angle_orig = 0;
int angle = 0;
bool engines_on = false;
bool clr_seg_dbg_created = false;

bool Detect_lane_N_Draw = true;
bool Training_CNN = false;

std::string vid_path = "data/vids/Ros2/lane.avi";
int loopCount = 0;

int Resized_width = 320;
int Resized_height = 240;

cv::VideoWriter in_q("data/Output/in_new.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(Resized_width, Resized_height));
cv::VideoWriter out("data/Output/out_new.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(Resized_width, Resized_height));

int waitTime = debugging ? 1 : 1;

int Ref_imgWidth = 1920;
int Ref_imgHeight = 1080;
int Frame_pixels = Ref_imgWidth * Ref_imgHeight;
int Resize_Framepixels = Resized_width * Resized_height;
float Lane_Extraction_minArea_per = 1000.0 / Frame_pixels;
int minArea_resized = static_cast<int>(Resize_Framepixels * Lane_Extraction_minArea_per);
float BWContourOpen_speed_MaxDist_per = 500.0 / Ref_imgHeight;
int MaxDist_resized = static_cast<int>(Resized_height * BWContourOpen_speed_MaxDist_per);
int CropHeight = 650;
int CropHeight_resized = static_cast<int>((CropHeight / static_cast<float>(Ref_imgHeight)) * Resized_height);
