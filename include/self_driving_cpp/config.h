#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <opencv2/opencv.hpp>
#include <string>

extern int detect;

extern bool Testing;
extern bool Profiling;
extern bool write_;
extern bool In_write;
extern bool Out_write;

extern bool debugging;
extern bool debugging_Lane;
extern bool debugging_L_ColorSeg;
extern bool debugging_L_Est;
extern bool debugging_L_Cleaning;
extern bool debugging_L_LaneInfoExtraction;

extern bool debugging_Signs;
extern bool debugging_TrafficLights;
extern bool debugging_TL_Config;
extern bool enable_SatNav;
extern bool animate_steering;

extern int angle_orig;
extern int angle;
extern bool engines_on;
extern bool clr_seg_dbg_created;

extern bool Detect_lane_N_Draw;
extern bool Training_CNN;

extern std::string vid_path;
extern int loopCount;

extern int Resized_width;
extern int Resized_height;

extern cv::VideoWriter in_q;
extern cv::VideoWriter out;

extern int waitTime;

extern int Ref_imgWidth;
extern int Ref_imgHeight;
extern int Frame_pixels;
extern int Resize_Framepixels;
extern float Lane_Extraction_minArea_per;
extern int minArea_resized;
extern float BWContourOpen_speed_MaxDist_per;
extern int MaxDist_resized;
extern int CropHeight;
extern int CropHeight_resized;

#endif // CONFIG_HPP
