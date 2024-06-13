#include "self_driving_cpp/morphology.h"
// #include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function definition
cv::Mat BwareaOpen(cv::Mat& img,cv::Mat& thresh, double minArea) {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inside BwareaOpen");
    // Ensure the image is binary
    cv::Mat thresh1 = cv::Mat::zeros(img.size(), img.type());
    // If the image has more than one channel (i.e., it's not grayscale), convert it to grayscale
    // if (img.channels() > 1) {
    //     cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    // }
    // Apply binary threshold
    // This will convert the grayscale image to a binary image, where pixels are either 0 (black) or 255 (white)
    cv::threshold(img, thresh, 0, 255, cv::THRESH_BINARY);
    
    // // Find contours in the binary image
    // // Contours are simply the boundaries of the connected white pixel regions
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE); 
    // cv::drawContours(thresh1, contours, -1, cv::Scalar(255), 1);
    
    // Filter out small contours based on the minimum area
    // This is where the 'area open' operation happens
    std::vector<std::vector<cv::Point>> contoursToRemove;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Area is less than minArea: %s", area < minArea ? "true" : "false");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "minArea: %f", minArea);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Area: %f", area);
        if (area < minArea) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Inside If statement");
            // If the area of the contour is less than the minimum area, add it to the list of contours to remove
        
            contoursToRemove.push_back(contour);
        }
    }
    // // Draw the small contours onto the thresholded image with black color (0)
    // // This effectively removes these contours from the image
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Contours to remove: %d", contoursToRemove.size());
    cv::drawContours(thresh1, contoursToRemove, -1, cv::Scalar(255, 255, 255), 2);
    // cv::drawContours(thresh1, contoursToRemove, -1, cv::Scalar(1), cv::FILLED);
    cv::imshow("Threshold11111", thresh1);
    cv::waitKey(1);
    // Return the modified threshold matrix
    return thresh;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<int, int> FindEXtrema(cv::Mat& img) {
    // Initialize the top and bottom values to the maximum and minimum possible integer values
    int top = std::numeric_limits<int>::max();
    int bottom = std::numeric_limits<int>::min();

    // Iterate over each pixel in the image
    for (int row = 0 ; row < img.rows ; row++) {
        for (int col = 0 ; col < img.cols ; col++) {
            // If the pixel value is not zero
            if (img.at<int>(row, col) != 0) {
                // If the current row is less than the current top value, update the top value
                if (row < top)  top = row;
                // If the current row is more than the current bottom value, update the bottom value
                if (row > bottom) bottom = row;
            }
        }
    }

    // If the top and bottom values have not been updated (i.e., there are no non-zero pixels), return {0, 0}
    if (top == std::numeric_limits<int>::max() || bottom == std::numeric_limits<int>::min()) {
        return {0, 0};
    }
    
    // Return the top and bottom values
    return {top, bottom};
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int FindLowestRow(cv::Mat& img) {
    // Find the maximum and minimum values in the image
    int bottom = std::numeric_limits<int>::min();

    for (int row = 0 ; row < img.rows ; row++) {
        for (int col = 0 ; col < img.cols ; col++) {
            if (img.at<int>(row, col) != 0) {
                if (row > bottom) bottom = row;
            }
        }
    }
    if (bottom == std::numeric_limits<int>::min()) {
        return 0;
    }
    
    return bottom;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Point ExtractPoint(cv::Mat& img, int row) {
    cv::Point point;
    
    for (int col = 0 ; col < img.cols ; col++) {
        if (img.at<int>(row, col) != 0) {
            point = cv::Point(col, row);
            return point;
        }
    }
    return point;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<cv::Mat, bool>RegisterLargestContour(cv::Mat& Gray_image)
{
    bool largest_contour_found = false;
    cv::Mat Mask = cv::Mat::zeros(Gray_image.size(), Gray_image.type());
    cv::Mat bin_img;
    cv::threshold(Gray_image, bin_img, 0, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
        int largest_contour_index = -1;
        double largest_area = 0.0;
        for (int i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > largest_area) {
                largest_area = area;
                largest_contour_index = i;

            }
        }
         if (largest_contour_index != -1) {
            cv::drawContours(Mask, contours, largest_contour_index, cv::Scalar(255), cv::FILLED);
            largest_contour_found = true;
        }
    }


    return {Mask, largest_contour_found};
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<cv::Mat, bool>RegisterLargestContourOuterLane(cv::Mat& Gray_image, double min_area)
{
    bool largest_contour_found = false;
    cv::Mat Mask = cv::Mat::zeros(Gray_image.size(), Gray_image.type());
    cv::Mat bin_img, bin_img_dilated;
    cv::threshold(Gray_image, bin_img, 0, 255, cv::THRESH_BINARY);

///////////////////////////////////////////TESTING SHADOW BREAKER CODE BY DILATING///////////////////////////////////////////

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(bin_img, bin_img_dilated, cv::MORPH_DILATE, kernel);
    cv::morphologyEx(bin_img_dilated, bin_img, cv::MORPH_ERODE, kernel);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
        int largest_contour_index = -1;
        double largest_area = 0.0;
        for (int i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > largest_area) {
                largest_area = area;
                largest_contour_index = i;

            }
        }
        if (largest_area < min_area) {
            largest_contour_found = false;
        }
         if (largest_contour_index != -1  && largest_contour_found) {
            cv::drawContours(Mask, contours, largest_contour_index, cv::Scalar(255), cv::FILLED);
        }
    }


    return {Mask, largest_contour_found};
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat ROI_extractor(cv::Mat& img, cv::Point start_point, cv::Point end_point) {
    cv::Rect rect(start_point, end_point);
    cv::Mat ROI = img(rect).clone();
    return ROI;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair< cv::Mat, std::vector<cv::Point>> RegisterLowestEdgePoints(cv::Mat& gray_image)
{
    std::vector<std::vector<cv::Point>> contours, contours2,tmp_contours ;
    std::vector<cv::Point> lowest_points;
    cv::Mat thresh = cv::Mat::zeros(gray_image.size(), gray_image.type());
    cv::Mat Lane_OneSide = cv::Mat::zeros(gray_image.size(), gray_image.type());
    cv::Mat Lane_TwoSide = cv::Mat::zeros(gray_image.size(), gray_image.type());
    cv::Mat bin_img;
    cv::threshold(gray_image, bin_img, 0, 255, cv::THRESH_BINARY);
    cv::findContours(bin_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(thresh, contours, 0, cv::Scalar(255, 255, 255), 1);
    std::pair<int, int> extrema = FindEXtrema(thresh);
    int top = extrema.first;
    int bottom = extrema.second;
    cv::Mat ROI = ROI_extractor(thresh, cv::Point(0, top + 5), cv::Point(thresh.cols, bottom -5));
    cv::findContours(ROI, contours2, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Point point_a, point_b;

    int LowRow_a = -1;
    int LowRow_b = -1;
    int Euc_row = 0;

    cv::Mat FirstLine = cv::Mat::zeros(gray_image.size(), gray_image.type());
    

    if (contours2.size() > 1){
        for (int i = 0; i < contours2.size(); i++) {
            if((contours2[i].size() > 50)){
                tmp_contours.push_back(contours2[i]);

            }
        }
    }

    for (int i = 0; i < contours2.size(); i++) {
        cv::Mat Lane_OneSide = cv::Mat::zeros(gray_image.size(), gray_image.type());
        cv::drawContours(Lane_OneSide, contours2, i, cv::Scalar(255, 255, 255), 1);
        cv::drawContours(Lane_TwoSide, contours2, i, cv::Scalar(255, 255, 255), 1);


        if (contours2.size() == 2){
            if(i ==0)
            {
                FirstLine =Lane_OneSide.clone();
                LowRow_a = FindLowestRow(Lane_OneSide);
            }
            else if(i == 1)
            {
                LowRow_b = FindLowestRow(Lane_OneSide);
                if(LowRow_a < LowRow_b)
                {
                    Euc_row = LowRow_a;
                }
                else
                {
                    Euc_row = LowRow_b;
                }
                point_a = ExtractPoint(FirstLine, Euc_row);
                point_b = ExtractPoint(Lane_OneSide, Euc_row);
                lowest_points.push_back(point_a);
                lowest_points.push_back(point_b);
            }
            

            }
        
        // Code inside the loop
    }
    return {Lane_TwoSide, lowest_points};
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////