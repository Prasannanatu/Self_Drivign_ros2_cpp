#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "self_driving_cpp/color_segmentation.h"
#include "self_driving_cpp/lane_detection.h"
#include "self_driving_cpp/config.h"
#include "self_driving_cpp/drive_bot.h"

class VideoFeedIn : public rclcpp::Node {
public:
    VideoFeedIn() : Node("video_feed_in"), car_(cv::Mat()) {
        RCLCPP_INFO(this->get_logger(), "Video feed in node has been started.");
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&VideoFeedIn::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 40);
        createTrackbars();
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow("Camera Feed", cv_ptr->image);
        car_.Drive(cv_ptr->image);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    Car car_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoFeedIn>());
    rclcpp::shutdown();
    return 0;
}
