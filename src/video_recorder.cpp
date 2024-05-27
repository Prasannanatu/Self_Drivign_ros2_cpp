#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoRecorder : public rclcpp::Node
{
public:
  VideoRecorder()
    : Node("video_recorder"),
      writer_("/home/prsnna/output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(1280, 720))
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&VideoRecorder::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    try
    {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    writer_.write(frame); // Write the frame to the video
    cv::imshow("output", frame); // Display the frame
    cv::waitKey(1); // Needed for imshow to work correctly
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  cv::VideoWriter writer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoRecorder>());
  rclcpp::shutdown();
  return 0;
}
