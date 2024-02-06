#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>

using namespace std;

class VideoPublisher : public rclcpp::Node {
public:
  VideoPublisher() : Node("video_publisher"), count_(0) {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&VideoPublisher::publishVideo, this));

    // 원래는 아래에 있던거
    
    // Load a frame from your video file
    // Modify the path based on your video file location
    std::string videoPath = "/workspaces/isaac_ros-dev/isaac_ros-dev/sample_video/AprilTag_test_video.mp4";
    std::string absolutePath = std::filesystem::absolute(videoPath).string();
    // cv::VideoCapture cap_(absolutePath);
    cap_.open(absolutePath);

    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Error opening video file");
      return;
    }
  }

private:
  void publishVideo() {
    cv::Mat frame;
    cap_ >> frame;

    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "camera_frame";
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = frame.cols * 3;
    size_t size = frame.rows * frame.cols * 3;
    msg.data.resize(size);

    // Copy the image data from the OpenCV matrix to the ROS message
    memcpy(msg.data.data(), frame.data, size);

    publisher_->publish(msg);

    // Set camera information
    float fx = 1285.711823560571474;
    float fy = 1290.706327751410072;

    float cx = 834.975118301024850;
    float cy = 628.697079583048776;

    // Publish CameraInfo message
    sensor_msgs::msg::CameraInfo camera_info_msg;
    camera_info_msg.header = msg.header;
    camera_info_msg.height = 1200; // msg.height;
    camera_info_msg.width = 1600; // msg.width;
    camera_info_msg.distortion_model = "plumb_bob";  // Update with the correct distortion model

    // Set camera matrix
    camera_info_msg.k[0] = fx;
    camera_info_msg.k[4] = fy;
    camera_info_msg.k[2] = cx;
    camera_info_msg.k[5] = cy;
    camera_info_msg.k[8] = 1.0;

    // Set rectification matrix
    camera_info_msg.r[0] = 1.0;
    camera_info_msg.r[4] = 1.0;
    camera_info_msg.r[8] = 1.0;

    // Set projection matrix
    camera_info_msg.p[0] = fx;
    camera_info_msg.p[5] = fy;
    camera_info_msg.p[2] = cx;
    camera_info_msg.p[6] = cy;
    camera_info_msg.p[10] = 1.0;

    camera_info_publisher_->publish(camera_info_msg);

    ++count_;

    // // Display the frame using OpenCV imshow
    // cv::imshow("Frame", frame);
    // cv::waitKey(1);  // Add a short delay to display the frame
    RCLCPP_INFO(this->get_logger(), "%zu\n", count_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  size_t count_;
  cv::VideoCapture cap_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // ROS_INFO("%s", "Start");
  // cout << "Start" << endl;
  rclcpp::spin(std::make_shared<VideoPublisher>());
  rclcpp::shutdown();
  return 0;
}
