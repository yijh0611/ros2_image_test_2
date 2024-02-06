#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
    : Node("imshow_test") // image_subscriber
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image",
            10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
        bridge_ = std::make_shared<cv_bridge::CvImage>();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // ROS 이미지 메시지를 OpenCV 이미지로 변환
            bridge_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat cv_image = bridge_->image;

            // OpenCV의 imshow 함수를 사용하여 이미지 표시
            cv::imshow("Received Image", cv_image);
            cv::waitKey(1);  // imshow 창 업데이트를 위해 추가
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert ROS image to OpenCV image: %s", e.what());
            return;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::shared_ptr<cv_bridge::CvImage> bridge_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto image_subscriber = std::make_shared<ImageSubscriber>();
    rclcpp::spin(image_subscriber);
    // ROS 2 노드가 종료될 때 OpenCV 창도 닫기
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
