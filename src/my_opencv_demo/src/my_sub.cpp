#include <memory>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "my_opencv_demo_interfaces/msg/image_with_time.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<my_opencv_demo_interfaces::msg::ImageWithTime>(
            "random_image", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const my_opencv_demo_interfaces::msg::ImageWithTime::SharedPtr msg) const
    {
        auto current_image = *msg;
        uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        RCLCPP_INFO(this->get_logger(), "get_image: %i ms", ms - current_image.time);
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(current_image.image, sensor_msgs::image_encodings::BGR8);
            // cv_bridge::toCvShare(msg, enc::BGR8);
            cv::imshow("view", cv_ptr->image);
            cv::waitKey(10);
        }
        catch (const cv_bridge::Exception &e)
        {
            auto logger = rclcpp::get_logger("my_subscriber");
            RCLCPP_ERROR(logger, "Could not convert to 'bgr8'.");
        }
    }
    rclcpp::Subscription<my_opencv_demo_interfaces::msg::ImageWithTime>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    cv::namedWindow("view");
    cv::startWindowThread();
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}