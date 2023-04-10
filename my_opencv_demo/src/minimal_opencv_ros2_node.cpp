#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>               // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp>                  // We include everything about OpenCV as we don't care much about compilation time at the moment.
#include <ctime>
#include "my_opencv_demo_interfaces/msg/image_with_time.hpp"

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node
{
public:
    MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0)
    {
        publisher_ =
            this->create_publisher<my_opencv_demo_interfaces::msg::ImageWithTime>("random_image", 10);
        timer_ = this->create_wall_timer(
            300ms, std::bind(&MinimalImagePublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // Create a new 640x480 image
        cv::Mat my_image(cv::Size(640, 480), CV_8UC3);

        // Generate an image where each pixel is a random color
        cv::randu(my_image, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));

        // Declare the text position
        cv::Point text_position(15, 40);

        // Declare the size and color of the font
        int font_size = 1;
        cv::Scalar font_color(255, 255, 255);

        // Declare the font weight
        int font_weight = 2;

        // Put the text in the image
        cv::putText(my_image, "ROS2 + OpenCV", text_position, cv::FONT_HERSHEY_COMPLEX, font_size, font_color, font_weight);

        // Write message to be sent. Member function toImageMsg() converts a CvImage
        // into a ROS image message
        msg_.image = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image)
                           .toImageMsg());

        uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // std::cout << "------------time publish" << ms << std::endl;
        // Publish the image to the topic defined in the publisher
        msg_.time = ms;
        publisher_->publish(msg_);
        RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
        count_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    my_opencv_demo_interfaces::msg::ImageWithTime msg_;
    rclcpp::Publisher<my_opencv_demo_interfaces::msg::ImageWithTime>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // create a ros2 node
    auto node = std::make_shared<MinimalImagePublisher>();

    // process ros2 callbacks until receiving a SIGINT (ctrl-c)
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
