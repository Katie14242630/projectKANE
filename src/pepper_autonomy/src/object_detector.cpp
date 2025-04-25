#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ObjectDetector : public rclcpp::Node
{
public:
    ObjectDetector()
    : Node("object_detector")
    {
        // Subscribe to LaserScan data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(), std::bind(&ObjectDetector::laser_callback, this, std::placeholders::_1));
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Process LaserScan data for object detection (like cylinders)
        std::vector<float> ranges = msg->ranges;
        float min_range = msg->range_min;
        float max_range = msg->range_max;

        // Check if there are any obstacles within a critical range in front of the robot
        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] < 0.65) {  // Example threshold for detecting objects within 0.5 meters
                RCLCPP_INFO(this->get_logger(), "Obstacle detected at %.2f meters", ranges[i]);
                // You could trigger more specific behavior here, such as notifying Nav2
                return;  // Early exit after detecting an object
            }
        }

        RCLCPP_INFO(this->get_logger(), "No obstacles detected");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetector>());
    rclcpp::shutdown();
    return 0;
}
