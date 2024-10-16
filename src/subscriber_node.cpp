#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/pixhawk_to_raspberry_pi.hpp"
#include "px4_msgs/msg/raspberry_pi_to_pixhawk.hpp"
#include <fstream>
#include <filesystem>
#include <iomanip>  // For formatting the float values

namespace fs = std::filesystem;

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode()
  : Node("subscriber_node")
  {
    // QoS setting to fit pixhawk
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    // Subscription to the first topic: Pixhawk to Raspberry Pi
    subscription_out_ = this->create_subscription<px4_msgs::msg::PixhawkToRaspberryPi>(
      "/fmu/out/pixhawk_to_raspberry_pi", qos,
      std::bind(&SubscriberNode::topic_callback_out, this, std::placeholders::_1)
    );

    // Subscription to the second topic: Raspberry Pi to Pixhawk
    subscription_in_ = this->create_subscription<px4_msgs::msg::RaspberryPiToPixhawk>(
      "/fmu/in/raspberry_pi_to_pixhawk", qos,
      std::bind(&SubscriberNode::topic_callback_in, this, std::placeholders::_1)
    );

    // Initialize the log files
    init_files();
  }

private:
  // Callback function for /fmu/out/pixhawk_to_raspberry_pi
  void topic_callback_out(const px4_msgs::msg::PixhawkToRaspberryPi::SharedPtr msg)
  {
    if (!out_file_.is_open()) return;

    // Write float32[16] msg_payload to file
    for (int i = 0; i < 16; i++) {
      out_file_ << std::fixed << std::setprecision(6) << msg->msg_payload[i];
      if (i < 15) {
        out_file_ << ", ";  // Separate values with commas
      }
    }
    out_file_ << "\n";  // Newline after each message
  }

  // Callback function for /fmu/in/raspberry_pi_to_pixhawk
  void topic_callback_in(const px4_msgs::msg::RaspberryPiToPixhawk::SharedPtr msg)
  {
    if (!in_file_.is_open()) return;

    // Write float32[16] msg_payload to file
    for (int i = 0; i < 16; i++) {
      in_file_ << std::fixed << std::setprecision(6) << msg->msg_payload[i];
      if (i < 15) {
        in_file_ << ", ";  // Separate values with commas
      }
    }
    in_file_ << "\n";  // Newline after each message
  }

  // Function to initialize log files
  void init_files() {
    // Create a directory for storing the log files
    fs::path dir = "ros2_message_logs";
    if (!fs::exists(dir)) {
      fs::create_directory(dir);
    }

    // Open files to append data from the two topics
    out_file_.open(dir / "pixhawk_to_raspberry_pi.txt", std::ios::out | std::ios::app);
    in_file_.open(dir / "raspberry_pi_to_pixhawk.txt", std::ios::out | std::ios::app);

    if (!out_file_.is_open() || !in_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open files for logging.");
    }
  }

  // Subscriptions for the two topics
  rclcpp::Subscription<px4_msgs::msg::PixhawkToRaspberryPi>::SharedPtr subscription_out_;
  rclcpp::Subscription<px4_msgs::msg::RaspberryPiToPixhawk>::SharedPtr subscription_in_;

  // File streams to log the message data
  std::ofstream out_file_;
  std::ofstream in_file_;
};

// Main function to run the node
int main(int argc, char *argv[])
{
  // Initialize the ROS 2 system
  rclcpp::init(argc, argv);

  // Create a node instance and spin it
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  return 0;
}
