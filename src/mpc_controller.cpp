#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/pixhawk_to_raspberry_pi.hpp"
#include "px4_msgs/msg/raspberry_pi_to_pixhawk.hpp"

class MpcController : public rclcpp::Node
{
public:
    MpcController() : Node("mpc_controller")
    {
        // Subscriber to Pixhawk to RaspberryPi message
        subscription_ = this->create_subscription<px4_msgs::msg::PixhawkToRaspberryPi>(
            "/fmu/out/pixhawk_to_raspberry_pi", 10,
            std::bind(&MpcController::topic_callback, this, std::placeholders::_1));

        // Publisher to RaspberryPi to Pixhawk message
        publisher_ = this->create_publisher<px4_msgs::msg::RaspberryPiToPixhawk>(
            "/fmu/in/raspberry_pi_to_pixhawk", 10);
    }

private:
    // Callback function for subscription
    void topic_callback(const px4_msgs::msg::PixhawkToRaspberryPi::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received message from Pixhawk, forwarding msg_payload.");

        // Create a message to send to the Pixhawk
        auto outgoing_msg = px4_msgs::msg::RaspberryPiToPixhawk();
        
        // Copy the incoming msg_payload to the outgoing message
        for (size_t i = 0; i < 16; i++) {
            outgoing_msg.msg_payload[i] = msg->msg_payload[i];
        }

        // Publish the message
        publisher_->publish(outgoing_msg);
    }

    rclcpp::Subscription<px4_msgs::msg::PixhawkToRaspberryPi>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::RaspberryPiToPixhawk>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MpcController>());
    rclcpp::shutdown();
    return 0;
}
