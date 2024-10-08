#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/pixhawk_to_raspberry_pi.hpp"
#include "px4_msgs/msg/raspberry_pi_to_pixhawk.hpp"

class SimPix : public rclcpp::Node
{
public:
    SimPix() : Node("sim_pix")
    {
        // Publisher to publish PixhawkToRaspberryPi messages
        publisher_ = this->create_publisher<px4_msgs::msg::PixhawkToRaspberryPi>(
            "/fmu/out/pixhawk_to_raspberry_pi", 10);

        // Subscriber to RaspberryPiToPixhawk messages
        subscription_ = this->create_subscription<px4_msgs::msg::RaspberryPiToPixhawk>(
            "/fmu/in/raspberry_pi_to_pixhawk", 10,
            std::bind(&SimPix::topic_callback, this, std::placeholders::_1));

/*         // Publish a message when the node starts
        auto msg = px4_msgs::msg::PixhawkToRaspberryPi();
        
        // Fill the message payload with some example data
        for (size_t i = 0; i < 16; i++) {
            msg.msg_payload[i] = static_cast<float>(i);
        }

        // Log and publish the message
        RCLCPP_INFO(this->get_logger(), "Publishing initial PixhawkToRaspberryPi message.");
        publisher_->publish(msg); */

        // Set a timer to continuously publish messages at a regular interval (optional)
        //timer_ = this->create_wall_timer(
        //    std::chrono::seconds(1), std::bind(&SimPix::publish_message, this));
    }

private:
    // Callback function to handle incoming RaspberryPiToPixhawk messages
    void topic_callback(const px4_msgs::msg::RaspberryPiToPixhawk::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received RaspberryPiToPixhawk message.");

        // You can process the received message here if needed
       /*  for (size_t i = 0; i < 16; i++) {
            RCLCPP_INFO(this->get_logger(), "Payload[%zu]: %f", i, msg->msg_payload[i]);
        } */

        // Create a message to send to the Pixhawk
        auto outgoing_msg = px4_msgs::msg::PixhawkToRaspberryPi();
        
        // Copy the incoming msg_payload to the outgoing message
        for (size_t i = 0; i < 16; i++) {
            outgoing_msg.msg_payload[i] = msg->msg_payload[i];
        }

        // Publish the message
        publisher_->publish(outgoing_msg);
    }

    // Function to periodically publish messages (optional)
    void publish_message()
    {
        auto msg = px4_msgs::msg::PixhawkToRaspberryPi();
        for (size_t i = 0; i < 16; i++) {
            msg.msg_payload[i] = static_cast<float>(i);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing periodic PixhawkToRaspberryPi message.");
        publisher_->publish(msg);
    }

    rclcpp::Publisher<px4_msgs::msg::PixhawkToRaspberryPi>::SharedPtr publisher_;
    rclcpp::Subscription<px4_msgs::msg::RaspberryPiToPixhawk>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimPix>());
    rclcpp::shutdown();
    return 0;
}
