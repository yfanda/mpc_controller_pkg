#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/raspberry_pi_to_pixhawk.hpp"
#include "px4_msgs/msg/pixhawk_to_raspberry_pi.hpp"
#include <cstdlib> 
#include <grampc.hpp>
#include <vector> 
#include "UAV_model.hpp"

class SimulatorNode : public rclcpp::Node
{
public:
    SimulatorNode() : Node("simulator"), problem(0.3, 0.01)
    {
        // get initial state x_0
        state_ = {0.0};
        double* state_ptr = state_.data();

        const char* workspace_path = std::getenv("PWD");
        std::string filename1_ = std::string(workspace_path) + "/src/mpc_controller_pkg/Traj/x_traj_rotor1_failure_3.txt";
        filename1 = filename1_.c_str();
        
        lineReader(state_ptr, 9, filename1, 1);

        
        //create subscriber and publisher       
        subscription_ = this->create_subscription<px4_msgs::msg::RaspberryPiToPixhawk>(
            "/fmu/in/raspberry_pi_to_pixhawk", 10,
            std::bind(&SimulatorNode::topic_callback, this, std::placeholders::_1));   
                
        publisher_ = this->create_publisher<px4_msgs::msg::PixhawkToRaspberryPi>(
            "/fmu/out/pixhawk_to_raspberry_pi", 10);

        RCLCPP_INFO(this->get_logger(), "Simulator node has been started.");

        // publish initial state x_0
        publish_state(state_ptr);

        std::ostringstream state_stream;
        state_stream << "Initial state: ";
        for (size_t i = 0; i < 9; ++i) {
            state_stream << state_[i] << " ";  
        }
        RCLCPP_INFO(this->get_logger(), "%s", state_stream.str().c_str());
    }

private:
    
    void topic_callback(const px4_msgs::msg::RaspberryPiToPixhawk::SharedPtr msg)
    {
        
        // RCLCPP_INFO(this->get_logger(), "Received message from RaspberryPiToPixhawk");

        std::array<double, 8> input;
        for (size_t i = 0; i < 8; ++i) {
            input[i] = static_cast<double>(msg->msg_payload[i]);
        }

        // print input
        std::ostringstream input_stream;
        input_stream << "Input values: ";
        for (size_t i = 0; i < 8; ++i) {
            input_stream << input[i] << " "; 
        }
        RCLCPP_INFO(this->get_logger(), "%s", input_stream.str().c_str());

        // integration with rk4
        std::array<double, 9> next_state = problem.rk4(state_, input);
        

        auto new_msg = px4_msgs::msg::PixhawkToRaspberryPi();
        for (size_t i = 0; i < 9; ++i) {
            new_msg.msg_payload[i] = static_cast<float>(next_state[i]);
        }
        for (size_t i = 9; i < 16; ++i) {
            new_msg.msg_payload[i] = 0.0f;
        }
        publisher_->publish(new_msg);

        state_ = next_state;

        // print state
        std::ostringstream state_stream;
        state_stream << "Updated state: ";
        for (size_t i = 0; i < 9; ++i) {
            state_stream << state_[i] << " ";  
        }
        RCLCPP_INFO(this->get_logger(), "%s", state_stream.str().c_str());

        // RCLCPP_INFO(this->get_logger(), "Published message to PixhawkToRaspberryPi");
    }

    void publish_state(typeRNum* x)
    {
        auto msg = px4_msgs::msg::PixhawkToRaspberryPi();

        
        for (size_t i = 0; i < 9; ++i) {
            msg.msg_payload[i] = static_cast<float>(x[i]);  // 将 x0 的值填充到消息中
        }

        
        for (size_t i = 9; i < 16; ++i) {
            msg.msg_payload[i] = 0.0f;  // 其余的值可以设置为 0
        }

        
        publisher_->publish(msg);

        // RCLCPP_INFO(this->get_logger(), "Initial state (x0) has been published.");
    }

    
    rclcpp::Subscription<px4_msgs::msg::RaspberryPiToPixhawk>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::PixhawkToRaspberryPi>::SharedPtr publisher_;
    const char* filename1; // x_taj.txt
    const char* filename2; // u_traj.txt
    UAVModel problem;
    std::array<double, 9> state_;

};

int main(int argc, char *argv[])
{
   
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SimulatorNode>());

    rclcpp::shutdown();
    return 0;
}
