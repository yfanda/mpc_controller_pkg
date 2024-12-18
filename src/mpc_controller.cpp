#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/pixhawk_to_raspberry_pi.hpp"
#include "px4_msgs/msg/raspberry_pi_to_pixhawk.hpp"
#include <iostream>
#include <fstream>
#include <chrono> 
#include <vector> 
#include <grampc.hpp>
#include "UAV_model.hpp"
#include <cstdlib> // for getenv()

#define NX	9
#define NU	8
#define NC	19
#define NP	0

class MpcController : public rclcpp::Node
{
public:
    // Node Constructer with initializing list problem(Thor,  dt),solver(&problem)
    MpcController() : Node("mpc_controller"), problem(0.3, 0.01),solver(&problem)
    {
        // QoS setting to fit pixhawk
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Subscriber to Pixhawk to RaspberryPi message
        subscription_ = this->create_subscription<px4_msgs::msg::PixhawkToRaspberryPi>(
            "/fmu/out/pixhawk_to_raspberry_pi", qos,
            std::bind(&MpcController::topic_callback, this, std::placeholders::_1));

        /* // Subscriber test to send back the msg it receives
        subscription_ = this->create_subscription<px4_msgs::msg::PixhawkToRaspberryPi>(
            "/fmu/out/pixhawk_to_raspberry_pi", qos,
            std::bind(&MpcController::test_publish_message, this, std::placeholders::_1)); */

        // Publisher to RaspberryPi to Pixhawk message
        publisher_ = this->create_publisher<px4_msgs::msg::RaspberryPiToPixhawk>(
            "/fmu/in/raspberry_pi_to_pixhawk", 10);
        
        /************Files for saving state and control data and computation time************/
        state_file_.open("state.txt", std::ios::out | std::ios::trunc);
        control_file_.open("control.txt", std::ios::out | std::ios::trunc);
        compute_time_file_.open("compute_time.txt", std::ios::out | std::ios::trunc);
        if(!state_file_.is_open() || !control_file_.is_open() || !compute_time_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output files.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Output files opened successfully.");
        }

        
        /*****Initialize parameters and options for grampc*******/        
        const char* workspace_path = std::getenv("PWD");
        filename1_ = std::string(workspace_path) + "/src/mpc_controller_pkg/Traj/x_traj.txt";
        filename2_ = std::string(workspace_path) + "/src/mpc_controller_pkg/Traj/u_traj.txt";
        filename1 = filename1_.c_str();
        filename2 = filename2_.c_str();
        

        typeRNum x0[9] = {0,0,0,0,0,0,0,0,0} ;
        lineReader(x0, 9, filename1, 1);

        typeRNum u0[8] = {0,0,0,0,0,0,0,0};
        lineReader(u0, 8, filename2, 1); 

        ctypeRNum umax[NU] = {100,100,100,100,100,0.4363,0.3491,0.5236};
        ctypeRNum umin[NU] = {0,0,0,0,0,-0.4363,-0.3491,-0.5236};

        ctypeRNum Thor = problem.Thor;	/* Prediction horizon */
        ctypeRNum dt = problem.dt;  /* Sampling time */
        typeRNum t = (typeRNum)0.0;   /* time at the current sampling step */

        /********* Option definition *********/
        /* Basic algorithmic options */
        ctypeInt Nhor = (typeInt)6;        /* Number of steps for the system integration */
        ctypeInt MaxGradIter = (typeInt)10;  /* Maximum number of gradient iterations */
        ctypeInt MaxMultIter = (typeInt)1;  /* Maximum number of augmented Lagrangian iterations */
        const char* ShiftControl = "on";

        /* Cost integration */
        const char* IntegralCost = "on";
        const char* TerminalCost = "on";
        const char* IntegratorCost = "trapezodial";

        /* System integration */
        const char* Integrator = "heun";
        ctypeRNum IntegratorRelTol = (typeRNum)1e-6;
        ctypeRNum IntegratorAbsTol = (typeRNum)1e-8;
        ctypeRNum IntegratorMinStepSize = EPS;
        ctypeInt  IntegratorMaxSteps = (typeInt)1e8;
        ctypeInt FlagsRodas[8] = { 0, 0, 0, NX, NX, 0, NX, NX }; 

        /* Line search */
        const char* LineSearchType = "explicit2";
        const char* LineSearchExpAutoFallback = "on";
        ctypeRNum LineSearchMax = (typeRNum)100;
        ctypeRNum LineSearchMin = (typeRNum)1e-10;
        ctypeRNum LineSearchInit = (typeRNum)1e-4;
        ctypeRNum LineSearchAdaptAbsTol = (typeRNum)1e-6;
        ctypeRNum LineSearchAdaptFactor = (typeRNum)3.0 / 2.0;
        ctypeRNum LineSearchIntervalTol = (typeRNum)1e-1;
        ctypeRNum LineSearchIntervalFactor = (typeRNum)0.85;

        /* Input and or parameter optimization	*/
        const char* OptimControl = "on";
        const char* OptimParam = "off";
        ctypeRNum OptimParamLineSearchFactor = (typeRNum)1.0;
        const char* OptimTime = "off";
        ctypeRNum OptimTimeLineSearchFactor = (typeRNum)1.0;

        /* Scaling Values for the states, inputs and parameters */
        const char* ScaleProblem = "off";
        ctypeRNum xScale[NX] = { 1 };
        ctypeRNum xOffset[NX] = { 0 };
        ctypeRNum uScale[NU] = { 1 };
        ctypeRNum uOffset[NU] = { 0 };
        /* ctypeRNum pScale[NP] = {  };
        ctypeRNum pOffset[NP] = {  };  */
        ctypeRNum TScale = (typeRNum)1.0;
        ctypeRNum TOffset = (typeRNum)0.0;
        ctypeRNum JScale = (typeRNum)1.0;
        /* ctypeRNum cScale[NC] = { 1 };  */

        /* Tye of constraints' consideration */
        const char* EqualityConstraints = "off";
        const char* InequalityConstraints = "off";
        const char* TerminalEqualityConstraints = "off";
        const char* TerminalInequalityConstraints = "off";
        const char* ConstraintsHandling = "auglag";
        /* ctypeRNum ConstraintsAbsTol[NC] = { 1e-4 }; */

        /* Multipliers & penalties */
        ctypeRNum MultiplierMax = (typeRNum) 1e6;
        ctypeRNum MultiplierDampingFactor = 0.8; //1
        ctypeRNum PenaltyMax = (typeRNum)1e6; 
        ctypeRNum PenaltyMin = (typeRNum)1e0;
        ctypeRNum PenaltyIncreaseFactor = (typeRNum)1.0005; //1.05
        ctypeRNum PenaltyDecreaseFactor = (typeRNum)0.95;
        ctypeRNum PenaltyIncreaseThreshold = (typeRNum)0.8; //1
        ctypeRNum AugLagUpdateGradientRelTol = (typeRNum)1e-2; 

        /* Convergences tests */
        const char* ConvergenceCheck = "on";
        ctypeRNum ConvergenceGradientRelTol = (typeRNum)1e-6; 

         /********* set parameters *********/
        solver.setparam_real_vector( "x0", x0);
        solver.setparam_real_vector( "u0", u0);
        solver.setparam_real_vector( "umax", umax);
        solver.setparam_real_vector( "umin", umin);
        solver.setparam_real( "Thor", Thor);
        solver.setparam_real( "dt", dt);
        solver.setparam_real( "t0", t);
        /********* Option definition *********/
        solver.setopt_int("Nhor", Nhor);
        solver.setopt_int("MaxGradIter", MaxGradIter);
        solver.setopt_int("MaxMultIter", MaxMultIter);
        solver.setopt_string("ShiftControl", ShiftControl);

        solver.setopt_string("IntegralCost", IntegralCost);
        solver.setopt_string("TerminalCost", TerminalCost);
        solver.setopt_string("IntegratorCost", IntegratorCost);

        solver.setopt_string("Integrator", Integrator);
        solver.setopt_real("IntegratorRelTol", IntegratorRelTol);
        solver.setopt_real("IntegratorAbsTol", IntegratorAbsTol);
        solver.setopt_real("IntegratorMinStepSize", IntegratorMinStepSize);
        solver.setopt_int("IntegratorMaxSteps", IntegratorMaxSteps);
        solver.setopt_int_vector("FlagsRodas", FlagsRodas); 

        solver.setopt_string("LineSearchType", LineSearchType);
        solver.setopt_string("LineSearchExpAutoFallback", LineSearchExpAutoFallback);
        solver.setopt_real("LineSearchMax", LineSearchMax);
        solver.setopt_real("LineSearchMin", LineSearchMin);
        solver.setopt_real("LineSearchInit", LineSearchInit);
        solver.setopt_real("LineSearchIntervalFactor", LineSearchIntervalFactor);
        solver.setopt_real("LineSearchAdaptFactor", LineSearchAdaptFactor);
        solver.setopt_real("LineSearchIntervalTol", LineSearchIntervalTol);

        solver.setopt_string("OptimControl", OptimControl);
        solver.setopt_string("OptimParam", OptimParam);
        solver.setopt_real("OptimParamLineSearchFactor", OptimParamLineSearchFactor);
        solver.setopt_string("OptimTime", OptimTime);
        solver.setopt_real("OptimTimeLineSearchFactor", OptimTimeLineSearchFactor);

        solver.setopt_string("ScaleProblem", ScaleProblem);
        solver.setopt_real_vector("xScale", xScale);
        solver.setopt_real_vector("xOffset", xOffset);
        solver.setopt_real_vector("uScale", uScale);
        solver.setopt_real_vector("uOffset", uOffset);
        /* solver.setopt_real_vector("pScale", pScale);
        solver.setopt_real_vector("pOffset", pOffset); */
        solver.setopt_real("TScale", TScale);
        solver.setopt_real("TOffset", TOffset);
        solver.setopt_real("JScale", JScale);
        /* solver.setopt_real_vector("cScale", cScale);  */

        solver.setopt_string("EqualityConstraints", EqualityConstraints);
        solver.setopt_string("InequalityConstraints", InequalityConstraints);
        solver.setopt_string("TerminalEqualityConstraints", TerminalEqualityConstraints);
        solver.setopt_string("TerminalInequalityConstraints", TerminalInequalityConstraints);
        solver.setopt_string("ConstraintsHandling", ConstraintsHandling);
    /* 	solver.setopt_real_vector("ConstraintsAbsTol", ConstraintsAbsTol);
    */
        solver.setopt_real("MultiplierMax", MultiplierMax);
        solver.setopt_real("MultiplierDampingFactor", MultiplierDampingFactor);
        solver.setopt_real("PenaltyMax", PenaltyMax);
        solver.setopt_real("PenaltyMin", PenaltyMin);
        solver.setopt_real("PenaltyIncreaseFactor", PenaltyIncreaseFactor);
        solver.setopt_real("PenaltyDecreaseFactor", PenaltyDecreaseFactor);
        solver.setopt_real("PenaltyIncreaseThreshold", PenaltyIncreaseThreshold);
        solver.setopt_real("AugLagUpdateGradientRelTol", AugLagUpdateGradientRelTol);

        solver.setopt_string("ConvergenceCheck", ConvergenceCheck);
        solver.setopt_real("ConvergenceGradientRelTol", ConvergenceGradientRelTol); 

        // print options and parameters
        solver.printopt();
        solver.printparam();           

        // MPC loop 
        iMPC = 1;
        // test
        problem.updateTrajData(filename1, filename2, iMPC);
        RCLCPP_INFO(this->get_logger(), "test101");
    }
     ~MpcController()
    {
        if (iMPC>0)
        {
            auto average_duration = total_duration_ /iMPC;
            RCLCPP_INFO(this->get_logger(), "MPC average execution time: %.2f ms", average_duration);             

        }
    }

private:
    // Callback function for subscription - get state x and publish u 
    void topic_callback(const px4_msgs::msg::PixhawkToRaspberryPi::SharedPtr msg)
    {

        // get state date and transfer it to double
        std::array<float, 9> state_float;
        for (size_t i = 0; i < 9; ++i) {
            state_float[i] = msg->msg_payload[i];  // 
        }

        std::vector<double> state_double(9);
        for (size_t i = 0; i < 9; ++i) {
            state_double[i] = static_cast<double>(state_float[i]);  // 
        }
        double* state_ptr = state_double.data();

        //get the test signal and timestamp
        std::array<float, 7> test_signal;
        test_signal[0]= iMPC;
        for (size_t i = 1; i < 7; ++i) {
            test_signal[i] = msg->msg_payload[i+9];  // 
        }

        uint64_t timestamp = msg->timestamp;

        //print the current state
        /*
        std::ostringstream state_stream;
        state_stream << "Current state: ";
        for (size_t i = 0; i < 9; ++i) {
            state_stream << state_double[i] << " ";  
        }
        RCLCPP_INFO(this->get_logger(), "%s", state_stream.str().c_str());  
        */
 

        // set the current state in solver
        solver.setparam_real_vector("x0", state_ptr);
        
        //update trajectory 
        problem.updateTrajData(filename1, filename2, iMPC);
       // Start measuring time
        auto start_time = std::chrono::high_resolution_clock::now();   
        solver.run();
        
        double* u_next = solver.getSolution()->unext;
        publish_message(u_next,test_signal, timestamp);
        // Stop measuring time
        auto end_time = std::chrono::high_resolution_clock::now();
        // Calculate duration in milliseconds
        //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::chrono::duration<double, std::milli> fp_ms = end_time - start_time;
        double duration = fp_ms.count();

        // print the control input
        std::ostringstream u_next_stream;
        u_next_stream << "Control input: ";
        for (size_t i = 0; i < 8; ++i) {
            u_next_stream << u_next[i] << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", u_next_stream.str().c_str());

        // Print the calulation time
        RCLCPP_INFO(this->get_logger(), "MPC took %.2f ms to execute.", duration); 
        total_duration_ += duration;

        double T_sim = iMPC * problem.dt;
        RCLCPP_INFO(this->get_logger(), "Current iMPC value: %d", iMPC);
        // wirte state control and coump_time into txt files
        if (state_file_.is_open()) {
            for (size_t i = 0; i < 9; ++i) {
                state_file_ << state_double[i];
                if (i < 8) state_file_ << "\t"; 
            }
            state_file_ << "\n";
        }
        if (control_file_.is_open()) {
            for (size_t i = 0; i < 8; ++i) {
                control_file_ << u_next[i];
                if (i < 7) control_file_ << "\t";
            }
            control_file_ << "\n";
        }
        if (compute_time_file_.is_open()) {
            compute_time_file_ << iMPC  << "\t" << duration<< "\n";
        }
        if (T_sim >= 45) {
            RCLCPP_INFO(this->get_logger(), "simulation time limited. Shutting down the node.");
            rclcpp::shutdown();  
        } 
        iMPC++;


    }

    // Function to publish the message
    void publish_message(double* u_next, std::array<float,7> test_signal, uint64_t timestamp)
    {
        auto msg = px4_msgs::msg::RaspberryPiToPixhawk();
        for (size_t i = 0; i < 8; ++i) {
        msg.msg_payload[i] = static_cast<float>(u_next[i]);  // u_next = msg_payload [0],...,[7]
        }

        for (size_t i = 9; i < 16; ++i) {
        msg.msg_payload[i] = test_signal[i-9];  // test_signal = msg_payload [9],...,[15]
        }
        msg.timestamp = timestamp; // timestamp 
        publisher_->publish(msg);        
    }
    // Function to publish the message
    void test_publish_message(const px4_msgs::msg::PixhawkToRaspberryPi::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "starting communication test ...");
        auto msg_out = px4_msgs::msg::RaspberryPiToPixhawk();
        for (size_t i = 0; i < 16; ++i) {
        msg_out.msg_payload[i] = static_cast<float>(msg->msg_payload[i]);  // u_next = msg_payload [0],...,[7]
        }
        publisher_->publish(msg_out);
       // if (iMPC >= 50) {
       //     rclcpp::shutdown();  
       //     RCLCPP_INFO(this->get_logger(), "communication test ended");
       // }
        iMPC++;        
    }

    rclcpp::Subscription<px4_msgs::msg::PixhawkToRaspberryPi>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::RaspberryPiToPixhawk>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    UAVModel problem;
    grampc::Grampc solver;
    std::string filename1_;
    std::string filename2_;
    const char* filename1; // x_taj.txt
    const char* filename2; // u_traj.txt
    std::ofstream state_file_;
    std::ofstream control_file_;
    std::ofstream compute_time_file_;
    int iMPC;
    double total_duration_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MpcController>());
    rclcpp::shutdown();
    return 0;
}
