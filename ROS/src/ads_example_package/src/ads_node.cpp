#include <iostream>
#include <string>
#include "AdsLib.h"
#include "AdsVariable.h"

#include <thread>
#include <chrono>

#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "crane_interfaces/msg/pressure_message.hpp"
#include "crane_interfaces/msg/visualisation.hpp"
#include "crane_interfaces/msg/motion_reference.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace craneads {

    struct AdsVariables
    {
        AdsVariables() = delete;

        explicit AdsVariables(AdsDevice& route)
            : activateMotion{route, "MAIN.bActivateMotion"}
            , velocityReference{route, "MAIN.fVelRef"}
            , positionReference{route, "MAIN.fPosRef"}
            , boomAngle{route, "MAIN.fBoomAngle"}
            , PSPressureMeasurement{route, "MAIN.fPSPressureMeasurement"}
            , RSPressureMeasurement{route, "MAIN.fRSPressureMeasurement"}
        {
            // Do nothing.
        }

        AdsVariable<bool> activateMotion;
        AdsVariable<double> velocityReference;
        AdsVariable<double> positionReference;
        AdsVariable<double> boomAngle;
        AdsVariable<double> PSPressureMeasurement;
        AdsVariable<double> RSPressureMeasurement;
    };

    class AdsHandler : public rclcpp::Node
    {
    public:
        explicit AdsHandler(const AmsNetId remoteNetId, const std::string remoteIpV4)
            : remoteNetId_(remoteNetId)
            , remoteIpV4_(remoteIpV4)
            , route_{remoteIpV4_, remoteNetId_, AMSPORT_R0_PLC_TC3}
            , ads_(route_), Node("AdsHandler") { }

        //AdsHandler() : AdsHandler({192, 168, 0, 10,  1, 1}, "192.168.0.10") { 
        AdsHandler() : AdsHandler({192, 168, 0, 15,  1, 1}, "192.168.0.15") { 

	std::cout << "Starting up ROS 2 ADS Node ..." << std::endl;
	
	//Subscriber
	motion_ref_sub_ = this->create_subscription<crane_interfaces::msg::MotionReference>("motion_reference", 10, std::bind(&AdsHandler::motion_ref_callback, this, _1));
	
	RCLCPP_INFO(this->get_logger(), "Subscribed to motion_reference topic");
	
	
	//Publisher
	
	//motion_publisher_ = this->create_publisher<std_msgs::msg::Float32>("motion_publisher", 10);
	//motion_timer_ = this->create_wall_timer(10ms, std::bind(&AdsHandler::motion_timer_callback, this));
	
	//state_publisher_ = this->create_publisher<std_msgs::msg::Float32>("state_publisher", 10);
	//state_publisher_ = this->create_publisher<crane_interfaces::msg::PressureMessage>("state_publisher", 10);
	state_publisher_ = this->create_publisher<crane_interfaces::msg::Visualisation>("state_publisher", 10);
	state_timer_ = this->create_wall_timer(10ms, std::bind(&AdsHandler::state_timer_callback, this));
	RCLCPP_INFO(this->get_logger(), "Publishing to state_publisher");
}
	std_msgs::msg::Float32 motion_message;
	//crane_interfaces::msg::PressureMessage state_message;
	crane_interfaces::msg::Visualisation state_message;
	
	
	
        void activateMotion()
        {
            ads_.activateMotion = true;
        }

        void deactivateMotion()
        {
            ads_.activateMotion = false;
        }

        void setVelocityReference(double value)
        {
            ads_.velocityReference = value;
        }

        void setPositionReference(double value)
        {
            ads_.positionReference = value;
        }

        double getAngleMeasurement()
        {
            return ads_.boomAngle;
        }
        
         double getPistonSidePressureMeasurement()
        {
            return ads_.PSPressureMeasurement;
        }
        
         double getRodSidePressureMeasurement()
        {
            return ads_.RSPressureMeasurement;
        }
	
//	void motion_timer_callback(){
//		double boom_angle = getAngleMeasurement();
//		
//		motion_message.data = (float) boom_angle;
//		motion_publisher_->publish(motion_message);
//
//		
//	}

	void motion_ref_callback(const crane_interfaces::msg::MotionReference::SharedPtr message) {
		double cyl_vel_ref = message->cyl_vel_ref;
		double cyl_pos_ref = message->cyl_pos_ref;
		bool start = message->start;
		bool stop = message->stop;
		//RCLCPP_INFO(this->get_logger(), "received Motion_Reference Message %f, %f", );
		if (start == true){
			activateMotion();	
		}
		
		if (stop == true) {
			deactivateMotion();
		}
		
		setVelocityReference(cyl_vel_ref);
		setPositionReference(cyl_pos_ref);
		
	}

	void state_timer_callback(){
		double boom_angle = getAngleMeasurement();
		double piston_pressure = getPistonSidePressureMeasurement();
		double rod_pressure = getRodSidePressureMeasurement();
		state_message.boom_angle = boom_angle;
		state_message.piston_pressure = piston_pressure;
		state_message.rod_pressure = rod_pressure;
		
		//state_message.data = (float) piston_pressure;
		state_publisher_->publish(state_message);
	}
	

        void printState()
        {
            const auto state = route_.GetState();
            std::cout << "ADS state: "
                      << std::dec << static_cast<uint16_t>(state.ads)
                      << " devState: "
                      << std::dec << static_cast<uint16_t>(state.device);
        }

        ~AdsHandler() { }
        
        //rclcpp::TimerBase::SharedPtr motion_timer_;
        rclcpp::TimerBase::SharedPtr state_timer_;
        
        rclcpp::Subscription<crane_interfaces::msg::MotionReference>::SharedPtr motion_ref_sub_;
        
        //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motion_publisher_;
        //rclcpp::Publisher<crane_interfaces::msg::PressureMessage>::SharedPtr state_publisher_;
        rclcpp::Publisher<crane_interfaces::msg::Visualisation>::SharedPtr state_publisher_;
        

    private:
        const AmsNetId remoteNetId_;
        const std::string remoteIpV4_;
        AdsDevice route_;
        AdsVariables ads_;
    };

}

using namespace std;

int main(int argc, char* argv[])
{

  //std::cout << "Example ROS2 ADS node starting up.." << std::endl;

  // Real lab PLC IP.
  //const AmsNetId remoteNetId { 192, 168, 0, 10, 1, 1 };
  //const std::string remoteIpV4 = "192.168.0.10";

  // Connecting to testbed computer.
  //const AmsNetId remoteNetId { 192, 168, 56, 1, 1, 1 };
  //const std::string remoteIpV4 = "192.168.56.1";

  //std::cout << "  Create AdsHandler.. ";
  //craneads::AdsHandler adsHandler(remoteNetId, remoteIpV4);
  //std::cout << "  OK" << std::endl;

  //adsHandler.deactivateMotion();

  //adsHandler.printState();

  //adsHandler.setVelocityReference(3.2);
  //std::this_thread::sleep_for (std::chrono::seconds(5));
  //adsHandler.setPositionReference(3.14);

  //adsHandler.activateMotion();
  //for(uint8_t n = 0; ; ++n)
  //{
  //  adsHandler.setPositionReference(static_cast<double>(n) / 255.0);
  //  std::cout << "Angle measurement from ADS: " << adsHandler.getAngleMeasurement() << std::endl;
  //  std::this_thread::sleep_for (std::chrono::seconds(2));
  //}
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<craneads::AdsHandler>());
	rclcpp::shutdown();

  return 0;
}
