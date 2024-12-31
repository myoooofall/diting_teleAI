
#include <unistd.h>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;
// Create a soprt_request class for soprt commond request
class soprt_request : public rclcpp::Node
{
public:
    soprt_request() : Node("req_sender")
    {
        // the state_suber is set to subscribe "sportmodestate" topic
        state_suber = this->create_subscription<unitree_go::msg::SportModeState>(
            "sportmodestate", 10, std::bind(&soprt_request::state_callback, this, _1));
        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
        instruction_suber = this->create_subscription<std_msgs::msg::String>(
            "instruction", 3, std::bind(&soprt_request::instruction_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(dt * 1000)), std::bind(&soprt_request::timer_callback, this));
        // navigation_suber = this->create_subscription<std_msgs::msg::String>(
        //     "navigation", 10, std::bind(&soprt_request::navigation_callback, this, _1));
        navigation_puber = this->create_publisher<std_msgs::msg::Int32>("destination", 10);
        t = -1; // Runing time count
    };

private:
    void timer_callback()
    {
        req_puber->publish(history_req);
        
    };
    void navigation_callback(std_msgs::msg::Int32::SharedPtr data)
    {
       navigation_puber->publish(data->data);
    }
    void instruction_callback(std_msgs::msg::String::SharedPtr data)
    {   std::string instruction = data->data;
	std::cout<<instruction<<std::endl;
        
        if(instruction.find("前") != std::string::npos){
            sport_req.Move(req,0.5,0, 0);
            std::cout <<"diting is operating 向前"<< std::endl;
        }
        else if (instruction.find("向后") != std::string::npos) { 
            sport_req.Move(req, -0.5, 0, 0); 
            std::cout <<"diting is operating 向后"<< std::endl;
        }
        else if(instruction.find("向左") != std::string::npos){
            sport_req.Move(req, 0, 0.5, 0);
            std::cout <<"diting is operating 向左"<< std::endl;
        }
        else if(instruction.find("向右") != std::string::npos){
            sport_req.Move(req, 0, -0.5, 0);
            std::cout <<"diting is operating 向右"<< std::endl;
        }
        else if(instruction.find("左转") != std::string::npos){
            sport_req.Move(req, 0, 0, 0.5);
            std::cout <<"diting is operating 左转"<< std::endl;
        }
        else if(instruction.find("右转") != std::string::npos){
            sport_req.Move(req, 0, 0, -0.5);
            std::cout <<"diting is operating 右转"<< std::endl;
        }
        else if(instruction.find("停") != std::string::npos){
            sport_req.StopMove(req);
            std::cout <<"diting is operating 停止"<< std::endl;
        }
        else if(instruction.find("坐") != std::string::npos){
            sport_req.Sit(req);
            std::cout <<"diting is operating 坐下"<< std::endl;
        }
        else if(instruction.find("站起") != std::string::npos){
            sport_req.RecoveryStand(req);

            std::cout <<"diting is operating 站起"<< std::endl;
        }
        else if(instruction.find("跳舞") != std::string::npos){
            sport_req.Dance1(req);
  
            std::cout <<"diting is operating 跳舞"<< std::endl;
        }
        else if(instruction.find("懒腰") != std::string::npos){
            sport_req.Stretch(req);
            std::cout <<"diting is operating 伸懒腰"<< std::endl;
        }
        else if(instruction.find("招呼") != std::string::npos){
            sport_req.Hello(req);
            std::cout <<"diting is operating 打招呼"<< std::endl;
        }
	    else if(instruction.find("解除") != std::string::npos){
                sport_req.BalanceStand(req);
                std::cout <<"diting is operating 解除锁定"<< std::endl;
            }
        
        else if(instruction.find("旗子") != std::string::npos){
            std::cout<<"diting is operating 旗子"<<std::endl;
            navigation_callback(1);
        }
        else if(instruction.find("灯塔") != std::string::npos){
            std::cout<<"diting is operating 灯塔"<<std::endl;
            navigation_callback(2);
        }
        else if(instruction.find("停止导航") != std::string::npos){
            std::cout<<"停止导航"<<std::endl;
            navigation_callback(0);

        }
	history_req=req;
    
    }
    void state_callback(unitree_go::msg::SportModeState::SharedPtr data)
    {
        // Get current position of robot when t<0
        // This position is used as the initial coordinate system

        if (t < 0)
        {
            // Get initial position
            px0 = data->position[0];
            py0 = data->position[1];
            yaw0 = data->imu_state.rpy[2];
            //std::cout << px0 << ", " << py0 << ", " << yaw0 << std::endl;
        }
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr state_suber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr instruction_suber;
    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    //rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_suber;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr navigation_puber;
    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    unitree_api::msg::Request history_req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    double t; // runing time count
    double dt = 0.002; //control time step
    bool navigation_flag = false;
    double px0 = 0;  // initial x position
    double py0 = 0;  // initial y position
    double yaw0 = 0; // initial yaw angle
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    rclcpp::TimerBase::SharedPtr timer_; // Create a timer callback object to send sport request in time intervals

    rclcpp::spin(std::make_shared<soprt_request>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
