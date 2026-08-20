/*
cd ~/tb4_ws
colcon build --packages-select tb4_cpp_prac3

cd ~/tb4_ws
source install/local_setup.bash
ros2 run tb4_cpp_prac3 wall_follower

ros2 param set /person_follower following_distance 1.0
ros2 param set /person_follower angle_control_gain_ 1.0
ros2 run tb4_cpp_prac3 person_follower --ros-args -p distance_control_gain:=2.0
*/

#include <chrono>  
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358
using rcl_interfaces::msg::ParameterType;


class WallFollower : public rclcpp::Node
{
public:
    WallFollower(): Node("wall_follower")
    {
        /*TODO TASK - MILESTONE # 4.1
            1. Declare all parameters used for configuring the "following distance", "following angle", and all control gains. Their default values should be given as well.
            2. Get all parameter values from the constructor, and save them to private class element variables.
            3. Print all parameter values here.
            4. Set the value of "following_angle_" after initialising all parameters
        */

        auto wall_side_desc = rcl_interfaces::msg::ParameterDescriptor{};
        wall_side_desc.description = "A positive value indicates that the wall will be on the left side of the robot, otherwise on the right";
        auto buffer_zone_desc = rcl_interfaces::msg::ParameterDescriptor{};
        buffer_zone_desc.description = "A positive value used to determine whether the tracking control is on or off";
        // Declare parameters
        this->declare_parameter<float>("following_distance", 0.7);
        this->declare_parameter<int8_t>("wall_side", 1, wall_side_desc); //1 is left
        this->declare_parameter<float>("buffer_zone", 0.4, buffer_zone_desc);
        this->declare_parameter<float>("forward_velocity", 0.4); 
        this->declare_parameter<float>("angle_control_gain_1", 1.0); //turning based on heading angle error
        this->declare_parameter<float>("angle_control_gain_2", 1.0); //turning based on distance error
        this->declare_parameter<float>("distance_control_gain", 0.5); //forward speed based on distance error
        // Get parameter values
        this->get_parameter("following_distance", following_distance_);
        this->get_parameter("wall_side", wall_side_);
        this->get_parameter("buffer_zone", buffer_zone_);
        this->get_parameter("forward_velocity", forward_velocity_);
        this->get_parameter("angle_control_gain_1", angle_control_gain_1_);
        this->get_parameter("angle_control_gain_2", angle_control_gain_2_);
        this->get_parameter("distance_control_gain", distance_control_gain_);
        // Print parameter values
        RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
        RCLCPP_INFO(this->get_logger(), "wall_side: %ld", wall_side_);
        RCLCPP_INFO(this->get_logger(), "buffer_zone: %.2f", buffer_zone_);
        RCLCPP_INFO(this->get_logger(), "forward_velocity: %.2f", forward_velocity_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_1: %.2f", angle_control_gain_1_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_2: %.2f", angle_control_gain_2_);
        RCLCPP_INFO(this->get_logger(), "distance_control_gain: %.2f", distance_control_gain_);

        if(wall_side_>0)
            following_angle_ = PI/2;
        else
            following_angle_ = -PI/2;


        /* TODO TASK - MILESTONE #4.3
            Initialise dynamic parameter handler by the rclcpp node method "add_on_set_parameters_callback"
        */
      dyn_params_handler_ = this->add_on_set_parameters_callback( //allow node to handle parameter
        std::bind(
        &WallFollower::dynamicParametersCallback, //changed wall
        this, std::placeholders::_1));


        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
             "/cmd_vel",
             rclcpp::SystemDefaultsQoS());
        using namespace std::placeholders;
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&WallFollower::scan_callback, this, _1)
        );
    }
private:
    std::recursive_mutex mutex_;
    // Define a command velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // Define a laser scan subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    /* TODO TASK - MILESTONE #4.2
        define dynamic parameter call back handle.
    */
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    /** 
     * @brief Callback executed when a parameter change is detected
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    double following_angle_;
    double following_distance_;
    int64_t wall_side_;
    double buffer_zone_;
    double forward_velocity_;
    double angle_control_gain_1_;
    double angle_control_gain_2_;
    double distance_control_gain_;
};

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    /*TODO TASKS
        MILESTONE # 6.1. Process the received scan_msg to get the location of the closest object in robot's environment. 
        NOTE: the four pillars of will be visible from the Lidar sensor, you have to remove the distance 
        measurements of these four pillars by ignoring any measurement less than 0.2 meter. 

        MILESTONE # 6.2. You have to calculate the bearing and the range of the closest object with respect to the robot frame. You have         
        to check the LaserScan message definition, and how the Lidar sensor is mounted with respective to  the robot's coordinate.

        MILESTONE # 6.3. Write a Wall Follow Reactive Control that takes the bearing and range information of the closest object in the environment 
        as the input and publish a message on topic /cmd_vel to control the motion of the robot. 
            3.1 If the robot is far away from the wall, it should move towards its nearest wall at a constant speed until the robot 
            arrives at a distance of desired value + buffer zone, with respect to its closest wall. 
            3.2 Next, the robot enter the wall follow mode with the control lawy in in Algorithm 1
            3.3 The robot should deal with corner cases by only using reactive control with properly tuned control gains. 
    */
    float min_distance = 100;
    float min_value = 100;
    int min_index = 0;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i){
      float tempdist = scan_msg->ranges[i];
      if (tempdist > 0.2 && tempdist < min_value){
          min_distance = tempdist;
          min_value = min_distance;
          // RCLCPP_INFO(this->get_logger(), "%.2f", min_value);
          min_index = i;
      }
    }
    
    // Use the index to calculate the angle where the smallest range is measured.
    float min_angle = (min_index - 320)*2*PI/640.0;

    geometry_msgs::msg::Twist cmd_vel_msg;

    /* 
    The magic number 12 is from the simulation setup, i.e., the range of the lidar sensor is from 0.164 to 12. 
    If min_value<12, the lidar sensor has a valid measurement. 
    */ 
    if(min_value<12) 
    {
        /* The robot is moving towards to the closed target at speed of forward_velocity_*/
        if(min_value>(following_distance_+buffer_zone_)){
            if(abs(min_angle)>PI/4.0){
                if(min_angle>PI/4.0)
                    cmd_vel_msg.angular.z = 1.0;
                else
                    cmd_vel_msg.angular.z = -1.0;
            }
            else{
                cmd_vel_msg.angular.z = 0;
                cmd_vel_msg.linear.x = forward_velocity_;
            }
        }
        // drive along the wall at a fixed distance
        else{
            double theta_tilde = min_angle - following_angle_;
            double d = min_value - following_distance_;

            cmd_vel_msg.linear.x = forward_velocity_;
            if(wall_side_>0) {
                if (std::abs(theta_tilde) > PI / 10.0) {
                cmd_vel_msg.angular.z = angle_control_gain_1_ * theta_tilde + angle_control_gain_2_ * d * (std::sin(theta_tilde) / theta_tilde);
                }
                else {
                    cmd_vel_msg.angular.z = angle_control_gain_1_ * theta_tilde + angle_control_gain_2_ * d;
                }
            }
            else {
                if (std::abs(theta_tilde) > PI / 10.0) {
                    cmd_vel_msg.angular.z = angle_control_gain_1_ * theta_tilde - angle_control_gain_2_ * d * (std::sin(theta_tilde) / theta_tilde);
                }
                else {
                    cmd_vel_msg.angular.z = angle_control_gain_1_ * theta_tilde - angle_control_gain_2_ * d;
                }
            }
        }
    }
    
    else // No valid measurement is available, move forward at a constant speed.
    {
        RCLCPP_INFO(this->get_logger(), "No Object is Detected");
        cmd_vel_msg.linear.x = forward_velocity_;
    }
    //publish the command velocity
    cmd_vel_publisher_->publish(cmd_vel_msg);
   
}


rcl_interfaces::msg::SetParametersResult 
WallFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters){
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    /*TODO TASK - MILESTONE #5.1 
      Check whether update of a parameter in the node is requested, if yes and save the updated
      parameter value.
    */
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "following_distance") {
        following_distance_ = parameter.as_double();
        if(following_distance_<0.0)
        {
          RCLCPP_WARN(this->get_logger(), "following_distance cannot set negative, set 0");
          following_distance_ = 0.0;
        }
      }
      else if (param_name == "buffer_zone") { //can be negative
        buffer_zone_ = parameter.as_double();
      }
      else if (param_name == "forward_velocity") { //cant be negative
        forward_velocity_ = parameter.as_double();
        if(forward_velocity_<0.0)
        {
          RCLCPP_WARN(this->get_logger(), "forward_velocity cannot set negative, set 0");
          forward_velocity_ = 0.0;
        }
      }
      else if (param_name == "angle_control_gain_1") { //can be negative
        angle_control_gain_1_ = parameter.as_double();
      }
      else if (param_name == "angle_control_gain_2") { //can be negative
        angle_control_gain_2_ = parameter.as_double();
      }
      else if (param_name == "distance_control_gain") { //can be negative
        distance_control_gain_ = parameter.as_double();
      }
    }
    else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == "wall_side") {
        wall_side_ = parameter.as_int();
        
        if(wall_side_>0)
            following_angle_ = PI/2;
        else
            following_angle_ = -PI/2;
      }
    }
  }
  result.successful = true;
  return result;

}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();
	return 0;
}
