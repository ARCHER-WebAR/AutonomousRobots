#include <chrono> // date and time utilities library
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include "std_srvs/srv/set_bool.hpp"


class TurtleBot4Prac1 : public rclcpp::Node
{
public:
  //public constructor names the node as "tb4_prac1", and initialise lights_on_ to "false"
  TurtleBot4Prac1(): Node("tb4_prac1")
  {
    //Create a server with service name as "lightring_service"
    service_ = this->create_service<std_srvs::srv::SetBool>(
      "lightring_service",
      std::bind(&TurtleBot4Prac1::service_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    // Create a publisher for the /cmd_lightring topic
    lightring_publisher_ = this->create_publisher<irobot_create_msgs::msg::LightringLeds>(
      "/cmd_lightring",
      rclcpp::SensorDataQoS());
}

private:
  // Perform this function when service is request.
  void service_callback(std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    // Create a ROS 2 message
    auto lightring_msg = irobot_create_msgs::msg::LightringLeds();
    // Stamp the message with the current time
    lightring_msg.header.stamp = this->get_clock()->now();
    // Lights are currently off
    if (request->data) {
      RCLCPP_INFO(this->get_logger(), "Incoming LightRing Overwrite Request Received");
      // Override system lights
      lightring_msg.override_system = true;
      // LED 0
      lightring_msg.leds[0].red = 255;
      lightring_msg.leds[0].blue = 0;
      lightring_msg.leds[0].green = 0;
      // LED 1
      lightring_msg.leds[1].red = 0;
      lightring_msg.leds[1].blue = 255;
      lightring_msg.leds[1].green = 0;
      // LED 2
      lightring_msg.leds[2].red = 0;
      lightring_msg.leds[2].blue = 0;
      lightring_msg.leds[2].green = 255;
      // LED 3
      lightring_msg.leds[3].red = 255;
      lightring_msg.leds[3].blue = 255;
      lightring_msg.leds[3].green = 0;
      // LED 4
      lightring_msg.leds[4].red = 255;
      lightring_msg.leds[4].blue = 0;
      lightring_msg.leds[4].green = 255;
      // LED 5
      lightring_msg.leds[5].red = 0;
      lightring_msg.leds[5].blue = 255;
      lightring_msg.leds[5].green = 255;
      
      // Publish the message
      response->success = true;
      response->message = "LightRing Overwrite Enabled";
      RCLCPP_INFO(this->get_logger(), "LightRing Overwrite Enabled");
    }
    else{
      RCLCPP_INFO(this->get_logger(), "Incoming Cancelling LightRing Overwrite Request");
      // Disable system override. The system will take back control of the lightring.
      lightring_msg.override_system = false;
      response->success = true;
      response->message = "LightRing Overwrite Canncelled";
      RCLCPP_INFO(this->get_logger(), "LightRing Overwrite Canncelled");
    }
    // Publish the message
    lightring_publisher_->publish(lightring_msg);
  }
      
  // Lightring Publisher
  rclcpp::Publisher<irobot_create_msgs::msg::LightringLeds>::SharedPtr lightring_publisher_;
  
  //Service server
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv); // initialise ROS 2
	rclcpp::spin(std::make_shared<TurtleBot4Prac1>()); // starts processing data from the node, include subscriber callback
	rclcpp::shutdown();
    return 0;
}
