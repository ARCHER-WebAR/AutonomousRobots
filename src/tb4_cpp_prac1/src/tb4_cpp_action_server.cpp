#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/action/drive_distance.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TurtleBot4Prac1 : public rclcpp::Node
{
public:
  using Drive_Distance = irobot_create_msgs::action::DriveDistance;
  using GoalHandleDriveDistance =
    rclcpp_action::ServerGoalHandle<Drive_Distance>;

  explicit TurtleBot4Prac1(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("tb4_prac1_action_server", options)
  {
    // Publisher of topic /cmd_vel
    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      rclcpp::SystemDefaultsQoS());
    using namespace std::placeholders;

    // Subscribe to the /odom topic
    this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&TurtleBot4Prac1::odom_callback, this, _1)
    );

    // Action server with name as "drive_distance_prac1"
    this->action_server_ = rclcpp_action::create_server<Drive_Distance>(
      this,
      "drive_distance_prac1",
      std::bind(&TurtleBot4Prac1::handle_goal, this, _1, _2),
      std::bind(&TurtleBot4Prac1::handle_cancel, this, _1),
      std::bind(&TurtleBot4Prac1::handle_accepted, this, _1));
  }

private:
  // Define drive distance server
  rclcpp_action::Server<Drive_Distance>::SharedPtr action_server_;
  // Define a command velocity publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // Define drive distance subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  // odometry pointer
  nav_msgs::msg::Odometry::SharedPtr odom_;
  // Callback function for handling goals
  rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Drive_Distance::Goal> goal);
  // Callback function for handling cancellation:
  rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandleDriveDistance> goal_handle);
  // Callback function for handling goal accept
  void handle_accepted(const std::shared_ptr<GoalHandleDriveDistance> goal_handle);
  // Action processing and update
  void execute(const std::shared_ptr<GoalHandleDriveDistance> goal_handle);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
};

void TurtleBot4Prac1::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  odom_ = odom_msg;
}

// Callback function for handling goals
rclcpp_action::GoalResponse TurtleBot4Prac1::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Drive_Distance::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(),
    "Received goal request with travel distance at %f m and maximum speed at %f m/s",
    goal->distance,
    goal->max_translation_speed);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Callback function for handling cancellation
rclcpp_action::CancelResponse TurtleBot4Prac1::handle_cancel(
  const std::shared_ptr<GoalHandleDriveDistance> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// Callback function for handling accepted goals
void TurtleBot4Prac1::handle_accepted(
  const std::shared_ptr<GoalHandleDriveDistance> goal_handle)
{
  using namespace std::placeholders;

  // this needs to return quickly to avoid blocking the executor
  std::thread{
    std::bind(&TurtleBot4Prac1::execute, this, _1),
    goal_handle}.detach();
}

// Action processing and update
void TurtleBot4Prac1::execute(const std::shared_ptr<GoalHandleDriveDistance> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal(); //get goal request

  //create feedback message and create result message
  auto feedback = std::make_shared<Drive_Distance::Feedback>(); 
  auto & remaining_travel_distance =
    feedback->remaining_travel_distance;

  auto result = std::make_shared<Drive_Distance::Result>();

  //velocity command in linear and angular
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.set__x(goal->max_translation_speed);

  int pub_freq = 100; //publish distance 100/sec

  rclcpp::Rate loop_rate(pub_freq);

  int count =
    int(pub_freq * goal->distance / goal->max_translation_speed); //number of loops

  geometry_msgs::msg::PoseStamped pose_stamped;

  for (int i = 0; (i < count) && rclcpp::ok(); ++i) { //run until loops done

    pose_stamped.header = odom_->header;
    pose_stamped.pose = odom_->pose.pose;

    if (goal_handle->is_canceling()) {
      result->set__pose(pose_stamped);
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    remaining_travel_distance = //update feedback
      goal->distance -
      goal->max_translation_speed * i / pub_freq;

    // Publish the command velocity
    cmd_vel_publisher_->publish(cmd_vel);

    // Publish feedback
    goal_handle->publish_feedback(feedback);

    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->set__pose(pose_stamped);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TurtleBot4Prac1>());

  rclcpp::shutdown();

  return 0;
}
