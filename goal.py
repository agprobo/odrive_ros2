#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/u_int8.hpp"

class NavigateToPoseClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseClient()
    : Node("navigate_to_pose_client")
  {
    // Create action client
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Create subscriptions for goal and cancel topics
    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/my_goal_pose", 10,
      std::bind(&NavigateToPoseClient::onGoalReceived, this, std::placeholders::_1));
      
    cancel_subscription_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/cancel_from_web_UI", 10,
      std::bind(&NavigateToPoseClient::onCancelReceived, this, std::placeholders::_1));

    // Wait for action server
    RCLCPP_INFO(this->get_logger(), "Waiting for the 'navigate_to_pose' action server...");
    client_->wait_for_action_server();
    RCLCPP_INFO(this->get_logger(), "'navigate_to_pose' action server is ready.");
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  GoalHandleNavigateToPose::SharedPtr active_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cancel_subscription_;

  void onGoalReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received a goal from the web UI.");
    
    // Construct the goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = *msg;

    // Send the goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigateToPoseClient::onGoalResponse, this, std::placeholders::_1);
    send_goal_options.result_callback =
      std::bind(&NavigateToPoseClient::onResult, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending the goal...");
    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void onCancelReceived(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received a cancel request from the web UI.");

    if (!active_goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
      return;
    }

    auto future_cancel = client_->async_cancel_goal(active_goal_handle_);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_cancel);
    RCLCPP_INFO(this->get_logger(), "Goal cancellation requested.");
  }

  void onGoalResponse(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server.");
    active_goal_handle_ = goal_handle;
  }

  void onResult(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
        break;
    }

    active_goal_handle_.reset();  // Clear active goal handle
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigateToPoseClient>());
  rclcpp::shutdown();
  return 0;
}
