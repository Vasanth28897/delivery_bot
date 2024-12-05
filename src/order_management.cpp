#include "delivery_bot/order_management.hpp"

OrderManagement::OrderManagement() : Node("order_management")
{
  ordered_tables_publishers_ = this->create_publisher<std_msgs::msg::Int32>("ordered_tables", 1); 

  kitchen_confirmation_response_ = this->create_subscription<std_msgs::msg::String>(
                                    "/kitchen_confirmation_response", 10, std::bind(&OrderManagement::kitchen_confirmation_response_callback, this, std::placeholders::_1));
  table_confirmation_response_ = this->create_subscription<std_msgs::msg::String>(
                                    "/table_confirmation_response", 10, std::bind(&OrderManagement::table_confirmation_response_callback, this, std::placeholders::_1));

  place_order_service_ = this->create_service<delivery_bot::srv::PlaceOrder>(
                          "place_order", std::bind(&OrderManagement::place_order_check, this, std::placeholders::_1, std::placeholders::_2));

  cancel_order_service_ = this->create_service<delivery_bot::srv::CancelOrder>(
                          "cancel_order", std::bind(&OrderManagement::cancel_order_check, this, std::placeholders::_1, std::placeholders::_2));

  navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&OrderManagement::publish_ordered_tables, this));
  max_tables_ = 3;
  order_size_ = 0;

  home_ = {6.106026, -2.914490};   
  kitchen_ = {4.5, 0.0}; 
  table_positions_ = {
    {1, {-1.5, 1.5}},    
    {2, {-2.5, -0.5}},    
    {3, {-1.5, -3.0}}
  };

  RCLCPP_INFO(this->get_logger(), "waiting for Orders...");
}

void OrderManagement::print_remaining_orders()
{
    RCLCPP_INFO(this->get_logger(), "Currently unserved orders: %d", order_size_);
}

void OrderManagement::print_order_queue()
{
  std::queue<int32_t> temp_queue = order_queue_;  
  std::stringstream queue_contents;
  
  while (!temp_queue.empty())
  {
    queue_contents << temp_queue.front() << " ";
    temp_queue.pop();
  }
    
  RCLCPP_INFO(this->get_logger(), "Order Queue: %s", queue_contents.str().c_str());
}

void OrderManagement::publish_ordered_tables()
{
  std_msgs::msg::Int32 msg;
  msg.data = order_size_;
  ordered_tables_publishers_->publish(msg);
}

void OrderManagement::place_order_check(const std::shared_ptr<delivery_bot::srv::PlaceOrder::Request> request,
                                          std::shared_ptr<delivery_bot::srv::PlaceOrder::Response> response)
{
  table_number_ = request->table_number;
  if(table_number_ < 1 || table_number_ > max_tables_)
  {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "There are only %d tables in the restaurant.", max_tables_);
    return;
  }
  if (order_queue_.size() > 2)
  {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "Order limit reached. Only %d orders can be placed at a time.", max_tables_);
    return;
  }

  order_queue_.push(table_number_);
  order_size_ = order_queue_.size();
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Received Order from Table number : %d", table_number_);
  RCLCPP_INFO(this->get_logger(), "Currently ordered tables: %zu", order_queue_.size());
  print_remaining_orders();

  if(order_size_ == 1)
  {
    move_to_kitchen();
  }

  print_order_queue();
  publish_ordered_tables();
}

void OrderManagement::cancel_order_check(const std::shared_ptr<delivery_bot::srv::CancelOrder::Request> request,
                                           std::shared_ptr<delivery_bot::srv::CancelOrder::Response> response)
{
  int32_t table_number = request->table_number;

  std::queue<int> temp_queue;
  found = false;
  while (!order_queue_.empty())
  {
    int current_order = order_queue_.front();
    order_queue_.pop();
    if (current_order == table_number)
    {
      found = true;
      order_size_--;
      if(table_number == next_table)
      {
        move_to_kitchen();
        order_size_--;
      }
    }
    else
    {
      temp_queue.push(current_order);
    }
  }
  if (!found)
  {
    RCLCPP_WARN(this->get_logger(), "Table number %d has not placed any order yet.", table_number);
    response->success = false;
  }
  order_queue_ = temp_queue;
  order_size_ = order_queue_.size();
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "Canceled order from Table number: %d", table_number);
  RCLCPP_INFO(this->get_logger(), "Currently ordered tables: %d", order_size_);
  print_remaining_orders();
  print_order_queue();
  publish_ordered_tables();
}

void OrderManagement::move_to_kitchen()
{
  RCLCPP_INFO(this->get_logger(), "Moving to Kitchen...");

  if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available!");
    return;
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = kitchen_.first;
  goal_pose.pose.position.y = kitchen_.second;
  goal_pose.pose.position.z = 0.0; 
  goal_pose.pose.orientation.z = 0.7071; // 90 degree rotation around z-axis in counter-clock-wise direction
  goal_pose.pose.orientation.w = 1.0;
  /** (yaw)z = 0.7071 - rotate the robot around the z - axis by 90 degree counterclockwise direction theta = 90 = pi/2. 
   * so, the formula to rotate around z axis is sin(pi/4) = sin(90/4) = 0.7071.
  **/ 

  send_goal_to_nav2(goal_pose);
  std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback_;
  if(feedback_)
  {
    RCLCPP_INFO(this->get_logger(), "feedback triggered");
  }
  else
  {
   RCLCPP_INFO(this->get_logger(), "feedback not triggered");
  }
}

void OrderManagement::kitchen_confirmation_response_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string response = msg->data;
  if(response == "yes")
  {
    next_table = order_queue_.front();      
    RCLCPP_INFO(this->get_logger(), "Received confirmation from the kitchen. Moving to table.");
    move_to_table(next_table);
    print_order_queue();
  }
  else if(response == "no")
  {
    RCLCPP_WARN(this->get_logger(), "There are no orders placed, returning to home.");
    if(order_size_ != 0)
    {
      RCLCPP_INFO(this->get_logger(), "Still there are orders to serve.");
    }
    else
    {
      return_home();
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Received invalid confirmation response.");
  }
}

void OrderManagement::move_to_table(int table_number)
{
  if (order_size_ == 0) 
  {
    RCLCPP_INFO(this->get_logger(), "No more orders to serve.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Moving to table %d...", table_number);
  geometry_msgs::msg::PoseStamped goal_pose;

  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = table_positions_[table_number].first;
  goal_pose.pose.position.y = table_positions_[table_number].second;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation.z = (table_number == 2) ? 0.0 : 3.140074; // 3.140074 - 180 degree rotation around z-axis in counter-clock-wise-direction
  goal_pose.pose.orientation.w = 1.0;

  send_goal_to_nav2(goal_pose);
}

void OrderManagement::table_confirmation_response_callback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string response = msg->data;
  if(response == "yes")
  {
    int current_order_ = order_queue_.front();
    RCLCPP_INFO(this->get_logger(), "Received confirmation from the table %d ", current_order_);
    order_queue_.pop();
    print_order_queue();
    order_size_ = order_queue_.size();
    print_remaining_orders();
    move_to_kitchen();
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Received invalid confirmation response.");
  }
}

void OrderManagement::return_home()
{
  RCLCPP_INFO(this->get_logger(), "Returning to home...");
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.pose.position.x = home_.first;
  goal_pose.pose.position.y = home_.second;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation.z = 3.140074;
  goal_pose.pose.orientation.w = 1.0;

  send_goal_to_nav2(goal_pose);
}

void OrderManagement::send_goal_to_nav2(const geometry_msgs::msg::PoseStamped goal_pose) 
{
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = goal_pose;
  RCLCPP_INFO(this->get_logger(), "Sending goal to navigation: (%.2f, %.2f)", goal_pose.pose.position.x, goal_pose.pose.position.y);
  
  if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return;
  }
  
  send_goal_options.goal_response_callback = std::bind(&OrderManagement::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&OrderManagement::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  navigation_client_->async_send_goal(goal_msg, send_goal_options);
}

void OrderManagement::goal_response_callback(
  const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) 
{
  if (goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED) {
    RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Goal failed!");
  }
}

void OrderManagement::feedback_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                                        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Feedback received: %.2f, %.2f", feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);  

  double distance_to_kitchen = std::sqrt(std::pow(feedback->current_pose.pose.position.x - kitchen_.first, 2) +
                                        std::pow(feedback->current_pose.pose.position.y - kitchen_.second, 2));

  if (distance_to_kitchen < 0.1 && !has_reached_kitchen) 
  {
    RCLCPP_INFO(this->get_logger(), "Robot has reached the kitchen!");
    has_reached_kitchen = true;  
    RCLCPP_INFO(this->get_logger(), "Waiting for kitchen confirmation...");
  }

  double distance_to_table = std::sqrt(std::pow(feedback->current_pose.pose.position.x - table_positions_[table_number_].first, 2) +
                                      std::pow(feedback->current_pose.pose.position.y - table_positions_[table_number_].second, 2));

  if (distance_to_table < 0.1 && !has_reached_table)  
  {
    RCLCPP_INFO(this->get_logger(), "Robot has reached the table %d", table_number_);
    has_reached_table = true; 
    RCLCPP_INFO(this->get_logger(), "Waiting for table%d confirmation...", table_number_);
  }

  double distance_to_home = std::sqrt(std::pow(feedback->current_pose.pose.position.x - home_.first, 2) +
                                      std::pow(feedback->current_pose.pose.position.y - home_.second, 2));

  if (distance_to_home < 0.1 && !has_reached_home) 
  {
    RCLCPP_INFO(this->get_logger(), "Robot has reached home!");
    has_reached_home = true;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OrderManagement>(); 
  rclcpp::executors::MultiThreadedExecutor executor; 
  executor.add_node(node); 
  executor.spin();
  rclcpp::shutdown();
  return 0;
}