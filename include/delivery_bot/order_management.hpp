#ifndef ORDER_MANAGEMENT_HPP
#define ORDER_MANAGEMENT_HPP

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <queue>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include "delivery_bot/srv/place_order.hpp"
#include "delivery_bot/srv/cancel_order.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

class OrderManagement : public rclcpp::Node
{
public:
  OrderManagement();  

private:
  int max_tables_;
  int next_table;
  int32_t table_number_;
  int order_size_;
  bool found;
  bool has_reached_kitchen = false;
  bool has_reached_table = false;
  bool has_reached_home = false;
  std::queue<int32_t> order_queue_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ordered_tables_publishers_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kitchen_confirmation_response_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr table_confirmation_response_;
  rclcpp::Service<delivery_bot::srv::PlaceOrder>::SharedPtr place_order_service_;
  rclcpp::Service<delivery_bot::srv::CancelOrder>::SharedPtr cancel_order_service_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;
  std::pair<double, double> home_;
  std::pair<double, double> kitchen_;
  std::unordered_map<int, std::pair<double, double>> table_positions_;

  void print_remaining_orders();
  void print_order_queue();
  void publish_ordered_tables();
  void place_order_check(const std::shared_ptr<delivery_bot::srv::PlaceOrder::Request> request,
                          std::shared_ptr<delivery_bot::srv::PlaceOrder::Response> response);
  void cancel_order_check(const std::shared_ptr<delivery_bot::srv::CancelOrder::Request> request,
                            std::shared_ptr<delivery_bot::srv::CancelOrder::Response> response);
  void move_to_kitchen();
  void kitchen_confirmation_response_callback(const std_msgs::msg::String::SharedPtr msg);
  void move_to_table(int table_number);
  void table_confirmation_response_callback(const std_msgs::msg::String::SharedPtr msg);
  void return_home();
  void send_goal_to_nav2(const geometry_msgs::msg::PoseStamped goal_pose);
  void goal_response_callback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle);
  void feedback_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                          const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

};

#endif // ORDER_MANAGEMENT_HPP
