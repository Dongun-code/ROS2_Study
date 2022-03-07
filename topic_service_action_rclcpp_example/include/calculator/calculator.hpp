#ifndef CALCULATOR__CALCULATOR_HPP_
#define CALCULATOR__CALCULATOR_HPP_
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "msg_srv_action_interface_example/msg/arithmetic_argument.hpp"
#include "msg_srv_action_interface_example/srv/arithmetic_operator.hpp"


class Calculator : public rclcpp::Node
{
public:
  using ArithmeticArgument = msg_srv_action_interface_example::msg::ArithmeticArgument;

  explicit Calculator(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Calculator();

  float calculate_given_formula(const float & a, const float & b, const int8_t & operators);

private:
  rclcpp::Subscription<ArithmeticArgument>::SharedPtr arithmetic_argument_subscriber_;

  float argument_a_;
  float argument_b_;

  int8_t argument_operator_;
  float argument_result_;

};
#endif  // CALCULATOR__CALCULATOR_HPP_
