
#include <algorithm>
#include <memory>
#include <rclcpp/client.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_srvs/srv/detail/set_bool__struct.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "visibility_control.h"

namespace robot_interface_proxy {


template<class T>
class VariableWatcher {
public:
  VariableWatcher(rclcpp::Duration timeout, rclcpp::Clock::SharedPtr clock)
    : clock_{clock}, timeout_{timeout}, has_changed_{false} {};

  void feed(T v) {
    if (!raw_value.has_value() || raw_value != v) {
      has_changed_ = true;
    }
    raw_value = v;
  }

  bool is_timeout() {
    if (!last_update_.has_value()) {
      return true;
    }
    if (clock_->now() > last_update_.value() + timeout_) {
      return true;
    }
    return  false;
  }

  bool has_changed() {
    return has_changed_;
  }

  void reset_changed_flag() {
    has_changed_ = false;
  }

  void reset_timeout() {
    last_update_ = clock_->now();
  }

  bool has_changed_or_timeout() {
    return raw_value.has_value() && (has_changed() || is_timeout());
  }

  bool has_value() { return raw_value.has_value(); }

  const T& get_value() { return raw_value.value(); }

  std::optional<T> raw_value;
private:
  rclcpp::Clock::SharedPtr clock_;
  std::optional<rclcpp::Time> last_update_;
  rclcpp::Duration timeout_;
  bool has_changed_;
};


class ProxyBase : public rclcpp_lifecycle::LifecycleNode {
public:
  ProxyBase(std::string node_name);
  virtual ~ProxyBase();

  virtual bool configure_interface();
  virtual bool activate_interface();
  virtual bool deactivate_interface();
  virtual bool cleanup_interface();
  virtual bool shutdown_interface();

  virtual bool read_interface();
  virtual bool write_interface();

  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  virtual rclcpp_lifecycle::LifecycleNode::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

  bool is_activated() { return is_activated_; }

  // to publish
  std::unique_ptr<VariableWatcher<std_msgs::msg::Int64>> ammo_watcher_;

  // to subscribe
  std::unique_ptr<VariableWatcher<geometry_msgs::msg::Twist>> target_vel_watcher_;
  std::unique_ptr<VariableWatcher<geometry_msgs::msg::Vector3>> camera_angle_watcher_;
  std::unique_ptr<VariableWatcher<bool>> fire_command_watcher_;

private:
  void read_callback();
  void write_callback();

  bool is_activated_ = false;

  std::optional<rclcpp::Time> last_target_vel_received_;

  rclcpp::TimerBase::SharedPtr read_timer_, write_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_vel_sub_;
  // geometry_msgs::msg::Twist target_vel_;
  // rclcpp::Time time_last_target_vel_received_;
  // bool is_target_vel_expired_ = true;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr camera_angle_sub_;
  // geometry_msgs::msg::Vector3 camera_angle_;

  // rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr current_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int64>::SharedPtr ammo_pub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr fire_command_service_;
};


}  // namespace robot_interface_proxy