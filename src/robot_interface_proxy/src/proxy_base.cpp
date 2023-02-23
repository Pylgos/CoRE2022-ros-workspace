#include "robot_interface_proxy/proxy_base.hpp"
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <rclcpp/qos.hpp>


using namespace std;
using namespace std::chrono;
using rclcpp_lifecycle::State;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using rclcpp::SensorDataQoS;
using rclcpp::SystemDefaultsQoS;
using rclcpp::Duration;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Int64;


namespace robot_interface_proxy {


ProxyBase::ProxyBase(string node_name) : LifecycleNode(node_name) {
  declare_parameter("read_rate", 100.0);
  declare_parameter("write_rate", 100.0);
  declare_parameter("target_vel_expire_duration", 1.0);

  RCLCPP_INFO(get_logger(), "starting...");
}

ProxyBase::~ProxyBase() {}

bool ProxyBase::read_interface() { return true; }
bool ProxyBase::write_interface() { return true; }
bool ProxyBase::configure_interface() { return true; }
bool ProxyBase::activate_interface() { return true; }
bool ProxyBase::deactivate_interface() { return true; }
bool ProxyBase::cleanup_interface() { return true; }
bool ProxyBase::shutdown_interface() { return true; }

CallbackReturn ProxyBase::on_configure(const State&) {
  RCLCPP_INFO(get_logger(), "configuring...");
  is_activated_ = false;
  bool success = configure_interface();
  if (success) {
    target_vel_watcher_ = std::make_unique<VariableWatcher<Twist>>(
      Duration::from_seconds(get_parameter("target_vel_expire_duration").as_double()), get_clock());
    
    camera_angle_watcher_ = std::make_unique<VariableWatcher<Vector3>>(
      Duration::from_seconds(100000000), get_clock());
    
    ammo_watcher_ = std::make_unique<VariableWatcher<Int64>>(
      Duration::from_seconds(100000000), get_clock());


    target_vel_sub_ = create_subscription<Twist>("target_vel", SystemDefaultsQoS(), [this](const Twist& target_vel){
      target_vel_watcher_->feed(target_vel);
    });

    camera_angle_sub_ = create_subscription<Vector3>("camera_angle", SystemDefaultsQoS(), [this](const Vector3& camera_angle){
      camera_angle_watcher_->feed(camera_angle);
    });

    // current_vel_pub_ = create_publisher<Twist>("current_vel", SensorDataQoS());
    ammo_pub_ = create_publisher<Int64>("ammo", SystemDefaultsQoS().reliable().transient_local());

    auto read_period = microseconds(static_cast<int64_t>(1e6 / get_parameter("read_rate").as_double()));
    read_timer_ = create_wall_timer(read_period, bind(&ProxyBase::read_callback, this));

    auto write_period = microseconds(static_cast<int64_t>(1e6 / get_parameter("write_rate").as_double()));
    write_timer_ =create_wall_timer(write_period, bind(&ProxyBase::write_callback, this));

    RCLCPP_INFO(get_logger(), "configuration success");
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(get_logger(), "configuration failed");
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn ProxyBase::on_activate(const State&) {
  bool success = activate_interface();
  if (success) {
    is_activated_ = true;
    RCLCPP_INFO(get_logger(), "activation success");
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(get_logger(), "activation failed");
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn ProxyBase::on_deactivate(const State&) {
  is_activated_ = false;
  bool success = deactivate_interface();
  if (success) {
    RCLCPP_INFO(get_logger(), "deactivation success");
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(get_logger(), "deactivation failed");
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn ProxyBase::on_cleanup(const State&) {
  is_activated_ = false;
  bool success = cleanup_interface();
  
  target_vel_sub_ = nullptr;
  camera_angle_sub_ = nullptr;
  // current_vel_pub_ = nullptr;
  ammo_pub_ = nullptr;
  read_timer_ = nullptr;
  write_timer_ = nullptr;
  
  if (success) {
    RCLCPP_INFO(get_logger(), "cleanup success");
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(get_logger(), "cleanup failed");
    return CallbackReturn::FAILURE;
  }
}

CallbackReturn ProxyBase::on_shutdown(const State&) {
  is_activated_ = false;
  bool success = shutdown_interface();
  if (success) {
    RCLCPP_INFO(get_logger(), "shutdown success");
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(get_logger(), "shutdown failed");
    return CallbackReturn::FAILURE;
  }
}


void ProxyBase::read_callback() {
  if (!is_activated_) return;

  bool success = read_interface();

  if (!success) {
    RCLCPP_ERROR(get_logger(), "deactivating because of interface read error");
    deactivate();
    return;
  }

  // current_vel_pub_->publish(current_vel);
  if (ammo_watcher_->has_changed()) {
    ammo_pub_->publish(ammo_watcher_->get_value());
  }
}


void ProxyBase::write_callback() {
  if (!is_activated_) return;

  auto target_vel_expire_duration = Duration::from_seconds(get_parameter("target_vel_expire_duration").as_double());
  if (target_vel_watcher_->is_timeout()) {
    RCLCPP_WARN(get_logger(), "target_vel topic have not been received for %.1lf seconds", target_vel_expire_duration.seconds());
    target_vel_watcher_->raw_value = Twist();
  }

  bool success = write_interface();

  if (!success) {
    RCLCPP_ERROR(get_logger(), "deactivating because of interface write error");
    deactivate();
    return;
  }
}


}