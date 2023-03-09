#include "robot_interface_proxy/proxy_base.hpp"
#include <asm-generic/errno-base.h>
#include <cerrno>
#include <cstring>
#include <limits>
#include <linux/can.h>
#include <memory>
#include <net/if.h>
#include <optional>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std;
using namespace std::chrono;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::Int64;

namespace robot_interface_proxy {

class CANProxy : public ProxyBase {
public:
  CANProxy(std::string node_name) : ProxyBase{node_name} {
    declare_parameter("can_interface", "can0");

    // to write
    declare_parameter("target_vel_can_id", 20);
    declare_parameter("camera_angle_can_id", 21);
    declare_parameter("fire_command_can_id", 23);
    declare_parameter("expand_camera_can_id", 24);
    declare_parameter("arm_control_can_id", 25);

    // to read
    declare_parameter("launcher_info_can_id", 22);
  }

  bool configure_interface() override {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (sock_ == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to create socket: %s",
                   strerror(errno));
      return false;
    }

    string can_name = get_parameter("can_interface").as_string();

    if (can_name.size() > 15) {
      RCLCPP_ERROR(get_logger(), "CAN interface name '%s' is too long",
                   can_name.c_str());
      return false;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;

    // shut up valgrind
    memset(&addr, 0, sizeof(addr));

    strncpy(ifr.ifr_name, can_name.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
    if (ifr.ifr_ifindex == 0) {
      RCLCPP_ERROR(get_logger(), "CAN interface '%s' not found",
                   can_name.c_str());
      return false;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int res =
        bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    if (res != 0) {
      RCLCPP_ERROR(get_logger(), "bind error: %s", strerror(errno));
      return false;
    }

    int val = 1;
    ioctl(sock_, FIONBIO, &val);

    return true;
  }

  void process_launcher_info_can_frame(const struct can_frame &frame) {
    canid_t id = frame.can_id & CAN_ERR_MASK;
    if (id != get_parameter("launcher_info_can_id").as_int())
      return;
    if (frame.len != sizeof(LauncherInfoMsg)) {
      RCLCPP_WARN(get_logger(),
                  "Invalid length of launcher info can frame. id=%u len=%u "
                  "expected len=%zu",
                  id, frame.len, sizeof(LauncherInfoMsg));
      return;
    }

    LauncherInfoMsg msg;
    memcpy(&msg, frame.data, sizeof(msg));

    Int64 m;
    m.data = msg.ammo;
    ammo_watcher_->feed(m);
  }

  void process_can_frame(const struct can_frame &frame) {
    process_launcher_info_can_frame(frame);
  }

  bool read_interface() override {
    while (true) {
      struct can_frame frame;
      ssize_t len = read(sock_, &frame, sizeof(frame));
      if (len == -1) {
        if (errno == EAGAIN) {
          break;
        } else {
          RCLCPP_ERROR(get_logger(), "read error: %s", strerror(errno));
          return false;
        }
      } else {
        process_can_frame(frame);
      }
    }
    return true;
  };

  template <class T> bool write_struct(const T &data, canid_t id) {
    static_assert(sizeof(data) <= 8);

    struct can_frame frame;
    frame.can_id = id;
    frame.len = sizeof(T);
    memcpy(frame.data, &data, sizeof(data));
    int ret = write(sock_, &frame, sizeof(frame));
    if (ret == -1) {
      if (errno != EAGAIN) {
        RCLCPP_ERROR(get_logger(), "write error: %s", strerror(errno));
        return false;
      }
    }
    return true;
  }

  template <class T>
  bool write_struct(const T &data, string can_id_param_name) {
    return write_struct(data, get_parameter(can_id_param_name).as_int());
  }

  bool write_interface() override {
    if (target_vel_watcher_->has_changed_or_timeout()) {
      target_vel_watcher_->reset_timeout();
      target_vel_watcher_->reset_changed_flag();

      const Twist twist = target_vel_watcher_->get_value();
      TargetVelocityMsg msg;
      msg.vx = twist.linear.x * 1000;
      msg.vy = twist.linear.y * 1000;
      msg.ang_vel = twist.angular.z * 1000;

      if (!write_struct(msg, "target_vel_can_id"))
        return false;
    }

    if (camera_angle_watcher_->has_changed_or_timeout()) {
      camera_angle_watcher_->reset_timeout();
      camera_angle_watcher_->reset_changed_flag();

      const Vector3 angle = camera_angle_watcher_->get_value();
      CameraAngleMsg msg;
      msg.pitch = angle.y * 1000;
      msg.yaw = angle.z * 1000;

      if (!write_struct(msg, "camera_angle_can_id"))
        return false;
    }

    if (fire_command_watcher_->has_changed_or_timeout()) {
      fire_command_watcher_->reset_timeout();
      fire_command_watcher_->reset_changed_flag();

      FireCommandMsg msg;
      msg.enable = fire_command_watcher_->get_value();
      if (!write_struct(msg, "fire_command_can_id"))
        return false;
    }

    if (arm_grabber_command_watcher_->has_changed_or_timeout() ||
        arm_lift_command_watcher_->has_changed_or_timeout()) {
      arm_grabber_command_watcher_->reset_timeout();
      arm_grabber_command_watcher_->reset_changed_flag();
      arm_lift_command_watcher_->reset_timeout();
      arm_lift_command_watcher_->reset_changed_flag();

      ArmControlMsg msg;
      msg.grabber_command = arm_grabber_command_watcher_->get_value() * 1000;
      msg.lift_command = arm_lift_command_watcher_->get_value() * 1000;

      if (!write_struct(msg, "arm_control_can_id"))
        return false;
    }

    if (expand_camera_has_triggered_) {
      expand_camera_has_triggered_ = false;
      struct can_frame frame;
      frame.can_id = get_parameter("expand_camera_can_id").as_int();
      frame.len = 0;
      int ret = write(sock_, &frame, sizeof(frame));
      if (ret == -1) {
        if (errno != EAGAIN) {
          RCLCPP_ERROR(get_logger(), "write error: %s", strerror(errno));
          return false;
        }
      }
    }

    return true;
  }

  bool cleanup_interface() override {
    close(sock_);
    return true;
  }

private:
  int sock_;

  struct TargetVelocityMsg {
    int16_t vx;      // 前後方向の速度[m/s] * 1000 前が+　後ろが-
    int16_t vy;      // 　左右方向の速度[m/s] * 1000 左が+ 右が-
    int16_t ang_vel; // 回転速度[rad/s] * 1000 左旋回が+ 右旋回が-
  } __attribute__((packed));

  struct CameraAngleMsg {
    int16_t pitch; // 上下方向の角度[rad] * 1000 上が-　下が+
    int16_t yaw;   // 左右方向の角度[rad] * 1000 左が+ 右が-
  } __attribute__((packed));

  struct LauncherInfoMsg {
    uint8_t ammo; // 残弾数
  } __attribute__((packed));

  struct FireCommandMsg {
    bool enable; // 発射を行うかどうか trueなら発射する
  } __attribute__((packed));

  struct ArmControlMsg {
    int16_t lift_command;
    int16_t grabber_command;
  } __attribute__((packed));
};

} // namespace robot_interface_proxy

int main(int argc, char *argv[]) {
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto node = std::make_shared<robot_interface_proxy::CANProxy>("can_proxy");

  exe.add_node(node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
