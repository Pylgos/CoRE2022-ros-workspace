#include "robot_interface_proxy/proxy_base.hpp"
#include <asm-generic/errno-base.h>
#include <cerrno>
#include <cstring>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/detail/int64__struct.hpp>
#include <sys/socket.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <unistd.h>
#include <optional>


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

    // to read
    declare_parameter("launcher_info_can_id", 22);
  }

  bool configure_interface() override {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (sock_ == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to create socket: %s", strerror(errno));
      return false;
    }

    string can_name = get_parameter("can_interface").as_string();

    if (can_name.size() > 15) {
      RCLCPP_ERROR(get_logger(), "CAN interface name '%s' is too long", can_name.c_str());
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
      RCLCPP_ERROR(get_logger(), "CAN interface '%s' not found", can_name.c_str());
      return false;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int res = bind(sock_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (res != 0) {
      RCLCPP_ERROR(get_logger(), "bind error: %s", strerror(errno));
      return false;
    }

    int val = 1;
    ioctl(sock_, FIONBIO, &val);

    return true;
  }

  void process_launcher_info_can_frame(const struct can_frame& frame) {
    canid_t id = frame.can_id & CAN_ERR_MASK;
    if (id != get_parameter("launcher_info_can_id").as_int()) return;
    if (frame.len != sizeof(LauncherInfoMsg)) {
      RCLCPP_WARN(get_logger(), "Invalid length of launcher info can frame. id=%u len=%u expected len=%zu", id, frame.len, sizeof(LauncherInfoMsg));
      return;
    }

    LauncherInfoMsg msg;
    memcpy(&msg, frame.data, sizeof(msg));

    Int64 m;
    m.data = msg.ammo;
    ammo_watcher_->feed(m);
  }

  void process_can_frame(const struct can_frame& frame) {
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

  bool write_interface() override {
    if (target_vel_watcher_->has_changed_or_timeout()) {
      target_vel_watcher_->reset_timeout();
      target_vel_watcher_->reset_changed_flag();
      struct can_frame frame;
      frame.can_id = get_parameter("target_vel_can_id").as_int();
      frame.len = 6;

      const Twist twist = target_vel_watcher_->get_value();
      TargetVelocityMsg msg;
      msg.vx = twist.linear.x * 1000;
      msg.vy = twist.linear.y * 1000;
      msg.ang_vel = twist.angular.z * 1000;
      memcpy(frame.data, &msg, sizeof(msg));

      int ret = write(sock_, &frame, sizeof(frame));
      if (ret == -1) {
        if (errno != EAGAIN) {
          RCLCPP_ERROR(get_logger(), "write error: %s", strerror(errno));
          return false;
        }
      }
    }

    if (camera_angle_watcher_->has_changed_or_timeout()) {
      camera_angle_watcher_->reset_timeout();
      camera_angle_watcher_->reset_changed_flag();

      struct can_frame frame;
      frame.can_id = get_parameter("camera_angle_can_id").as_int();
      frame.len = 4;

      const Vector3 angle = camera_angle_watcher_->get_value();
      CameraAngleMsg msg;
      msg.pitch = angle.y * 1000;
      msg.yaw = angle.z * 1000;
      memcpy(frame.data, &msg, sizeof(msg));

      int ret = write(sock_, &frame, sizeof(frame));
      if (ret == -1) {
        if (errno != EAGAIN) {
          RCLCPP_ERROR(get_logger(), "write error: %s", strerror(errno));
          return false;
        }
      }
    }

    if (fire_command_watcher_->has_changed_or_timeout()) {
      fire_command_watcher_->reset_timeout();
      fire_command_watcher_->reset_changed_flag();

      struct can_frame frame;
      frame.can_id = get_parameter("fire_command_can_id").as_int();
      frame.len = sizeof(FireCommandMsg);
      
      FireCommandMsg msg;
      msg.enable = fire_command_watcher_->get_value();
      memcpy(frame.data, &msg, sizeof(msg));

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
    int16_t vx; // 前後方向の速度[m/s] * 1000 前が+　後ろが-
    int16_t vy; //　左右方向の速度[m/s] * 1000 左が+ 右が-
    int16_t ang_vel; // 回転速度[rad/s] * 1000 左旋回が+ 右旋回が-
  } __attribute__((packed));

  struct CameraAngleMsg {
    int16_t pitch; // 上下方向の角度[rad] * 1000 上が-　下が+
    int16_t yaw; // 左右方向の角度[rad] * 1000 左が+ 右が-
  } __attribute__((packed));

  struct LauncherInfoMsg {
    uint8_t ammo; // 残弾数
  } __attribute__((packed));

  struct FireCommandMsg {
    bool enable; // 発射を行うかどうか trueなら発射する
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
