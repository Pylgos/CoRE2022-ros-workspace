#include <cerrno>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <limits>
#include <linux/can.h>
#include <memory>
#include <net/if.h>
#include <optional>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <sensor_msgs/msg/joy.hpp>


using namespace std;
using namespace std::chrono;
using sensor_msgs::msg::Joy;


namespace joy2can {

class Joy2Can : public rclcpp::Node {
public:
  Joy2Can(std::string node_name) : rclcpp::Node{node_name}, sock_{-1} {
    declare_parameter("can_interface", "can0");
    declare_parameter("can_id", 20);
    double write_frequency = declare_parameter("write_frequency", 60.0);

    timer_ = create_wall_timer(nanoseconds(static_cast<uint64_t>(1e9 / write_frequency)), [this]{
      if (sock_ == -1 || !write_interface()) {
        cleanup_interface();
        configure_interface();
        this_thread::sleep_for(1s);
        timer_->reset();
      }
    });

    joy_sub_ = create_subscription<Joy>("joy", rclcpp::SensorDataQoS(), [this](const Joy::ConstSharedPtr msg){
      joy_msg_ = msg;
    });
  }

  bool configure_interface() {
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
      cleanup_interface();
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
      cleanup_interface();
      return false;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    int res =
        bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    if (res != 0) {
      RCLCPP_ERROR(get_logger(), "bind error: %s", strerror(errno));
      cleanup_interface();
      return false;
    }

    int val = 1;
    ioctl(sock_, FIONBIO, &val);

    return true;
  }

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

  bool write_interface() {
    if (joy_msg_ == nullptr) return true;
    
    JoyMsg joy;
    memset(&joy, 0, sizeof(joy));
    if (joy_msg_->axes.size() >= 6) {
      joy.left_stick_x = joy_msg_->axes.at(0) * 127;
      joy.left_stick_y = joy_msg_->axes.at(1) * 127;
      joy.right_stick_x = joy_msg_->axes.at(2) * 127;
      joy.right_stick_y = joy_msg_->axes.at(3) * 127;
      joy.l2 = (0.5 - joy_msg_->axes.at(4) / 2) * 255;
      joy.r2 = (0.5 - joy_msg_->axes.at(5) / 2) * 255;
    }

    for (size_t i = 0; i < min(8UL, joy_msg_->buttons.size()); i++) {
      joy.buttons_1 |= joy_msg_->buttons.at(i) << i;
    }

    for (int i = 0; i < min(8, (int)joy_msg_->buttons.size() - 8); i++) {
      joy.buttons_2 |= joy_msg_->buttons.at(i+8) << i;
    }
    
    return write_struct(joy, "can_id");
  }

  void cleanup_interface() {
    if (sock_ != -1) {
      close(sock_);
      sock_ = -1;
    }
  }

  struct JoyMsg {
    int8_t left_stick_x;
    int8_t left_stick_y;
    int8_t right_stick_x;
    int8_t right_stick_y;
    uint8_t l2;
    uint8_t r2;
    uint8_t buttons_1;
    uint8_t buttons_2;
  } __attribute__((packed));

private:
  int sock_;
  Joy::ConstSharedPtr joy_msg_;
  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char *argv[]) {
  // force flush of the stdout buffer.
  // this ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto node = std::make_shared<joy2can::Joy2Can>("joy2can");

  exe.add_node(node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
