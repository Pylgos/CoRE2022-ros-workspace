#include <chrono>
#include <memory>
#include <librealsense2/rs.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "aps_driver.hpp"

using namespace std;
using std_srvs::srv::SetBool;
using namespace std::chrono_literals;

struct AxisAlignedBBox {
  AxisAlignedBBox(rs2::vertex c1, rs2::vertex c2) : c1{c1}, c2{c2} {}

  bool contains(const rs2::vertex &v) const {
    return c1.x <= v.x && v.x <= c2.x && c1.y <= v.y && v.y <= c2.y &&
           c1.z <= v.z && v.z <= c2.z;
  }

  int num_points_inside(const rs2::vertex *vertices, const size_t size) const {
    int result = 0;
    for (size_t i = 0; i < size; i++) {
      if (contains(vertices[i])) {
        result++;
      }
    }
    return result;
  }

  rs2::vertex c1, c2;
};

const AxisAlignedBBox bbox{rs2::vertex{-0.3, -0.3, 0.1},
                           rs2::vertex{0.3, 0.1, 1.0}};
const int threshold = 100;
const int launch_power = INT16_MAX * 0.3;
const chrono::nanoseconds fire_duration = 500ms;
const chrono::nanoseconds reload_duration = 1s;
const double angle_neutral = 30;
const double angle_fire = 0;
const double angle_reload = 50;

int main(const int argc, const char **argv) {
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("aps_node");
  bool enabled = false;
  const auto service = node->create_service<SetBool>(
      "enable_aps", [&](const SetBool::Request::ConstSharedPtr req,
                        const SetBool::Response::SharedPtr resp) {
        enabled = req->data;
        resp->success = true;
        RCLCPP_INFO(node->get_logger(), "APS ENABLED: %u", enabled);
      });

  ApsDriver aps("can0", 26);
  bool running = true;

  while (running) {
    try {
      rs2::pointcloud pc;
      rs2::points points;
      rs2::pipeline pipe;
      RCLCPP_INFO(node->get_logger(), "Starting pipeline...");
      pipe.start();
      RCLCPP_INFO(node->get_logger(), "Pipeline started!");

      double angle = angle_neutral;
      int power = 0;
      enum class State {
        READY,
        FIRE,
        RELOAD,
      } state = State::READY;
      chrono::high_resolution_clock::time_point state_expire_time =
          chrono::high_resolution_clock().now();

      while (running) {
        auto frames = pipe.wait_for_frames();

        if (enabled) {
          auto depth = frames.get_depth_frame();
          points = pc.calculate(depth);
          auto vertices = points.get_vertices();
          auto num_points_inside_bbox =
              bbox.num_points_inside(vertices, points.size());

          auto now = chrono::high_resolution_clock().now();
          bool state_expired = state_expire_time >= now;

          power = launch_power;

          switch (state) {
          case State::READY: {
            if (num_points_inside_bbox >= threshold) {
              state_expire_time = now + fire_duration;
              state = State::FIRE;
              angle = angle_fire;
            }
          } break;

          case State::FIRE: {
            if (state_expired) {
              state_expire_time = now + reload_duration;
              state = State::RELOAD;
              angle = angle_reload;
            }
          } break;

          case State::RELOAD: {
            if (state_expired) {
              state = State::READY;
              angle = angle_neutral;
            }
          } break;
          }
        } else {
          angle = angle_neutral;
          power = 0;
        }

        aps.write(angle, power);

        if (rclcpp::ok()) {
          rclcpp::spin_some(node);
        } else {
          running = false;
        }
      }
    } catch (const rs2::error &e) {
      RCLCPP_ERROR(node->get_logger(), "realsense error: what: %s", e.what());
      if (!rclcpp::ok())
        break;
    }
  }
}