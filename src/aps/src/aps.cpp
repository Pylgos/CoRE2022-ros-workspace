#include <chrono>
#include <limits>
#include <memory>
#include <ratio>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/executors.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <librealsense2/rs.hpp>

#include "servo_driver.hpp"

using namespace std;
using std_srvs::srv::SetBool;
using namespace std::chrono_literals;

struct AxisAlignedBBox {
  AxisAlignedBBox(rs2::vertex c1, rs2::vertex c2) : c1{c1}, c2{c2} {}

  bool contains(const rs2::vertex &v) {
    return c1.x <= v.x && v.x <= c2.x && c1.y <= v.y && v.y <= c2.y && c1.z <= v.z && v.z <= c2.z;
  }

  int num_points_inside(const rs2::vertex *vertices, const size_t size) {
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

AxisAlignedBBox bbox{rs2::vertex{-0.3, -0.3, 0.1}, rs2::vertex{0.3, 0.1, 1.0}};
const int threshold = 100;
const int motor_power = INT16_MAX * 0.3;
chrono::nanoseconds trigger_duration = 500ms;

int main(const int argc, const char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("aps_node");
  bool enabled = false;
  auto service = node->create_service<SetBool>(
      "enable_aps", [&enabled](const SetBool::Request::ConstSharedPtr req,
                               const SetBool::Response::SharedPtr resp) {
        enabled = req->data;
        resp->success = true;
      });

  ServoDriver sv("can0", 25);

  rs2::pointcloud pc;
  rs2::points points;
  rs2::pipeline pipe;
  pipe.start();

  auto last_triggered_time = chrono::high_resolution_clock::time_point();

  while (true) {
    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    points = pc.calculate(depth);
    auto vertices = points.get_vertices();
    auto num_points_inside_bbox =
        bbox.num_points_inside(vertices, points.size());
    
    auto now = chrono::high_resolution_clock().now();

    if (num_points_inside_bbox >= threshold) {
      last_triggered_time = now;
    }

    if (now - last_triggered_time > trigger_duration) {
      if (enabled) {
        sv.write(30, motor_power);
      } else {
        sv.write(30, 0);
      }
    } else {
      cout << "triggered: " << num_points_inside_bbox << endl;
      if (enabled) {
        sv.write(0, motor_power);
      }
    } 

    if (rclcpp::ok())
      rclcpp::spin_some(node);
    else
      break;
  }
}