#include <chrono>
#include <functional>
#include <iomanip>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class TfMatrixConsumer : public rclcpp::Node {
 public:
  TfMatrixConsumer()
      : Node("tf_matrix_consumer"), buffer_(get_clock()), listener_(buffer_) {
    target_frame_ = declare_parameter<std::string>("target_frame", "camera_link");
    source_frame_ = declare_parameter<std::string>("source_frame", "base_link");
    timer_ = create_wall_timer(1s, std::bind(&TfMatrixConsumer::print_matrix, this));
  }

 private:
  void print_matrix() {
    try {
      const auto transform =
          buffer_.lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
      const auto& t = transform.transform.translation;
      const auto& q = transform.transform.rotation;
      const double xx = q.x * q.x;
      const double yy = q.y * q.y;
      const double zz = q.z * q.z;
      const double xy = q.x * q.y;
      const double xz = q.x * q.z;
      const double yz = q.y * q.z;
      const double wx = q.w * q.x;
      const double wy = q.w * q.y;
      const double wz = q.w * q.z;

      RCLCPP_INFO(
        get_logger(),
        "T_%s_%s = [[%.6f, %.6f, %.6f, %.6f], [%.6f, %.6f, %.6f, %.6f], "
        "[%.6f, %.6f, %.6f, %.6f], [0, 0, 0, 1]]",
          target_frame_.c_str(), source_frame_.c_str(), 1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),
          2.0 * (xz + wy), t.x, 2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx),
          t.y, 2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), t.z);
    } catch (const tf2::TransformException& exception) {
      RCLCPP_DEBUG(get_logger(), "Waiting for transform: %s", exception.what());
    }
  }

  std::string target_frame_;
  std::string source_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfMatrixConsumer>());
  rclcpp::shutdown();
  return 0;
}
