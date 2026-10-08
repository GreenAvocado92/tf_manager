#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

class PointCloudTfTransformer : public rclcpp::Node {
 public:
  PointCloudTfTransformer()
      : Node("pointcloud_tf_transformer"), buffer_(get_clock()), listener_(buffer_) {
    const auto config_path = declare_parameter<std::string>("pointcloud_config", "");
    if (config_path.empty()) {
      throw std::runtime_error("Parameter 'pointcloud_config' must point to a YAML file");
    }
    load_configuration(config_path);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudTfTransformer::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(), "Transforming %s (%s) from %s to %s and publishing %s", cloud_topic_.c_str(),
        input_xyz_unit_.c_str(), source_frame_id_.c_str(), target_frame_id_.c_str(),
        output_topic_.c_str());
  }

 private:
  void load_configuration(const std::string& config_path) {
    const YAML::Node config = YAML::LoadFile(config_path);
    input_xyz_unit_ = config["input_xyz_unit"].as<std::string>();
    cloud_topic_ = config["cloud_topic"].as<std::string>();
    output_topic_ = config["output_topic"].as<std::string>();
    source_frame_id_ = config["source_frame_id"].as<std::string>();
    target_frame_id_ = config["target_frame_id"].as<std::string>();

    if (input_xyz_unit_ != "m" && input_xyz_unit_ != "mm") {
      throw std::runtime_error("'input_xyz_unit' must be either 'm' or 'mm'");
    }
    if (cloud_topic_.empty() || output_topic_.empty() || source_frame_id_.empty() ||
        target_frame_id_.empty()) {
      throw std::runtime_error("Point cloud topics and frame IDs in YAML cannot be empty");
    }
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
    // if (!input->header.frame_id.empty() && input->header.frame_id != source_frame_id_) {
    //   RCLCPP_WARN_THROTTLE(
    //     get_logger(), *get_clock(), 5000,
    //     "Ignoring cloud with frame_id '%s'; expected '%s'", input->header.frame_id.c_str(),
    //     source_frame_id_.c_str());
    //   return;
    // }

    geometry_msgs::msg::TransformStamped transform_message;
    try {
      transform_message =
          buffer_.lookupTransform(target_frame_id_, source_frame_id_, input->header.stamp, 100ms);
    } catch (const tf2::TransformException& exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "TF unavailable; publishing the input cloud unchanged: %s", exception.what());
      publisher_->publish(*input);
      return;
    }

    sensor_msgs::msg::PointCloud2 output = *input;
    output.header.frame_id = target_frame_id_;
    tf2::Transform transform;
    tf2::fromMsg(transform_message.transform, transform);
    const auto & translation = transform.getOrigin();
    const auto rotation = transform.getRotation();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "T_%s_%s: translation=(%.6f, %.6f, %.6f), quaternion=(%.6f, %.6f, %.6f, %.6f)",
        target_frame_id_.c_str(), source_frame_id_.c_str(), translation.x(), translation.y(),
        translation.z(), rotation.w(), rotation.x(), rotation.y(), rotation.z());
    // TF translations follow ROS convention and are expressed in metres.  Keep
    // all point-cloud coordinates and the transformed output in millimetres.
    const double input_scale_to_mm = input_xyz_unit_ == "m" ? 1000. : 1.0;
    const tf2::Vector3 translation_in_mm(translation.x() * 1000.0, translation.y() * 1000.0,
                                         translation.z() * 1000.0);

    try {
      sensor_msgs::PointCloud2Iterator<float> x(output, "x");
      sensor_msgs::PointCloud2Iterator<float> y(output, "y");
      sensor_msgs::PointCloud2Iterator<float> z(output, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        const tf2::Vector3 point(*x * input_scale_to_mm, *y * input_scale_to_mm,
                                 *z * input_scale_to_mm);
        const tf2::Vector3 transformed_point =
            transform.getBasis() * point + translation_in_mm;
        *x = static_cast<float>(transformed_point.x());
        *y = static_cast<float>(transformed_point.y());
        *z = static_cast<float>(transformed_point.z());
      }
    } catch (const std::runtime_error& exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 must contain float32 x, y and z fields: %s", exception.what());
      return;
    }
    publisher_->publish(output);
  }

  std::string input_xyz_unit_;
  std::string cloud_topic_;
  std::string output_topic_;
  std::string source_frame_id_;
  std::string target_frame_id_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PointCloudTfTransformer>());
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("pointcloud_tf_transformer"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
