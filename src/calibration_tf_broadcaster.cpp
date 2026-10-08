#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "yaml-cpp/yaml.h"

class CalibrationTfBroadcaster : public rclcpp::Node {
 public:
  struct DynamicTransformConfig {
    std::string topic;
    std::string parent_frame;
    std::string child_frame;
  };

  CalibrationTfBroadcaster()
      : Node("calibration_tf_broadcaster") {
    const auto yaml_path = declare_parameter<std::string>("calibration_yaml", "");

    if (yaml_path.empty()) {
      throw std::runtime_error("Parameter 'calibration_yaml' must point to a YAML file");
    }

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    load_dynamic_transform_configurations(yaml_path);
    publish_static_transforms(yaml_path);

    for (const auto & config : dynamic_transforms_) {
      dynamic_subscriptions_.push_back(
        create_subscription<geometry_msgs::msg::PoseStamped>(
          config.topic, rclcpp::SensorDataQoS(),
          [this, config](geometry_msgs::msg::PoseStamped::SharedPtr pose) {
            dynamic_transform_callback(pose, config);
          }));
      RCLCPP_INFO(
        get_logger(), "Listening to %s and broadcasting %s -> %s",
        config.topic.c_str(), config.parent_frame.c_str(), config.child_frame.c_str());
    }
  }

 private:
  void load_dynamic_transform_configurations(const std::string& yaml_path) {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const YAML::Node transforms = root["dynamic_transforms"];
    if (!transforms || !transforms.IsSequence() || transforms.size() == 0U) {
      throw std::runtime_error("YAML must contain a non-empty 'dynamic_transforms' sequence");
    }

    std::unordered_set<std::string> child_frames;
    dynamic_transforms_.reserve(transforms.size());
    for (const auto & item : transforms) {
      if (!item.IsMap()) {
        throw std::runtime_error("Each dynamic_transforms item must be a map");
      }
      DynamicTransformConfig config{item["topic"].as<std::string>(),
                                    item["parent_frame"].as<std::string>(),
                                    item["child_frame"].as<std::string>()};
      if (config.topic.empty() || config.parent_frame.empty() || config.child_frame.empty()) {
        throw std::runtime_error("Dynamic transform topics and frame names cannot be empty");
      }
      if (!child_frames.insert(config.child_frame).second) {
        throw std::runtime_error("Each dynamic transform must have a unique child_frame: " +
                                 config.child_frame);
      }
      dynamic_transforms_.push_back(std::move(config));
    }
  }

  void publish_static_transforms(const std::string& yaml_path) {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const YAML::Node transforms = root["static_transforms"];
    if (!transforms || !transforms.IsSequence()) {
      throw std::runtime_error("YAML must contain a 'static_transforms' sequence");
    }

    std::vector<geometry_msgs::msg::TransformStamped> messages;
    messages.reserve(transforms.size());
    for (const auto & item : transforms) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = now();
      transform.header.frame_id = item["parent_frame"].as<std::string>();
      transform.child_frame_id = item["child_frame"].as<std::string>();

      const auto matrix = item["transform_matrix"];
      if (!matrix || !matrix.IsSequence() || matrix.size() != 4U) {
        throw std::runtime_error("Each transform requires a 4x4 'transform_matrix'");
      }

      double values[4][4];
      for (std::size_t row = 0; row < 4U; ++row) {
        if (!matrix[row].IsSequence() || matrix[row].size() != 4U) {
          throw std::runtime_error("'transform_matrix' must contain four rows of four values");
        }
        for (std::size_t column = 0; column < 4U; ++column) {
          values[row][column] = matrix[row][column].as<double>();
        }
      }
      constexpr double kTolerance = 1e-6;
      if (
        std::abs(values[3][0]) > kTolerance || std::abs(values[3][1]) > kTolerance ||
        std::abs(values[3][2]) > kTolerance || std::abs(values[3][3] - 1.0) > kTolerance)
      {
        throw std::runtime_error("The final transform_matrix row must be [0, 0, 0, 1]");
      }

      const tf2::Matrix3x3 rotation_matrix(values[0][0], values[0][1], values[0][2],
                                           values[1][0], values[1][1], values[1][2],
                                           values[2][0], values[2][1], values[2][2]);
      tf2::Quaternion rotation;
      rotation_matrix.getRotation(rotation);
      rotation.normalize();

      transform.transform.translation.x = values[0][3];
      transform.transform.translation.y = values[1][3];
      transform.transform.translation.z = values[2][3];
      transform.transform.rotation.x = rotation.x();
      transform.transform.rotation.y = rotation.y();
      transform.transform.rotation.z = rotation.z();
      transform.transform.rotation.w = rotation.w();
      messages.push_back(transform);
    }
    static_broadcaster_->sendTransform(messages);
    RCLCPP_INFO(get_logger(), "Published %zu static calibration transforms", messages.size());
  }

  void dynamic_transform_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose,
      const DynamicTransformConfig& config) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = pose->header.stamp;
    if (transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0) {
      transform.header.stamp = now();
    }
    transform.header.frame_id =
      pose->header.frame_id.empty() ? config.parent_frame : pose->header.frame_id;
    transform.child_frame_id = config.child_frame;
    transform.transform.translation.x = pose->pose.position.x;
    transform.transform.translation.y = pose->pose.position.y;
    transform.transform.translation.z = pose->pose.position.z;
    transform.transform.rotation = pose->pose.orientation;
    dynamic_broadcaster_->sendTransform(transform);
  }

  std::vector<DynamicTransformConfig> dynamic_transforms_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
    dynamic_subscriptions_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CalibrationTfBroadcaster>());
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("calibration_tf_broadcaster"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
