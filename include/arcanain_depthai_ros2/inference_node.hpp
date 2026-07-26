// Copyright 2026 shiryu nakano
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ARCANAIN_DEPTHAI_ROS2__INFERENCE_NODE_HPP_
#define ARCANAIN_DEPTHAI_ROS2__INFERENCE_NODE_HPP_

#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <depthai/depthai.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace arcanain_depthai_ros2
{

struct YoloConfig
{
  int width;
  int height;
  int classes;
  int coordinates;
  std::vector<float> anchors;
  std::map<std::string, std::vector<int>> anchor_masks;
  float iou_threshold;
  float confidence_threshold;
  std::vector<std::string> labels;
};

class InferenceNode : public rclcpp::Node
{
public:
  explicit InferenceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~InferenceNode() override;

private:
  static std::string expand_user(const std::string & path);
  static YoloConfig load_yolo_config(const std::string & path);
  std::string label_name(int label_id) const;
  void timer_callback();
  void draw_detection(
    cv::Mat & frame, const dai::ImgDetection & detection,
    const std::string & label) const;

  std::string frame_id_;
  std::vector<std::string> labels_;
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::DataOutputQueue> detection_queue_;
  std::shared_ptr<dai::DataOutputQueue> image_queue_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace arcanain_depthai_ros2

#endif  // ARCANAIN_DEPTHAI_ROS2__INFERENCE_NODE_HPP_
