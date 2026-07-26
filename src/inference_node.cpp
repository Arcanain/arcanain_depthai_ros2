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

#include "arcanain_depthai_ros2/inference_node.hpp"

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace arcanain_depthai_ros2
{

InferenceNode::InferenceNode(const rclcpp::NodeOptions & options)
: Node("yolo_detection_node", options)
{
  const auto model = declare_parameter<std::string>("model", "");
  const auto config_path = declare_parameter<std::string>("config_path", "");
  const auto detections_topic =
    declare_parameter<std::string>("detections_topic", "detections");
  const auto image_topic =
    declare_parameter<std::string>("image_topic", "detections/image");
  frame_id_ =
    declare_parameter<std::string>("frame_id", "camera_rgb_optical_frame");
  const auto camera_fps = declare_parameter<double>("camera_fps", 30.0);
  const auto confidence_override =
    declare_parameter<double>("confidence_threshold", -1.0);

  if (model.empty()) {
    throw std::invalid_argument("The 'model' parameter must contain a .blob path");
  }
  if (config_path.empty()) {
    throw std::invalid_argument("The 'config_path' parameter must be set");
  }
  if (confidence_override > 1.0) {
    throw std::invalid_argument("confidence_threshold must be between 0 and 1");
  }

  const fs::path model_path = fs::absolute(expand_user(model));
  if (!fs::is_regular_file(model_path) || model_path.extension() != ".blob") {
    throw std::runtime_error(
            "Model blob does not exist: " + model_path.string() +
            ". Run the model download command documented in README.md.");
  }

  auto yolo = load_yolo_config(expand_user(config_path));
  if (confidence_override >= 0.0) {
    yolo.confidence_threshold = static_cast<float>(confidence_override);
  }
  labels_ = yolo.labels;

  detection_publisher_ =
    create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic, 10);
  image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic, 10);

  dai::Pipeline pipeline;
  auto camera = pipeline.create<dai::node::ColorCamera>();
  auto network = pipeline.create<dai::node::YoloDetectionNetwork>();
  auto detection_output = pipeline.create<dai::node::XLinkOut>();
  auto image_output = pipeline.create<dai::node::XLinkOut>();

  detection_output->setStreamName("nn");
  image_output->setStreamName("nn_image");

  camera->setBoardSocket(dai::CameraBoardSocket::CAM_A);
  camera->setPreviewSize(yolo.width, yolo.height);
  camera->setResolution(
    dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  camera->setInterleaved(false);
  camera->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
  camera->setFps(static_cast<float>(camera_fps));

  network->setConfidenceThreshold(yolo.confidence_threshold);
  network->setNumClasses(yolo.classes);
  network->setCoordinateSize(yolo.coordinates);
  network->setAnchors(yolo.anchors);
  network->setAnchorMasks(yolo.anchor_masks);
  network->setIouThreshold(yolo.iou_threshold);
  network->setBlobPath(model_path.string());
  network->setNumInferenceThreads(2);
  network->input.setBlocking(false);

  camera->preview.link(network->input);
  network->out.link(detection_output->input);
  network->passthrough.link(image_output->input);

  device_ = std::make_shared<dai::Device>(pipeline);
  detection_queue_ = device_->getOutputQueue("nn", 4, false);
  image_queue_ = device_->getOutputQueue("nn_image", 4, false);
  timer_ = create_wall_timer(30ms, std::bind(&InferenceNode::timer_callback, this));

  RCLCPP_INFO(get_logger(), "Model blob: %s", model_path.c_str());
  RCLCPP_INFO(get_logger(), "On-device inference started");
}

InferenceNode::~InferenceNode()
{
  timer_.reset();
  detection_queue_.reset();
  image_queue_.reset();
  if (device_) {
    device_->close();
    device_.reset();
  }
}

std::string InferenceNode::expand_user(const std::string & path)
{
  if (path == "~" || path.rfind("~/", 0) == 0) {
    const char * home = std::getenv("HOME");
    if (home == nullptr) {
      throw std::runtime_error("HOME is not set; cannot expand model path");
    }
    return std::string(home) + path.substr(1);
  }
  return path;
}

YoloConfig InferenceNode::load_yolo_config(const std::string & path)
{
  if (!fs::is_regular_file(path)) {
    throw std::runtime_error("YOLO config does not exist: " + path);
  }

  const YAML::Node root = YAML::LoadFile(path);
  const auto nn_config = root["nn_config"];
  const auto metadata = nn_config["NN_specific_metadata"];
  const auto input_size = nn_config["input_size"].as<std::string>();
  const auto separator = input_size.find('x');
  if (separator == std::string::npos) {
    throw std::runtime_error("Invalid input_size in YOLO config: " + input_size);
  }

  YoloConfig config;
  config.width = std::stoi(input_size.substr(0, separator));
  config.height = std::stoi(input_size.substr(separator + 1));
  config.classes = metadata["classes"].as<int>();
  config.coordinates = metadata["coordinates"].as<int>();
  config.anchors = metadata["anchors"].as<std::vector<float>>();
  config.iou_threshold = metadata["iou_threshold"].as<float>();
  config.confidence_threshold = metadata["confidence_threshold"].as<float>();
  config.labels = root["mappings"]["labels"].as<std::vector<std::string>>();

  for (const auto & mask : metadata["anchor_masks"]) {
    config.anchor_masks.emplace(
      mask.first.as<std::string>(), mask.second.as<std::vector<int>>());
  }
  return config;
}

std::string InferenceNode::label_name(int label_id) const
{
  if (label_id >= 0 && static_cast<std::size_t>(label_id) < labels_.size()) {
    return labels_[label_id];
  }
  return std::to_string(label_id);
}

void InferenceNode::timer_callback()
{
  const auto detections = detection_queue_->tryGet<dai::ImgDetections>();
  const auto image = image_queue_->tryGet<dai::ImgFrame>();
  if (!detections || !image) {
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = frame_id_;

  vision_msgs::msg::Detection2DArray detection_array;
  detection_array.header = header;
  auto data = image->getData();
  const int width = image->getWidth();
  const int height = image->getHeight();
  const auto plane_size = static_cast<std::size_t>(width * height);
  if (data.size() < plane_size * 3) {
    RCLCPP_ERROR(get_logger(), "Received an incomplete BGR frame");
    return;
  }

  std::vector<cv::Mat> channels{
    cv::Mat(height, width, CV_8UC1, data.data()),
    cv::Mat(
      height, width, CV_8UC1,
      data.data() + plane_size),
    cv::Mat(
      height, width, CV_8UC1,
      data.data() + plane_size * 2)
  };
  cv::Mat frame;
  cv::merge(channels, frame);

  for (const auto & detection : detections->detections) {
    const auto label = label_name(detection.label);
    vision_msgs::msg::Detection2D detection_message;
    detection_message.header = header;

    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = label;
    hypothesis.hypothesis.score = detection.confidence;
    detection_message.results.push_back(hypothesis);

    detection_message.bbox.center.position.x =
      (detection.xmin + detection.xmax) / 2.0;
    detection_message.bbox.center.position.y =
      (detection.ymin + detection.ymax) / 2.0;
    detection_message.bbox.size_x = detection.xmax - detection.xmin;
    detection_message.bbox.size_y = detection.ymax - detection.ymin;
    detection_array.detections.push_back(detection_message);

    draw_detection(frame, detection, label);
  }

  detection_publisher_->publish(detection_array);
  image_publisher_->publish(
    *cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg());
}

void InferenceNode::draw_detection(
  cv::Mat & frame, const dai::ImgDetection & detection,
  const std::string & label) const
{
  const int xmin = std::clamp(
    static_cast<int>(detection.xmin * frame.cols), 0, frame.cols - 1);
  const int ymin = std::clamp(
    static_cast<int>(detection.ymin * frame.rows), 0, frame.rows - 1);
  const int xmax = std::clamp(
    static_cast<int>(detection.xmax * frame.cols), 0, frame.cols - 1);
  const int ymax = std::clamp(
    static_cast<int>(detection.ymax * frame.rows), 0, frame.rows - 1);

  cv::rectangle(frame, {xmin, ymin}, {xmax, ymax}, {0, 255, 0}, 2);
  const auto caption =
    label + " " + std::to_string(static_cast<int>(detection.confidence * 100)) + "%";
  cv::putText(
    frame, caption, {xmin, std::max(20, ymin - 8)},
    cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 0}, 2, cv::LINE_AA);
}

}  // namespace arcanain_depthai_ros2

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<arcanain_depthai_ros2::InferenceNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("inference_node"), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
