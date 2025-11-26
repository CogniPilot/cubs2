// Copyright 2025 CogniPilot Foundation
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
#include "cubs2_rviz/sim_panel.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace cubs2
{

SimPanel::SimPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout;

  // Reset button
  reset_button_ = new QPushButton("Reset Simulation");
  layout->addWidget(reset_button_);
  connect(reset_button_, &QPushButton::clicked, this, &SimPanel::onResetButtonClicked);

  // Pause button
  pause_checkbox_ = new QCheckBox("Pause simulation");
  pause_checkbox_->setChecked(false);
  layout->addWidget(pause_checkbox_);
  connect(pause_checkbox_, &QCheckBox::toggled, this, &SimPanel::onPauseToggled);

  // Playback speed selector
  speed_combo_ = new QComboBox();
  speed_combo_->addItem("0.25x", 0.25);
  speed_combo_->addItem("0.5x", 0.5);
  speed_combo_->addItem("1x", 1.0);
  speed_combo_->addItem("2x", 2.0);
  speed_combo_->addItem("5x", 5.0);
  speed_combo_->addItem("10x", 10.0);
  speed_combo_->addItem("20x", 20.0);
  speed_combo_->addItem("50x", 50.0);
  speed_combo_->addItem("100x", 100.0);
  speed_combo_->setCurrentIndex(2);
  layout->addWidget(speed_combo_);
  connect(speed_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &SimPanel::onSpeedChanged);

  // Time step selector
  // Note: Ground contact dynamics require dt < ~3ms for Nyquist stability
  // (ωₙ=350 rad/s)
  dt_combo_ = new QComboBox();
  dt_combo_->addItem("dt: 0.001s", 0.001);
  dt_combo_->addItem("dt: 0.002s", 0.002);
  dt_combo_->addItem("dt: 0.005s", 0.005);
  dt_combo_->addItem("dt: 0.01s", 0.01);
  dt_combo_->addItem("dt: 0.1s", 0.1);
  dt_combo_->setCurrentIndex(3);  // Default to 0.01s
  layout->addWidget(dt_combo_);
  connect(dt_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &SimPanel::onDtChanged);

  setLayout(layout);

  node_ = std::make_shared<rclcpp::Node>("cubs2_reset_panel");
  reset_publisher_ = node_->create_publisher<std_msgs::msg::Empty>("/reset", 10);
  pause_publisher_ = node_->create_publisher<std_msgs::msg::Empty>("/pause", 10);
  speed_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/set_speed", 10);
  dt_publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/set_dt", 10);
  paused_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/sat/paused", 10, std::bind(&SimPanel::onPausedMsg, this, std::placeholders::_1));

  // Also listen for /reset from anywhere and reset UI controls accordingly
  reset_subscriber_ = node_->create_subscription<std_msgs::msg::Empty>(
      "/reset", 10, [this](const std_msgs::msg::Empty::SharedPtr /*msg*/) {
      RCLCPP_INFO(node_->get_logger(), "SimPanel: /reset received -> resetting UI controls");
      resetUiControls();
      });

  // Create timer for spinning ROS2 node (process callbacks in Qt thread)
  ros_spin_timer_ = new QTimer(this);
  ros_spin_timer_->setInterval(10);  // 100 Hz for responsive callbacks
  connect(ros_spin_timer_, &QTimer::timeout, this, [this]() {rclcpp::spin_some(node_);});
  ros_spin_timer_->start();
}

SimPanel::~SimPanel() = default;

void SimPanel::onInitialize()
{
  RCLCPP_INFO(node_->get_logger(), "SimPanel::onInitialize() called");
  rviz_common::Panel::onInitialize();
  context_ = getDisplayContext();

  if (!camera_pos_publisher_) {
    camera_pos_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PointStamped>("/viz/camera_position", 10);
    RCLCPP_INFO(node_->get_logger(), "Created camera position publisher");
  }

  if (!camera_timer_) {
    camera_timer_ = new QTimer(this);
    camera_timer_->setInterval(100);  // 10 Hz
    connect(camera_timer_, &QTimer::timeout, this, &SimPanel::publishCameraPosition);
    camera_timer_->start();
    RCLCPP_INFO(node_->get_logger(), "Started camera position timer");
  }

  // Publish default controls immediately on startup (speed=1x, dt=1s)
  resetUiControls(true);
  ui_initialized_ = true;  // avoid re-normalizing on first /sat/paused
  RCLCPP_INFO(node_->get_logger(), "Published default speed/dt on startup");
}

void SimPanel::onResetButtonClicked()
{
  std_msgs::msg::Empty msg;
  reset_publisher_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Published /reset message");
  speed_combo_->setCurrentIndex(2);
  dt_combo_->setCurrentIndex(2);
}

void SimPanel::onPauseToggled(bool checked)
{
  // The node currently toggles pause state on each /pause message.
  // Publishing here simply flips the state to match the checkbox.
  std_msgs::msg::Empty msg;
  pause_publisher_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), checked ? "Pause checkbox checked -> requesting pause" :
                                             "Pause checkbox unchecked -> requesting resume");
}

void SimPanel::onPausedMsg(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Update checkbox to reflect actual paused state from the node.
  // Block signals to avoid echoing a /pause message back and creating a loop.
  bool old = pause_checkbox_->blockSignals(true);
  pause_checkbox_->setChecked(msg->data);
  pause_checkbox_->blockSignals(old);

  // On first paused-state message (node just started), normalize combo boxes
  if (!ui_initialized_) {
    RCLCPP_INFO(node_->get_logger(), "SimPanel: first /sat/paused -> normalizing combo boxes");
    resetUiControls();
    ui_initialized_ = true;
  }
}

void SimPanel::onSpeedChanged(int index)
{
  double speed = speed_combo_->itemData(index).toDouble();
  std_msgs::msg::Float64 msg;
  msg.data = speed;
  speed_publisher_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Set sim speed to %.2fx", speed);
}

void SimPanel::onDtChanged(int index)
{
  double dt = dt_combo_->itemData(index).toDouble();
  std_msgs::msg::Float64 msg;
  msg.data = dt;
  dt_publisher_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "Set time step to %.3fs", dt);
}

void SimPanel::publishCameraPosition()
{
  if (!context_ || !camera_pos_publisher_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Cannot publish camera position: context_=%p, camera_pos_publisher_=%p",
                         static_cast<void *>(context_),
                         static_cast<void *>(camera_pos_publisher_.get()));
    return;
  }

  auto vm = context_->getViewManager();
  if (!vm) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "ViewManager is null");
    return;
  }
  auto vc = vm->getCurrent();
  if (!vc) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Current ViewController is null");
    return;
  }

  // Retrieve camera position from current view controller's Ogre camera
  Ogre::Camera * cam = nullptr;
  try {
    cam = vc->getCamera();
  } catch (...) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Exception getting camera from ViewController");
    cam = nullptr;
  }
  if (!cam) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Camera is null");
    return;
  }
  const Ogre::Vector3 pos = cam->getDerivedPosition();

  geometry_msgs::msg::PointStamped msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp = node_->now();
  msg.point.x = static_cast<double>(pos.x);
  msg.point.y = static_cast<double>(pos.y);
  msg.point.z = static_cast<double>(pos.z);
  camera_pos_publisher_->publish(msg);
}

void SimPanel::resetUiControls(bool emit_signals)
{
  // Restore default selections: speed=1x (index 2), dt=0.01s (index 3)
  // Optionally suppress signals if we don't want to publish set_speed/set_dt.
  if (!speed_combo_ || !dt_combo_) {
    return;
  }
  if (!emit_signals) {
    bool old1 = speed_combo_->blockSignals(true);
    bool old2 = dt_combo_->blockSignals(true);
    speed_combo_->setCurrentIndex(2);
    dt_combo_->setCurrentIndex(3);  // 0.01s default
    dt_combo_->blockSignals(old2);
    speed_combo_->blockSignals(old1);
  } else {
    speed_combo_->setCurrentIndex(2);
    dt_combo_->setCurrentIndex(3);  // 0.01s default
  }
}

}  // namespace cubs2

PLUGINLIB_EXPORT_CLASS(cubs2::SimPanel, rviz_common::Panel)
