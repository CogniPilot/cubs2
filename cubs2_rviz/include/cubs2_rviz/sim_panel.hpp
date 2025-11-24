#pragma once

#include <Ogre.h>
#include <QCheckBox>
#include <QComboBox>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/view_manager.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>

namespace cubs2 {

class SimPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit SimPanel(QWidget* parent = nullptr);
  ~SimPanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void onResetButtonClicked();
  void onPauseToggled(bool checked);
  void onSpeedChanged(int index);
  void onDtChanged(int index);
  void onPausedMsg(const std_msgs::msg::Bool::SharedPtr msg);
  void publishCameraPosition();
  void resetUiControls(bool emit_signals = true);

private:
  QPushButton* reset_button_;
  QCheckBox* pause_checkbox_;
  QComboBox* speed_combo_;
  QComboBox* dt_combo_;
  QTimer* camera_timer_{nullptr};
  QTimer* ros_spin_timer_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pause_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr dt_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr paused_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr camera_pos_publisher_{nullptr};
  rviz_common::DisplayContext* context_{nullptr};

  // Ensure UI resets to defaults on first sim paused-state message
  bool ui_initialized_{false};
};

}  // namespace cubs2
