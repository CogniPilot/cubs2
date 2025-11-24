#ifndef FIXED_WING_PURT_HUD_PANEL_HPP
#define FIXED_WING_PURT_HUD_PANEL_HPP

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <QLabel>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace cubs2 {

// Custom widget for drawing the HUD display
class HUDWidget : public QWidget {
  Q_OBJECT

public:
  explicit HUDWidget(QWidget* parent = nullptr);

  void setAttitude(double roll, double pitch, double yaw);
  void setAltitude(double altitude);
  void setAirspeed(double airspeed);

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  double roll_{0.0};      // radians
  double pitch_{0.0};     // radians
  double yaw_{0.0};       // radians
  double altitude_{0.0};  // meters
  double airspeed_{0.0};  // m/s

  void drawHorizon(QPainter& painter, int cx, int cy, int size);
  void drawRollIndicator(QPainter& painter, int cx, int cy, int size);
  void drawText(QPainter& painter);
};

// RViz panel that subscribes to pose and velocity topics
class HUDPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit HUDPanel(QWidget* parent = nullptr);
  ~HUDPanel() override;

private:
  void onPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onVelocityReceived(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscription_;

  HUDWidget* hud_widget_{nullptr};

  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};
  double altitude_{0.0};
  double airspeed_{0.0};
};

}  // namespace cubs2

#endif  // FIXED_WING_PURT_HUD_PANEL_HPP
