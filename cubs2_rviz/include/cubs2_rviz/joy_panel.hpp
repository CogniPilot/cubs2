#pragma once

#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <cubs2_msgs/msg/aircraft_control.hpp>

namespace cubs2 {

// Custom 2D joystick widget for aileron/elevator control
class JoystickWidget : public QWidget {
  Q_OBJECT
public:
  explicit JoystickWidget(QWidget* parent = nullptr);
  QSize sizeHint() const override { return QSize(200, 200); }

  double getAileron() const { return aileron_; }    // -1 to 1
  double getElevator() const { return elevator_; }  // -1 to 1

  void setTrim(double ail_trim, double elev_trim) {
    aileron_trim_ = ail_trim;
    elevator_trim_ = elev_trim;
    update();
  }

  void reset() {
    aileron_ = 0.0;
    elevator_ = 0.0;
    aileron_trim_ = 0.0;
    elevator_trim_ = 0.0;
    update();
  }

  void setPosition(double aileron, double elevator) {
    aileron_ = aileron;
    elevator_ = elevator;
    update();
  }

Q_SIGNALS:
  void joystickMoved(double aileron, double elevator);

protected:
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;

private Q_SLOTS:
  void springBackStep();

private:
  void updatePosition(const QPoint& pos);
  void startSpringBack();

  double aileron_{0.0};   // -1 (left) to +1 (right)
  double elevator_{0.0};  // -1 (down) to +1 (up)
  double aileron_trim_{0.0};
  double elevator_trim_{0.0};
  bool dragging_{false};
  QTimer* spring_timer_{nullptr};
};

class JoyPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit JoyPanel(QWidget* parent = nullptr);
  ~JoyPanel() override;

private Q_SLOTS:
  void onJoystickMoved(double aileron, double elevator);
  void onThrottleChanged(int value);
  void onRudderChanged(int value);
  void onAileronTrimChanged(int value);
  void onElevatorTrimChanged(int value);
  void publishControlInputs();
  void onResetClicked();
  void onEnabledChanged(int state);

private:
  void controlCallback(const cubs2_msgs::msg::AircraftControl::SharedPtr msg);
  void updateDisplayFromExternal(double aileron, double elevator, double throttle, double rudder);

private:
  // Virtual joystick controls
  JoystickWidget* joystick_{nullptr};
  QSlider* throttle_slider_{nullptr};
  QSlider* rudder_slider_{nullptr};
  QSlider* aileron_trim_slider_{nullptr};
  QSlider* elevator_trim_slider_{nullptr};
  QLabel* throttle_label_{nullptr};
  QLabel* rudder_label_{nullptr};
  QLabel* aileron_trim_label_{nullptr};
  QLabel* elevator_trim_label_{nullptr};
  QTimer* control_timer_{nullptr};
  QPushButton* reset_button_{nullptr};
  QCheckBox* enable_checkbox_{nullptr};

  double aileron_{0.0};
  double elevator_{0.0};
  double throttle_{0.0};
  double rudder_{0.0};
  double aileron_trim_{0.0};
  double elevator_trim_{0.0};
  bool enabled_{true};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<cubs2_msgs::msg::AircraftControl>::SharedPtr joy_publisher_{nullptr};
  rclcpp::Subscription<cubs2_msgs::msg::AircraftControl>::SharedPtr joy_subscriber_{nullptr};
};

}  // namespace cubs2
