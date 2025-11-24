#include "cubs2_rviz/hud_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <thread>

namespace cubs2 {

// ============================================================================
// HUDWidget Implementation
// ============================================================================

HUDWidget::HUDWidget(QWidget* parent) : QWidget(parent) {
  setMinimumSize(400, 400);
  setStyleSheet("background-color: rgba(0, 0, 0, 180);");
}

void HUDWidget::setAttitude(double roll, double pitch, double yaw) {
  roll_ = roll;
  pitch_ = pitch;
  yaw_ = yaw;
  update();
}

void HUDWidget::setAltitude(double altitude) {
  altitude_ = altitude;
  update();
}

void HUDWidget::setAirspeed(double airspeed) {
  airspeed_ = airspeed;
  update();
}

void HUDWidget::paintEvent(QPaintEvent* /* event */) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  int w = width();
  int h = height();
  int cx = w / 2;
  int cy = h / 2;
  int size = qMin(w, h) - 40;

  // Draw artificial horizon
  drawHorizon(painter, cx, cy, size);

  // Draw roll indicator
  drawRollIndicator(painter, cx, cy, size);

  // Draw text information
  drawText(painter);

  // Draw center crosshair
  painter.setPen(QPen(QColor(0, 255, 0), 2));
  painter.drawLine(cx - 30, cy, cx - 10, cy);
  painter.drawLine(cx + 10, cy, cx + 30, cy);
  painter.drawLine(cx, cy - 10, cx, cy - 5);
}

void HUDWidget::drawHorizon(QPainter& painter, int cx, int cy, int size) {
  painter.save();

  // Translate to center and apply roll rotation
  painter.translate(cx, cy);
  painter.rotate(-roll_ * 180.0 / M_PI);  // Negative for correct direction

  int radius = size / 2;

  // Apply pitch offset (pixels per degree)
  double pixels_per_rad = radius / 1.5;  // ~60 degrees visible range
  int pitch_offset = static_cast<int>(pitch_ * pixels_per_rad);

  // Make rectangles extra large to avoid gaps when rotated
  int extra = radius * 2;  // Extra size to cover rotation

  // Draw sky (blue) - fill entire background
  painter.setBrush(QColor(135, 206, 235, 200));
  painter.setPen(Qt::NoPen);
  painter.drawRect(-radius - extra, -radius - extra, 2 * (radius + extra), 2 * (radius + extra));

  // Draw ground (brown) - layered on top, extends from horizon downward
  painter.setBrush(QColor(139, 90, 43, 200));
  painter.drawRect(-radius - extra, -pitch_offset, 2 * (radius + extra),
                   radius + extra + pitch_offset);

  // Draw horizon line
  painter.setPen(QPen(QColor(255, 255, 255), 3));
  painter.drawLine(-radius, -pitch_offset, radius, -pitch_offset);

  // Draw pitch ladder (every 10 degrees)
  painter.setPen(QPen(QColor(255, 255, 255), 2));
  QFont font = painter.font();
  font.setPixelSize(12);
  painter.setFont(font);

  for (int deg = -60; deg <= 60; deg += 10) {
    if (deg == 0) continue;  // Skip horizon line

    double rad = deg * M_PI / 180.0;
    int y = -pitch_offset - static_cast<int>(rad * pixels_per_rad);

    // Only draw if visible
    if (std::abs(y) < radius) {
      int line_length = (deg % 20 == 0) ? 40 : 20;
      painter.drawLine(-line_length, y, line_length, y);

      // Draw angle text
      if (deg % 20 == 0) {
        painter.drawText(-line_length - 25, y + 5, QString::number(deg));
        painter.drawText(line_length + 10, y + 5, QString::number(deg));
      }
    }
  }

  painter.restore();
}

void HUDWidget::drawRollIndicator(QPainter& painter, int cx, int cy, int size) {
  painter.save();
  painter.translate(cx, cy);

  int radius = size / 2 - 30;  // Moved inside, reduced from size/2+20 to size/2-30

  // Draw roll arc
  painter.setPen(QPen(QColor(255, 255, 255), 2));
  painter.drawArc(-radius, -radius, 2 * radius, 2 * radius, 30 * 16, 120 * 16);

  // Set font for roll angle labels
  QFont font = painter.font();
  font.setPixelSize(12);
  painter.setFont(font);

  // Draw tick marks and labels (every 10 degrees)
  for (int deg = -60; deg <= 60; deg += 10) {
    double rad = deg * M_PI / 180.0;
    int x1 = static_cast<int>(radius * std::sin(rad));
    int y1 = static_cast<int>(-radius * std::cos(rad));

    int tick_len = (deg == 0 || deg == -30 || deg == 30 || deg == -60 || deg == 60) ? 15 : 10;
    int x2 = static_cast<int>((radius - tick_len) * std::sin(rad));
    int y2 = static_cast<int>(-(radius - tick_len) * std::cos(rad));

    painter.drawLine(x1, y1, x2, y2);

    // Draw angle labels for major tick marks
    if (deg % 30 == 0 && deg != 0) {
      int label_x = static_cast<int>((radius - tick_len - 15) * std::sin(rad));
      int label_y = static_cast<int>(-(radius - tick_len - 15) * std::cos(rad));
      painter.drawText(label_x - 10, label_y + 5, QString::number(std::abs(deg)));
    }
  }

  // Draw current roll indicator (triangle)
  painter.rotate(-roll_ * 180.0 / M_PI);
  painter.setBrush(QColor(255, 255, 0));
  painter.setPen(QPen(QColor(200, 200, 0), 2));

  QPolygon triangle;
  triangle << QPoint(0, -radius + 5) << QPoint(-8, -radius + 18) << QPoint(8, -radius + 18);
  painter.drawPolygon(triangle);

  painter.restore();
}

void HUDWidget::drawText(QPainter& painter) {
  QFont font = painter.font();
  font.setPixelSize(18);
  font.setBold(true);
  painter.setFont(font);

  int w = width();

  // Altitude (left side) - white text on dark grey background
  QString altText = QString("ALT\n%1 m").arg(altitude_, 0, 'f', 1);
  QFontMetrics fm(font);
  QRect altRect = fm.boundingRect(QRect(0, 0, 200, 100), Qt::AlignLeft | Qt::TextWordWrap, altText);
  altRect.moveTo(10, 10);
  altRect.adjust(-5, -5, 5, 5);  // Add padding

  painter.fillRect(altRect, QColor(60, 60, 60, 180));  // Semi-transparent dark grey background
  painter.setPen(Qt::white);
  painter.drawText(altRect, Qt::AlignCenter, altText);

  // Airspeed (right side) - white text on dark grey background
  QString iasText = QString("IAS\n%1 m/s").arg(airspeed_, 0, 'f', 1);
  QRect iasRect = fm.boundingRect(QRect(0, 0, 200, 100), Qt::AlignLeft | Qt::TextWordWrap, iasText);
  iasRect.moveTo(w - iasRect.width() - 15, 10);
  iasRect.adjust(-5, -5, 5, 5);  // Add padding

  painter.fillRect(iasRect, QColor(60, 60, 60, 180));  // Semi-transparent dark grey background
  painter.setPen(Qt::white);
  painter.drawText(iasRect, Qt::AlignCenter, iasText);
}

// ============================================================================
// HUDPanel Implementation
// ============================================================================

HUDPanel::HUDPanel(QWidget* parent) : rviz_common::Panel(parent) {
  auto* layout = new QVBoxLayout;

  // Create HUD widget
  hud_widget_ = new HUDWidget(this);
  layout->addWidget(hud_widget_);

  setLayout(layout);

  // Create ROS2 node
  node_ = std::make_shared<rclcpp::Node>("hud_panel_node");

  // Subscribe to pose topic
  pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/sportcub/pose", 10, std::bind(&HUDPanel::onPoseReceived, this, std::placeholders::_1));

  // Subscribe to velocity topic (for airspeed)
  velocity_subscription_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/sportcub/velocity", 10,
      std::bind(&HUDPanel::onVelocityReceived, this, std::placeholders::_1));

  // Create timer for spinning ROS2 node (process callbacks in Qt thread)
  ros_spin_timer_ = new QTimer(this);
  ros_spin_timer_->setInterval(10);  // 100 Hz for responsive callbacks
  connect(ros_spin_timer_, &QTimer::timeout, this, [this]() { rclcpp::spin_some(node_); });
  ros_spin_timer_->start();
}

HUDPanel::~HUDPanel() = default;

void HUDPanel::onPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Extract altitude (z position)
  altitude_ = msg->pose.position.z;

  // Convert quaternion to Euler angles
  tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                    msg->pose.orientation.w);

  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  // Update HUD
  hud_widget_->setAttitude(roll_, pitch_, yaw_);
  hud_widget_->setAltitude(altitude_);
}

void HUDPanel::onVelocityReceived(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  // Calculate airspeed magnitude from velocity
  double vx = msg->twist.linear.x;
  double vy = msg->twist.linear.y;
  double vz = msg->twist.linear.z;
  airspeed_ = std::sqrt(vx * vx + vy * vy + vz * vz);

  hud_widget_->setAirspeed(airspeed_);
}

}  // namespace cubs2

PLUGINLIB_EXPORT_CLASS(cubs2::HUDPanel, rviz_common::Panel)
