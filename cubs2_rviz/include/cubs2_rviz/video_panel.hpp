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
#ifndef CUBS2_RVIZ__VIDEO_PANEL_HPP_
#define CUBS2_RVIZ__VIDEO_PANEL_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <QComboBox>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QMutex>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace cubs2 {

// Custom widget for displaying video stream
class VideoWidget : public QLabel {
  Q_OBJECT

public:
  explicit VideoWidget(QWidget* parent = nullptr);
  void setFrame(const QImage& frame);

protected:
  void resizeEvent(QResizeEvent* event) override;

private:
  QImage current_frame_;
  void updateDisplay();
};

// RViz panel for GStreamer video streaming
class VideoPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit VideoPanel(QWidget* parent = nullptr);
  ~VideoPanel() override;

  void onInitialize() override;

protected Q_SLOTS:
  void onConnectClicked();
  void onDisconnectClicked();
  void onSourceChanged(int index);
  void updateFrame();

private:
  // UI elements
  VideoWidget* video_widget_;
  QComboBox* source_combo_;
  QLineEdit* uri_edit_;
  QPushButton* connect_button_;
  QPushButton* disconnect_button_;
  QLabel* status_label_;

  // GStreamer pipeline
  GstElement* pipeline_;
  GstElement* appsink_;
  bool is_connected_;
  QMutex frame_mutex_;
  QImage pending_frame_;
  bool has_new_frame_;

  // ROS 2 display context
  rviz_common::DisplayContext* context_;
  QTimer* frame_timer_;

  // Helper methods
  bool initializeGStreamer();
  void cleanupGStreamer();
  bool createPipeline(const std::string& uri);

  // Predefined video sources
  struct VideoSource {
    QString name;
    QString uri;
    QString description;
  };
  std::vector<VideoSource> predefined_sources_;
  void setupPredefinedSources();

  // GStreamer callback
  static GstFlowReturn on_new_sample(GstAppSink* appsink, gpointer user_data);
};

}  // namespace cubs2

#endif  // CUBS2_RVIZ__VIDEO_PANEL_HPP_
