#include "cubs2_rviz/video_panel.hpp"
#include <QGridLayout>
#include <QGroupBox>

namespace cubs2 {

// ============================================================================
// VideoWidget Implementation
// ============================================================================

VideoWidget::VideoWidget(QWidget* parent) : QLabel(parent) {
  setMinimumSize(320, 240);
  setAlignment(Qt::AlignCenter);
  setStyleSheet("QLabel { background-color: black; }");
  setText("No Video Stream");
  setScaledContents(false);
}

void VideoWidget::setFrame(const QImage& frame) {
  current_frame_ = frame;
  updateDisplay();
}

void VideoWidget::resizeEvent(QResizeEvent* event) {
  QLabel::resizeEvent(event);
  updateDisplay();
}

void VideoWidget::updateDisplay() {
  if (current_frame_.isNull()) {
    return;
  }

  // Scale image to fit widget while maintaining aspect ratio
  QImage scaled = current_frame_.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
  setPixmap(QPixmap::fromImage(scaled));
}

// ============================================================================
// VideoPanel Implementation
// ============================================================================

VideoPanel::VideoPanel(QWidget* parent)
    : rviz_common::Panel(parent),
      pipeline_(nullptr),
      appsink_(nullptr),
      is_connected_(false),
      has_new_frame_(false) {
  // Initialize GStreamer
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }

  // Create UI layout
  QVBoxLayout* main_layout = new QVBoxLayout;

  // Video display widget
  video_widget_ = new VideoWidget(this);
  main_layout->addWidget(video_widget_, 1);

  // Control panel
  QGroupBox* control_group = new QGroupBox("Video Source");
  QGridLayout* control_layout = new QGridLayout;

  // Source selection
  source_combo_ = new QComboBox;
  setupPredefinedSources();
  for (const auto& source : predefined_sources_) {
    source_combo_->addItem(source.name);
  }
  source_combo_->addItem("Custom URI");
  control_layout->addWidget(new QLabel("Source:"), 0, 0);
  control_layout->addWidget(source_combo_, 0, 1);

  // URI input
  uri_edit_ = new QLineEdit;
  uri_edit_->setPlaceholderText("rtsp://192.168.1.100:8554/video");
  uri_edit_->setEnabled(false);
  control_layout->addWidget(new QLabel("URI:"), 1, 0);
  control_layout->addWidget(uri_edit_, 1, 1);

  // Connect/Disconnect buttons
  connect_button_ = new QPushButton("Connect");
  disconnect_button_ = new QPushButton("Disconnect");
  disconnect_button_->setEnabled(false);
  control_layout->addWidget(connect_button_, 2, 0);
  control_layout->addWidget(disconnect_button_, 2, 1);

  // Status label
  status_label_ = new QLabel("Disconnected");
  status_label_->setStyleSheet("QLabel { color: red; }");
  control_layout->addWidget(status_label_, 3, 0, 1, 2);

  control_group->setLayout(control_layout);
  main_layout->addWidget(control_group);

  setLayout(main_layout);

  // Connect signals
  connect(source_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(onSourceChanged(int)));
  connect(connect_button_, SIGNAL(clicked()), this, SLOT(onConnectClicked()));
  connect(disconnect_button_, SIGNAL(clicked()), this, SLOT(onDisconnectClicked()));

  // Frame update timer
  frame_timer_ = new QTimer(this);
  connect(frame_timer_, SIGNAL(timeout()), this, SLOT(updateFrame()));
}

VideoPanel::~VideoPanel() {
  cleanupGStreamer();
}

void VideoPanel::onInitialize() {
  rviz_common::Panel::onInitialize();
  context_ = getDisplayContext();
}

void VideoPanel::setupPredefinedSources() {
  predefined_sources_ = {
      {"Test Pattern",
       "videotestsrc pattern=smpte ! video/x-raw,width=640,height=480,framerate=30/1",
       "SMPTE color bars test pattern"},

      {"USB Camera (default)",
       "v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480,framerate=30/1",
       "Default USB camera /dev/video0"},

      {"RTSP Stream (example)",
       "rtspsrc location=rtsp://192.168.1.100:8554/video latency=0 ! decodebin",
       "RTSP network stream"},

      {"UDP Stream (H.264)",
       "udpsrc port=5600 ! application/x-rtp,encoding-name=H264 ! rtph264depay ! avdec_h264",
       "UDP H.264 stream on port 5600"},

      {"UDP Stream (MJPEG)",
       "udpsrc port=5600 ! application/x-rtp,encoding-name=JPEG ! rtpjpegdepay ! jpegdec",
       "UDP MJPEG stream on port 5600"},
  };
}

void VideoPanel::onSourceChanged(int index) {
  bool is_custom = (index == static_cast<int>(predefined_sources_.size()));
  uri_edit_->setEnabled(is_custom);

  if (!is_custom && index >= 0 && index < static_cast<int>(predefined_sources_.size())) {
    uri_edit_->setText(predefined_sources_[index].uri);
    status_label_->setText(predefined_sources_[index].description);
  } else if (is_custom) {
    status_label_->setText("Enter custom GStreamer pipeline");
  }
}

void VideoPanel::onConnectClicked() {
  std::string uri = uri_edit_->text().toStdString();

  if (uri.empty()) {
    status_label_->setText("Error: Empty URI");
    status_label_->setStyleSheet("QLabel { color: red; }");
    return;
  }

  if (createPipeline(uri)) {
    is_connected_ = true;
    connect_button_->setEnabled(false);
    disconnect_button_->setEnabled(true);
    source_combo_->setEnabled(false);
    uri_edit_->setEnabled(false);
    status_label_->setText("Connected");
    status_label_->setStyleSheet("QLabel { color: green; }");

    // Start frame update timer
    frame_timer_->start(33);  // ~30 fps

    // Start pipeline
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  } else {
    status_label_->setText("Error: Failed to create pipeline");
    status_label_->setStyleSheet("QLabel { color: red; }");
  }
}

void VideoPanel::onDisconnectClicked() {
  is_connected_ = false;
  cleanupGStreamer();
  connect_button_->setEnabled(true);
  disconnect_button_->setEnabled(false);
  source_combo_->setEnabled(true);

  int index = source_combo_->currentIndex();
  bool is_custom = (index == static_cast<int>(predefined_sources_.size()));
  uri_edit_->setEnabled(is_custom);

  status_label_->setText("Disconnected");
  status_label_->setStyleSheet("QLabel { color: red; }");

  video_widget_->clear();
  video_widget_->setText("No Video Stream");
}

bool VideoPanel::createPipeline(const std::string& uri) {
  // Build complete pipeline with videoconvert and appsink
  std::string pipeline_str =
      uri +
      " ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink emit-signals=true sync=false";

  GError* error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

  if (error != nullptr) {
    status_label_->setText(QString("GStreamer error: ") + error->message);
    g_error_free(error);
    return false;
  }

  if (!pipeline_) {
    status_label_->setText("Failed to create pipeline");
    return false;
  }

  // Get appsink element
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
  if (!appsink_) {
    status_label_->setText("Failed to get appsink");
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return false;
  }

  // Configure appsink
  g_object_set(appsink_, "emit-signals", TRUE, "sync", FALSE, nullptr);

  // Set callback for new samples
  static GstAppSinkCallbacks callbacks = {
      nullptr,        // eos
      nullptr,        // new_preroll
      on_new_sample,  // new_sample
      nullptr,        // new_event
      nullptr,        // propose_allocation
      {}              // _gst_reserved
  };
  gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), &callbacks, this, nullptr);

  return true;
}

void VideoPanel::cleanupGStreamer() {
  // Stop timer first to prevent accessing freed GStreamer objects
  if (frame_timer_) {
    frame_timer_->stop();
  }

  // Clear callbacks before destroying pipeline to prevent race conditions
  if (appsink_) {
    gst_app_sink_set_callbacks(GST_APP_SINK(appsink_), nullptr, nullptr, nullptr);
    appsink_ = nullptr;
  }

  if (pipeline_) {
    // Stop pipeline first
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    // Wait for state change to complete to ensure all threads have stopped
    gst_element_get_state(pipeline_, nullptr, nullptr, GST_CLOCK_TIME_NONE);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

GstFlowReturn VideoPanel::on_new_sample(GstAppSink* appsink, gpointer user_data) {
  VideoPanel* panel = static_cast<VideoPanel*>(user_data);

  // Safety check - panel might be destroyed during shutdown
  if (!panel || !panel->pipeline_) {
    return GST_FLOW_FLUSHING;
  }

  GstSample* sample = gst_app_sink_pull_sample(appsink);
  if (!sample) {
    return GST_FLOW_ERROR;
  }

  GstCaps* caps = gst_sample_get_caps(sample);
  GstStructure* structure = gst_caps_get_structure(caps, 0);

  int width, height;
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;

  if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    // Create QImage from buffer (RGB format) - make deep copy
    QImage frame(map.data, width, height, width * 3, QImage::Format_RGB888);
    QImage frame_copy = frame.copy();

    gst_buffer_unmap(buffer, &map);

    // Store frame thread-safely for Qt event loop to display
    {
      QMutexLocker locker(&panel->frame_mutex_);
      panel->pending_frame_ = frame_copy;
      panel->has_new_frame_ = true;
    }
  }

  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

void VideoPanel::updateFrame() {
  // Check for new frame from GStreamer thread
  QImage frame_to_display;
  bool have_frame = false;

  {
    QMutexLocker locker(&frame_mutex_);
    if (has_new_frame_) {
      frame_to_display = pending_frame_;
      has_new_frame_ = false;
      have_frame = true;
    }
  }

  // Update widget in main thread (outside mutex lock)
  if (have_frame) {
    video_widget_->setFrame(frame_to_display);

    // Update status to show we're receiving frames
    if (is_connected_) {
      status_label_->setText("Connected - Receiving video");
      status_label_->setStyleSheet("QLabel { color: green; }");
    }
  }

  // Also check pipeline status for errors
  if (pipeline_ && is_connected_) {
    GstState state;
    gst_element_get_state(pipeline_, &state, nullptr, 0);

    if (state != GST_STATE_PLAYING && state != GST_STATE_PAUSED) {
      status_label_->setText("Warning: Pipeline stopped");
      status_label_->setStyleSheet("QLabel { color: orange; }");
    }
  }
}

}  // namespace cubs2

// Plugin export
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cubs2::VideoPanel, rviz_common::Panel)
