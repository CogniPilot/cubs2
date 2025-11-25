#!/bin/bash
# Test GStreamer Video Stream for Cubs2 RViz Video Panel
# This script creates a test pattern video stream that simulates a drone camera feed

echo "Cubs2 Video Panel Test Stream Generator"
echo "========================================"
echo ""
echo "This will create a test video stream that you can view in RViz."
echo "The test pattern includes a moving ball, timestamp, and frame counter."
echo ""

# Check if GStreamer is installed
if ! command -v gst-launch-1.0 &> /dev/null; then
    echo "ERROR: GStreamer is not installed!"
    echo "Install with: sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-good"
    exit 1
fi

# Default parameters
WIDTH=640
HEIGHT=480
FPS=30
PORT=5600
PATTERN="ball"  # Options: smpte, snow, black, white, red, green, blue, checkers-1, ball, etc.

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -w|--width)
            WIDTH="$2"
            shift 2
            ;;
        -h|--height)
            HEIGHT="$2"
            shift 2
            ;;
        -f|--fps)
            FPS="$2"
            shift 2
            ;;
        -p|--port)
            PORT="$2"
            shift 2
            ;;
        --pattern)
            PATTERN="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -w, --width WIDTH      Video width (default: 640)"
            echo "  -h, --height HEIGHT    Video height (default: 480)"
            echo "  -f, --fps FPS          Frames per second (default: 30)"
            echo "  -p, --port PORT        UDP port (default: 5600)"
            echo "  --pattern PATTERN      Test pattern (default: ball)"
            echo "                         Options: smpte, ball, checkers-1, snow, etc."
            echo "  --help                 Show this help"
            echo ""
            echo "Examples:"
            echo "  $0                           # Basic test with default settings"
            echo "  $0 --pattern smpte           # SMPTE color bars"
            echo "  $0 -w 1280 -h 720 -f 60      # HD at 60fps"
            echo ""
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "Stream Configuration:"
echo "  Resolution: ${WIDTH}x${HEIGHT}"
echo "  FPS: ${FPS}"
echo "  Pattern: ${PATTERN}"
echo "  UDP Port: ${PORT}"
echo ""
echo "Starting test stream..."
echo ""
echo "To view in RViz:"
echo "  1. Launch RViz: rviz2"
echo "  2. Add Cubs2 Video Panel (Panels -> Add New Panel -> cubs2::VideoPanel)"
echo "  3. Select 'UDP Stream (H.264)' or use custom URI:"
echo "     udpsrc port=${PORT} ! application/x-rtp,encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264"
echo "  4. Click 'Connect'"
echo ""
echo "Press Ctrl+C to stop the stream"
echo ""

# Create the GStreamer pipeline
# Using H.264 encoding for realistic drone camera simulation
gst-launch-1.0 -v \
    videotestsrc pattern=${PATTERN} is-live=true ! \
    video/x-raw,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1 ! \
    clockoverlay time-format="%Y-%m-%d %H:%M:%S" ! \
    textoverlay text="Cubs2 Test Feed" valignment=top halignment=left font-desc="Sans 20" ! \
    x264enc tune=zerolatency bitrate=2000 speed-preset=superfast key-int-max=30 ! \
    rtph264pay config-interval=1 pt=96 ! \
    udpsink host=127.0.0.1 port=${PORT} sync=false

echo ""
echo "Stream stopped."
