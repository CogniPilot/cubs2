#!/bin/bash
# Download rosbag files
# Rosbag files are hosted on Google Drive

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="$SCRIPT_DIR/../data"

echo "Downloading rosbag files to $DATA_DIR"
mkdir -p "$DATA_DIR"

# Google Drive folder: https://drive.google.com/drive/folders/1G_zh7rY5Au3LstkbNXrgmPpwtEmabTle?usp=sharing

# Function to download from Google Drive
download_gdrive() {
    local file_id=$1
    local output=$2
    
    echo "Downloading $output..."
    wget --no-check-certificate "https://drive.google.com/uc?export=download&id=${file_id}" -O "$output"
}

# Rosbag file IDs
STABILIZE_FILE_ID="1iqMQ5RkGm1Xoe_Fsk4vxxpnLmrNbGNy4"

# Download stabilized flight rosbag
if [ ! -f "$DATA_DIR/cub_stabilize_2025-10-21.mcap" ]; then
    download_gdrive "$STABILIZE_FILE_ID" "$DATA_DIR/cub_stabilize_2025-10-21.mcap"
    echo "Download complete!"
else
    echo "cub_stabilize_2025-10-21.mcap already exists, skipping download"
fi

echo ""
echo "All rosbag files downloaded successfully!"
echo "File location: $DATA_DIR/"
