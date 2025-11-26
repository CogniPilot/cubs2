#!/usr/bin/env python3
# Copyright 2025 CogniPilot Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Filter rosbag to only include specific topics."""
import os
import sys

import rosbag2_py

# Configuration
if len(sys.argv) < 2:
    print('Usage: filter_bag.py <input_bag.mcap>')
    sys.exit(1)

INPUT_BAG = sys.argv[1]
OUTPUT_BAG = INPUT_BAG.replace('.mcap', '_filtered')
TOPICS_TO_KEEP = ['/cub1/pose', '/cub1/joy_serial_status']


def filter_bag():
    """Filter the bag file to only include specified topics."""
    # Remove old output if it exists
    if os.path.exists(OUTPUT_BAG):
        os.system(f'rm -rf {OUTPUT_BAG}')

    # Open input bag for reading
    storage_options = rosbag2_py.StorageOptions(
        uri=INPUT_BAG, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # Open output bag for writing
    output_storage_options = rosbag2_py.StorageOptions(
        uri=OUTPUT_BAG, storage_id='mcap')
    writer = rosbag2_py.SequentialWriter()
    writer.open(output_storage_options, converter_options)

    # Create topics in output bag
    topic_id = 0
    for topic_name in TOPICS_TO_KEEP:
        if topic_name in type_map:
            topic = rosbag2_py.TopicMetadata(
                id=topic_id,
                name=topic_name,
                type=type_map[topic_name],
                serialization_format='cdr',
            )
            writer.create_topic(topic)
            topic_id += 1

    # Filter and write messages
    msg_count = 0
    print(f'Filtering bag: {INPUT_BAG} -> {OUTPUT_BAG}')
    print(f'Topics to keep: {TOPICS_TO_KEEP}')

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()

        if topic in TOPICS_TO_KEEP:
            writer.write(topic, data, timestamp)
            msg_count += 1
            if msg_count % 1000 == 0:
                print(f'  Processed {msg_count} messages...')

    print(f'\nComplete! Wrote {msg_count} messages to {OUTPUT_BAG}')

    # Check output size
    if os.path.exists(f'{OUTPUT_BAG}/{OUTPUT_BAG}_0.mcap'):
        size_mb = os.path.getsize(
            f'{OUTPUT_BAG}/{OUTPUT_BAG}_0.mcap') / (1024 * 1024)
        print(f'Output size: {size_mb:.2f} MB')


if __name__ == '__main__':
    filter_bag()
