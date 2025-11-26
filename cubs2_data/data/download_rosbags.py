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
"""
Download rosbag files from Google Drive.

This script allows selective downloading of rosbag files hosted on Google Drive.
Run with --list to see available files, or specify IDs or file names to download.

"""
import argparse
from pathlib import Path
import subprocess
import sys

# Available rosbag files with Google Drive URLs
# Format: https://drive.google.com/uc?export=download&id=<file_id>
AVAILABLE_BAGS = [{'id': 1,
                   'filename': 'cub_stabilize_2025_10_21.mcap',
                   'url': (
                       'https://drive.google.com/uc?export=download&'
                       'id=11ourp5eBdvDx2e9HdO3fB2hCIJbOixIK'
                   ),
                   'description': 'Stabilized flight test - October 21, 2025 (filtered)',
                   'size': '1.4 MB',
                   },
                  {'id': 2,
                   'filename': 'cub_stabilize_2025_11_13.mcap',
                   'url': (
                       'https://drive.google.com/uc?export=download&'
                       'id=1zlkt57-Os6wgQ9rdz4i36VClBskRT2Fo'
                   ),
                   'description': 'Stabilized flight test - November 13, 2025 (filtered)',
                   'size': '2.6 MB',
                   },
                  ]


def download_file(url: str, output_path: Path) -> bool:
    """
    Download file from Google Drive using curl.

    Parameters
    ----------
    url : str
        Direct Google Drive download URL
    output_path : Path
        Destination path for downloaded file

    Returns
    -------
    bool
        True if download successful, False otherwise

    """
    try:
        print(f'Downloading {output_path.name}...')

        # Use curl with progress bar
        subprocess.run(
            [
                'curl',
                '-L',
                '-o',
                str(output_path),
                '-C',
                '-',  # Resume capability
                '--progress-bar',  # Show progress
                url,
            ],
            check=True,
        )

        print(f'✓ Downloaded {output_path.name}')
        return True
    except subprocess.CalledProcessError as e:
        print(f'✗ Failed to download {output_path.name}')
        print(f'  Error: {e}')
        return False
    except FileNotFoundError:
        print('Error: curl not found. Please install curl:')
        print('  sudo apt-get install curl')
        sys.exit(1)


def list_available_bags():
    """Display list of available rosbag files."""
    print('\nAvailable rosbag files:')
    print('=' * 80)

    for bag in AVAILABLE_BAGS:
        print(f"\nID: {bag['id']}")
        print(f"  Filename: {bag['filename']}")
        print(f"  Description: {bag['description']}")
        print(f"  Size: {bag['size']}")

    print('\n' + '=' * 80)
    print(f'\nTotal: {len(AVAILABLE_BAGS)} files')


def get_bag_by_id(bag_id: int):
    """Get bag info by ID."""
    for bag in AVAILABLE_BAGS:
        if bag['id'] == bag_id:
            return bag
    return None


def get_bag_by_filename(filename: str):
    """Get bag info by filename."""
    for bag in AVAILABLE_BAGS:
        if bag['filename'] == filename:
            return bag
    return None


def main():
    parser = argparse.ArgumentParser(
        description='Download Cubs2 rosbag files from Google Drive',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --list                          # List all available files
  %(prog)s --all                           # Download all files
  %(prog)s 1                               # Download bag with ID 1
  %(prog)s 1 2                             # Download bags with IDs 1 and 2
  %(prog)s cub_stabilize_2025_10_21.mcap   # Download by filename
  %(prog)s cub_*.mcap                      # Download files matching pattern
        """,
    )

    parser.add_argument(
        'files',
        nargs='*',
        help='Bag IDs or filenames to download (use --list to see available files)',
    )
    parser.add_argument('--list', '-l', action='store_true',
                        help='List all available rosbag files')
    parser.add_argument(
        '--all',
        '-a',
        action='store_true',
        help='Download all available rosbag files')
    parser.add_argument(
        '--output-dir',
        '-o',
        type=Path,
        default=None,
        help='Output directory (default: current directory)',
    )
    parser.add_argument(
        '--force',
        '-f',
        action='store_true',
        help='Force re-download even if file exists',
    )

    args = parser.parse_args()

    # Determine output directory
    if args.output_dir:
        output_dir = args.output_dir
    else:
        # Default to script's directory
        output_dir = Path(__file__).parent

    output_dir.mkdir(parents=True, exist_ok=True)

    # Handle --list
    if args.list:
        list_available_bags()
        return 0

    # Determine which files to download
    if args.all:
        bags_to_download = AVAILABLE_BAGS[:]
    elif args.files:
        bags_to_download = []

        for item in args.files:
            # Try to parse as ID first
            try:
                bag_id = int(item)
                bag = get_bag_by_id(bag_id)
                if bag:
                    bags_to_download.append(bag)
                else:
                    print(f'Warning: No bag with ID {bag_id}')
            except ValueError:
                # Not an integer, treat as filename/pattern
                import fnmatch

                matches = [bag for bag in AVAILABLE_BAGS if fnmatch.fnmatch(
                    bag['filename'], item)]
                if matches:
                    bags_to_download.extend(matches)
                else:
                    print(f"Warning: No files match pattern '{item}'")

        # Remove duplicates while preserving order
        seen = set()
        unique_bags = []
        for bag in bags_to_download:
            if bag['id'] not in seen:
                seen.add(bag['id'])
                unique_bags.append(bag)
        bags_to_download = unique_bags

        if not bags_to_download:
            print('\nNo files to download. Use --list to see available files.')
            return 1
    else:
        # No arguments provided
        print(
            ('No files specified. Use --list to see available files '
             'or --all to download everything.')
        )
        parser.print_help()
        return 1

    # Download files
    print(f'\nDownloading to: {output_dir.absolute()}')
    print(f'Files to download: {len(bags_to_download)}\n')

    success_count = 0
    skip_count = 0
    fail_count = 0

    for bag in bags_to_download:
        filename = bag['filename']
        output_path = output_dir / filename

        # Check if file already exists
        if output_path.exists() and not args.force:
            print(
                f'⊙ {filename} already exists, skipping (use --force to re-download)')
            skip_count += 1
            continue

        # Download
        if download_file(bag['url'], output_path):
            success_count += 1
        else:
            fail_count += 1

    # Summary
    print('\n' + '=' * 80)
    print('Download Summary:')
    print(f'  ✓ Downloaded: {success_count}')
    print(f'  ⊙ Skipped: {skip_count}')
    print(f'  ✗ Failed: {fail_count}')
    print('=' * 80)

    return 0 if fail_count == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
