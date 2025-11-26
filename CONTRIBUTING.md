# Contributing to cubs2

Thank you for your interest in contributing to cubs2! This document provides guidelines for contributing to the project.

## Code of Conduct

We are committed to providing a welcoming and inclusive environment for all contributors. Please be respectful and constructive in all interactions.

## How to Contribute

### Reporting Issues

- Check if the issue already exists before creating a new one
- Provide a clear description of the problem
- Include steps to reproduce the issue
- Specify your environment (OS, ROS version, etc.)

### Submitting Changes

1. Fork the repository
2. Create a new branch for your feature or fix
3. Make your changes following the coding standards below
4. Test your changes thoroughly
5. Submit a pull request with a clear description

## Coding Standards

### Python Code

- Follow PEP 8 style guide
- Use `ament_flake8` for linting (all code must pass with default configuration)
- Maximum line length: 99 characters
- Use double quotes for strings (unless escaping is needed)
- Remove unused imports and variables
- Add blank line after class docstrings

### C++ Code

- Follow ROS 2 C++ style guide
- Use `ament_lint_auto` for automated linting
- Include copyright headers on all files

### Copyright Headers

All source files must include the Apache 2.0 license header:

**Python files:**
```python
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
```

**C++ files:**
```cpp
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
```

## Testing

- All changes must pass existing tests
- Add new tests for new functionality
- Run tests using: `colcon test --packages-select <package_name>`
- Verify linting: `ament_flake8` (for Python) or `ament_lint_auto` (configured in packages)

## Building

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>
```

## License

By contributing to this project, you agree that your contributions will be licensed under the Apache License, Version 2.0.
