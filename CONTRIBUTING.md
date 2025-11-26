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

This project follows standard ROS 2 coding practices with automated linting.

### Python Code

- Follow PEP 8 style guide (enforced by `ament_flake8`)
- Use NumPy-style docstrings (enforced by `ament_pep257`)
- Maximum line length: 99 characters
- All code must pass `ament_flake8` and `ament_pep257` with default configuration

### C++ Code

- Follow ROS 2 C++ style guide (enforced by `ament_cpplint` and `ament_uncrustify`)
- Use consistent formatting and naming conventions
- Include copyright headers on all files

### Linting

All linting is configured in each package's `package.xml` and automatically runs via `colcon test`:

- `ament_copyright` - Validates license headers
- `ament_flake8` - Python style checking
- `ament_pep257` - Python docstring validation
- `ament_cpplint` - C++ style checking
- `ament_uncrustify` - C++ code formatting
- `ament_lint_cmake` - CMake linting
- `ament_xmllint` - XML validation

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

- All changes must pass existing tests and linting
- Add new tests for new functionality
- Run tests using: `colcon test --packages-select <package_name>`
- View test results: `colcon test-result --verbose`

Linting is automatically included in `colcon test` for all packages. All linters must pass:
- Python: `ament_flake8`, `ament_pep257`, `ament_copyright`
- C++: `ament_cpplint`, `ament_uncrustify`, `ament_copyright`
- Build files: `ament_lint_cmake`, `ament_xmllint`

## Building

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>
```

## License

By contributing to this project, you agree that your contributions will be licensed under the Apache License, Version 2.0.
