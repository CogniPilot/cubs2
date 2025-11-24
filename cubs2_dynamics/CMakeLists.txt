cmake_minimum_required(VERSION 3.8)
project(cubs2_dynamics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  
  # Python style check (flake8 only - others too strict for initial development)
  ament_flake8()
  
  # Unit tests
  ament_add_pytest_test(${PROJECT_NAME}_pytest test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
