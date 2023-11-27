cmake_minimum_required(VERSION 3.5)
project(http_requester_interfaces)

# enable more compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# invoke code and target generation for ROS interface files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MakeHTTPRequest.srv"
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)

# setup linters, if enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# allow ament to install package.xml, etc.
ament_package()
