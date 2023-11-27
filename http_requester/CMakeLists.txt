cmake_minimum_required(VERSION 3.8)
project(http_requester)

# enable more compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(http_requester_interfaces REQUIRED)
find_package(libcurl_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# build library with node component
add_library(http_requester_component SHARED src/http_requester.cpp)
target_include_directories(http_requester_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(http_requester_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(http_requester_component
  ${http_requester_interfaces_TARGETS}
  CURL::libcurl
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
)

# register node component and generate a stand-alone executable
rclcpp_components_register_node(http_requester_component
  PLUGIN "http_requester::HTTPRequesterNode"
  EXECUTABLE "http_requester"
)

# install targets
install(TARGETS
  http_requester_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# setup linters and tests, if enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# allow ament to install package.xml, etc.
ament_package()
