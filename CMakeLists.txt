cmake_minimum_required(VERSION 3.14)
project(vesc_interface)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)
add_compile_options(-Wall -Wextra -pedantic -Werror)

set(VESC_INTERFACE_LIB_SRC
  src/vesc_interface.cpp
)

set(VESC_INTERFACE_LIB_HEADERS
  include/vesc_interface/vesc_interface.hpp
  include/vesc_interface/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${VESC_INTERFACE_LIB_SRC}
  ${VESC_INTERFACE_LIB_HEADERS}
)

set(VESC_INTERFACE_NODE_SRC
  src/vesc_interface_node.cpp
)

set(VESC_INTERFACE_NODE_HEADERS
  include/vesc_interface/vesc_interface_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${VESC_INTERFACE_NODE_SRC}
  ${VESC_INTERFACE_NODE_HEADERS}
)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "vesc_interface::VescInterfaceNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_vesc_interface.cpp)
  set(TEST_VESC_INTERFACE_EXE test_vesc_interface)
  ament_add_gtest(${TEST_VESC_INTERFACE_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_VESC_INTERFACE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
