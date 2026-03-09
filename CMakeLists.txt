cmake_minimum_required(VERSION 3.8)
project(cpu_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 定义消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/X86CpuTemperature.msg"
)

# 创建可执行文件
add_executable(cpu_monitor_node src/cpu_monitor_node.cpp)

# 链接依赖项
ament_target_dependencies(cpu_monitor_node rclcpp)

# 链接生成的接口
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(cpu_monitor_node "${cpp_typesupport_target}")

# 安装可执行文件
install(TARGETS cpu_monitor_node
  DESTINATION lib/${PROJECT_NAME})

# 安装消息文件
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg)

ament_package()
