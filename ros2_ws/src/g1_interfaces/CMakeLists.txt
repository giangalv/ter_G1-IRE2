cmake_minimum_required(VERSION 3.8)
project(g1_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionChunk.msg"
  "msg/ArmCmd.msg"
  "msg/ContactState.msg"
  "msg/FailureState.msg"
  "msg/HandCmd.msg"
  "msg/MemoryEntry.msg"
  "msg/ObjectPose.msg"
  "msg/RiskScore.msg"
  "msg/SubtaskToken.msg"
  "msg/TactileArray.msg"
  "msg/TactileEmbedding.msg"
  "srv/GetObjectPose.srv"
  "srv/QueryMemory.srv"
  "srv/TriggerRecovery.srv"
  "action/ExecuteTask.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
