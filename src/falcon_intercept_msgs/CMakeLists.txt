cmake_minimum_required(VERSION 3.8)
project(falcon_intercept_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GuidanceCommand.msg"
  "msg/TargetCameraPosition.msg"
  "msg/CameraBearingObservation.msg"
  "msg/AsyncStereoMeasurement.msg"
  "msg/TargetTrack.msg"
  "msg/SearchCommand.msg"
  "msg/MissionStatus.msg"
  "msg/CommandProposal.msg"
  "msg/SafetyStatus.msg"
  "msg/CaptureWindowStatus.msg"
  "msg/VehicleInterfaceStatus.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
