cmake_minimum_required(VERSION 3.8)
project(g1_onboard_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Interface files. Order does not matter — rosidl resolves inter-message references
# automatically.
#
# Design notes:
# - 2026-05-15: ValidatedCmd was removed in favour of two separate topics —
#   geometry_msgs/Twist for validated walking commands and JointCmd for
#   validated upper-body joint commands. The G1 SDK runs LocoClient (lower-body
#   walking) and rt/arm_sdk (upper-body joints) as independent controllers,
#   so emitting them on independent topics matches the hardware semantics.
# - 2026-05-16: ActionChunk was removed. Producers publish step-by-step
#   JointCmd; chunk metadata (chunk_id, step_index) travels with each step.
#   See JointCmd.msg "Wire format" section for the rationale.
set(msg_files
  "msg/AudioPCM.msg"
  "msg/JointCmd.msg"
  "msg/LocoCommand.msg"
  "msg/NavState.msg"
  "msg/BufState.msg"
  "msg/SpeakerState.msg"
  "msg/EstopFlag.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
