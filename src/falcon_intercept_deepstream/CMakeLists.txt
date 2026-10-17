cmake_minimum_required(VERSION 3.18)

if(NOT DEFINED CMAKE_CUDA_COMPILER AND EXISTS "/usr/local/cuda/bin/nvcc")
  set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH
    "Jetson CUDA compiler")
endif()

project(falcon_intercept_deepstream LANGUAGES CXX CUDA)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-base-1.0
  gstreamer-video-1.0
)

set(DEEPSTREAM_ROOT "/opt/nvidia/deepstream/deepstream-7.1" CACHE PATH
  "DeepStream SDK installation root")
set(DEEPSTREAM_INCLUDE_DIR "${DEEPSTREAM_ROOT}/sources/includes")
find_path(CUDA_INCLUDE_DIR
  NAMES cuda_runtime_api.h
  PATHS /usr/local/cuda/include /usr/local/cuda-12.6/include
  REQUIRED
)

find_library(NVDS_GST_META_LIBRARY
  NAMES nvdsgst_meta
  PATHS "${DEEPSTREAM_ROOT}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)
find_library(NVDS_META_LIBRARY
  NAMES nvds_meta
  PATHS "${DEEPSTREAM_ROOT}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)
find_library(NVBUF_SURFACE_LIBRARY
  NAMES nvbufsurface
  PATHS "${DEEPSTREAM_ROOT}/lib" /usr/lib/aarch64-linux-gnu/nvidia
  REQUIRED
)

add_library(falcon_nv12_color_matrix STATIC
  src/falcon_nv12_color_matrix.cu
)
target_include_directories(falcon_nv12_color_matrix PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(falcon_nv12_color_matrix PUBLIC
  CUDA::cudart
  CUDA::cuda_driver
)
set_target_properties(falcon_nv12_color_matrix PROPERTIES
  CUDA_ARCHITECTURES 87
  POSITION_INDEPENDENT_CODE ON
)

add_library(gstfalconcolor SHARED
  src/gst_falcon_color.cpp
)
target_include_directories(gstfalconcolor PRIVATE
  "${DEEPSTREAM_INCLUDE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(gstfalconcolor PRIVATE
  PkgConfig::GSTREAMER
  falcon_nv12_color_matrix
  "${NVBUF_SURFACE_LIBRARY}"
)
set_target_properties(gstfalconcolor PROPERTIES
  OUTPUT_NAME gstfalconcolor
)

add_library(nvdsinfer_custom_yolo26 SHARED
  src/nvdsinfer_custom_yolo26.cpp
)
target_include_directories(nvdsinfer_custom_yolo26 PRIVATE
  "${DEEPSTREAM_INCLUDE_DIR}"
  "${CUDA_INCLUDE_DIR}"
)

add_executable(deepstream_three_camera_node
  src/deepstream_three_camera_node.cpp
)
ament_target_dependencies(deepstream_three_camera_node
  rclcpp
  std_msgs
)
target_include_directories(deepstream_three_camera_node PRIVATE
  "${DEEPSTREAM_INCLUDE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(deepstream_three_camera_node
  PkgConfig::GSTREAMER
  falcon_nv12_color_matrix
  "${NVDS_GST_META_LIBRARY}"
  "${NVDS_META_LIBRARY}"
  "${NVBUF_SURFACE_LIBRARY}"
)

set(DEFAULT_PARSER_LIBRARY
  "${CMAKE_INSTALL_PREFIX}/lib/libnvdsinfer_custom_yolo26.so")
target_compile_definitions(deepstream_three_camera_node PRIVATE
  FALCON_DEFAULT_PARSER_PATH="${DEFAULT_PARSER_LIBRARY}"
)

install(TARGETS
  nvdsinfer_custom_yolo26
  LIBRARY DESTINATION lib
)
install(TARGETS
  gstfalconcolor
  LIBRARY DESTINATION lib/gstreamer-1.0
)
install(TARGETS
  deepstream_three_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  tools/deepstream_three_camera_smoke_test.sh
  tools/deepstream_ros_topic_smoke_test.sh
  tools/deepstream_detection_topic_check.py
  tools/deepstream_roi_runtime_smoke_test.py
  tools/deepstream_roi_end_to_end_smoke.sh
  tools/export_yolo26_onnx.py
  tools/export_yolo26_int8_engine.py
  tools/unwrap_ultralytics_engine.py
  tools/deepstream_single_camera_benchmark.sh
  tools/deepstream_preview.sh
  tools/summarize_deepstream_fps.py
  tools/imx477_wb_sweep.sh
  tools/three_camera_preview.sh
  tools/three_camera_preview_power_safe.sh
  tools/video_detection_stats.py
  tools/deepstream_video_replay.sh
  tools/prepare_video_review_clips.py
  tools/deepstream_file_eval.sh
  tools/compare_precision_profiles.py
  tools/record_three_yolo_annotated.sh
  tools/camera_mapping_contract_check.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
