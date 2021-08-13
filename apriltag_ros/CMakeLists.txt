cmake_minimum_required(VERSION 3.5.1)
project(apriltag_ros)

if (POLICY CMP0069) # for IPO
  cmake_policy(SET CMP0069 NEW)
endif ()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(StandardProjectSetting)

find_package(
  catkin REQUIRED
  COMPONENTS cmake_modules
             cv_bridge
             geometry_msgs
             image_geometry
             image_transport
             message_generation
             nodelet
             pluginlib
             roscpp
             sensor_msgs
             std_msgs
             tf)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
add_library(apriltag_pkg INTERFACE)
target_link_libraries(apriltag_pkg INTERFACE ${apriltag_STATIC_LDFLAGS})
target_include_directories(apriltag_pkg INTERFACE ${apriltag_INCLUDE_DIRS}/apriltag)
target_compile_options(apriltag_pkg INTERFACE -fno-math-errno -funsafe-math-optimizations -ffinite-math-only
                                              -fno-signed-zeros -funroll-loops)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# add_compile_options(
#   "-funsafe-loop-optimizations"       # only gcc-5

# Note: These options have been turned off to allow for binary releases -
# in local builds, they can be reactivated to achieve higher performance.
# if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64" OR "x86_32")
#   message("enabling msse2 for x86_64 or x86_32 architecture")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -msse2 ")
# endif()
# if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "arm")
#   message("enabling -mfpu=neon -mfloat-abi=softfp for ARM architecture")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv7-a -mcpu=cortex-a9 -mfpu=neon -mtune=cortex-a9 -mvectorize-with-neon-quad -ffast-math ")
# endif()

add_message_files(FILES AprilTagDetection.msg AprilTagDetectionArray.msg)

add_service_files(FILES AnalyzeSingleImage.srv)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  cv_bridge
  geometry_msgs
  image_transport
  message_runtime
  nodelet
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf
  DEPENDS
  OpenCV
  apriltag
  LIBRARIES
  ${PROJECT_NAME}_common
  ${PROJECT_NAME}_continuous_detector
  ${PROJECT_NAME}_single_image_detector)

###########
## Build ##
###########

include(StaticAnalyzers)
include(Sanitizers)
add_library(project_options INTERFACE)
enable_sanitizers(project_options)

include(CompilerWarning)
add_library(project_warnings INTERFACE)
set_project_warnings(project_warnings)

add_library(catkin_pkg INTERFACE)
target_include_directories(catkin_pkg SYSTEM INTERFACE ${catkin_INCLUDE_DIRS})
target_link_libraries(catkin_pkg INTERFACE ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_common src/common_functions.cpp)
add_dependencies(${PROJECT_NAME}_common ${PROJECT_NAME}_generate_messages_cpp)

target_include_directories(${PROJECT_NAME}_common PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(${PROJECT_NAME}_common SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_common PUBLIC catkin_pkg ${OpenCV_LIBRARIES} apriltag_pkg PRIVATE project_options)

add_library(${PROJECT_NAME}_continuous_detector src/continuous_detector.cpp)
target_link_libraries(${PROJECT_NAME}_continuous_detector PUBLIC ${PROJECT_NAME}_common)

add_library(${PROJECT_NAME}_single_image_detector src/single_image_detector.cpp)
target_link_libraries(${PROJECT_NAME}_single_image_detector PUBLIC ${PROJECT_NAME}_common)

add_executable(${PROJECT_NAME}_continuous_node src/${PROJECT_NAME}_continuous_node.cpp)
add_dependencies(${PROJECT_NAME}_continuous_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_continuous_node PRIVATE ${PROJECT_NAME}_continuous_detector project_options)

add_executable(${PROJECT_NAME}_single_image_server_node src/${PROJECT_NAME}_single_image_server_node.cpp)
add_dependencies(${PROJECT_NAME}_single_image_server_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_single_image_server_node PRIVATE ${PROJECT_NAME}_single_image_detector catkin_pkg)

add_executable(${PROJECT_NAME}_single_image_client_node src/${PROJECT_NAME}_single_image_client_node.cpp)
add_dependencies(${PROJECT_NAME}_single_image_client_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_single_image_client_node PRIVATE ${PROJECT_NAME}_common)

include(CatkinExtension)
add_subdirectory(profiling)

if (ENABLE_COVERAGE)
  add_custom_target(use_test_set COMMAND roslaunch apriltag_ros profiling.launch bag_file:=${BAG_FILE_DIR}
                                         cam_name:=${CAM_NAME})
  catkin_add_gcov_report(TARGET use_test_set OUTPUT_DIR "coverage.xml"
                         EXTRA_OPTIONS --xml-pretty --exclude-unreachable-branches --exclude-throw-branches)
endif ()

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING
        PATTERN "*.h")

install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}_common ${PROJECT_NAME}_continuous_detector ${PROJECT_NAME}_continuous_node
                ${PROJECT_NAME}_single_image_client_node ${PROJECT_NAME}_single_image_detector
                ${PROJECT_NAME}_single_image_server_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/analyze_image DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
