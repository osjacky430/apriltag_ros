add_executable(apriltag_ros_profiling profiling.cpp)
target_link_libraries(apriltag_ros_profiling PRIVATE ${PROJECT_NAME}_common project_options)

option(ENABLE_PROFILING "Enable profiling using pprof" OFF)
if (ENABLE_PROFILING)
  catkin_add_profiler_report(
    EXE_TARGET apriltag_ros_profiling EXE_COMMAND roslaunch apriltag_ros profiling.launch bag_file:=${BAG_FILE_DIR}
                                                  cam_name:=${CAM_NAME} OUTPUT_FORMAT "pdf"
    PROF_FILE_PATH "ros_continuous_node_profile.prof" ADD_PROF_DIR_MACRO CPUPROFILE_FREQUENCY 250)
endif ()
