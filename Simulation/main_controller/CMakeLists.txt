cmake_minimum_required(VERSION 2.8.3)
project(main_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -03)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        geometry_msgs
        nav_msgs
        mavros_msgs
        tf
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


set(THIRD_PARTY_LIBS
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
#	/usr/lib/libserial.so
        )

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
catkin_package(

)

add_library(loadController
        include/loadController/flightController.h
        src/loadController/flightController.cpp)
target_link_libraries(loadController
        ${catkin_LIBRARIES}
        ${THIRD_PARTY_LIBS}
        )
add_library(trajectoryGenerator
        include/trajGenerator/trajectory_generator_waypoint.h
        src/trajGenerator/trajectory_generator_waypoint.cpp)
target_link_libraries(trajectoryGenerator
        ${catkin_LIBRARIES})

add_executable(takeoff
        App/takeoff.cpp
        src/loadController/math.cpp
        include/loadController/common_include.h
        include/loadController/math.h
        )
target_link_libraries(takeoff
        ${THIRD_PARTY_LIBS}
        loadController
        trajectoryGenerator
        )

add_executable(realsenseRemap
        App/realsenseRemap.cpp
        )
target_link_libraries(realsenseRemap
        ${THIRD_PARTY_LIBS}
        )

add_executable(viconRemap
        App/viconRemap.cpp
        )
target_link_libraries(viconRemap
        ${THIRD_PARTY_LIBS}
        )

add_executable(showStatus
        App/showStatus.cpp
        )
target_link_libraries(showStatus
        ${THIRD_PARTY_LIBS}
        )

add_executable(ICRA2021_test_node
        App/test.cpp
        src/loadController/math.cpp
        include/loadController/common_include.h
        include/loadController/math.h
        )
target_link_libraries(ICRA2021_test_node
        ${THIRD_PARTY_LIBS}
        loadController
        trajectoryGenerator
        )

add_executable(reduceError
        App/reduceError.cpp
)
target_link_libraries(reduceError
        ${THIRD_PARTY_LIBS}
        )

add_executable(visualiseBag
        App/visualiseBag.cpp
        )
target_link_libraries(visualiseBag
        ${THIRD_PARTY_LIBS}
        )

add_executable(visualiseBag2txt
        App/visualiseBag2txt.cpp
        )
target_link_libraries(visualiseBag2txt
        ${THIRD_PARTY_LIBS}
        )

add_executable(visualiseBag_Path
        App/visualiseBag_Path.cpp
        )
target_link_libraries(visualiseBag_Path
        ${THIRD_PARTY_LIBS}
        )

add_executable(visualiseBag_Path_part23
        App/visualiseBag_Path_part23.cpp
        )
target_link_libraries(visualiseBag_Path_part23
        ${THIRD_PARTY_LIBS}
        )

add_executable(visualiseBag_Path_part3
        App/visualiseBag_Path_part3.cpp
        )
target_link_libraries(visualiseBag_Path_part3
        ${THIRD_PARTY_LIBS}
        )