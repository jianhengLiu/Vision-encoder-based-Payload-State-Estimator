cmake_minimum_required(VERSION 3.0.2)
project(img_extract)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        )
find_package(OpenCV REQUIRED)
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES img_extract
        #  CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
add_executable(img_extract
        src/img_extract.cpp)
target_link_libraries(img_extract
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(img_catcher
        src/img_catcher.cpp)
target_link_libraries(img_catcher
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(fisheye_calibration
        src/fisheye_calibration.cpp)
target_link_libraries(fisheye_calibration
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(fisheye_undistort
        src/fisheye_undistort.cpp)
target_link_libraries(fisheye_undistort
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(fisheye_undistort_node
        src/fisheye_undistort_node.cpp)
target_link_libraries(fisheye_undistort_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(fisheye_undistort_node_sub
        src/fisheye_undistort_node_sub.cpp)
target_link_libraries(fisheye_undistort_node_sub
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )