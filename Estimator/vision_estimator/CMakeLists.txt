cmake_minimum_required(VERSION 2.8.3)
project(STAPLE)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -03)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        geometry_msgs
        nav_msgs
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(fhog
        include/fhog.h
        include/sse.hpp
        src/fhog.cpp)
target_link_libraries(fhog
        ${catkin_LIBRARIES}
        )

add_library(staple_tracker
        include/staple_tracker.hpp
        src/staple_tracker.cpp)
target_link_libraries(staple_tracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(tracker
        src/tracker.cpp
        src/fhog.cpp)
target_link_libraries(tracker
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        fhog
        staple_tracker
        )

add_executable(tracker_template
        App/tracker_template.cpp
        src/fhog.cpp)
target_link_libraries(tracker_template
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        fhog
        staple_tracker
        )

add_executable(tracker_encoder
        App/tracker_encoder.cpp
        src/fhog.cpp)
target_link_libraries(tracker_encoder
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        fhog
        staple_tracker
        )

add_executable(tracker_encoder2Video
        App/tracker_encoder2Video.cpp
        src/fhog.cpp)
target_link_libraries(tracker_encoder2Video
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        fhog
        staple_tracker
        )
