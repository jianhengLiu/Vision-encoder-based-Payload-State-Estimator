# 设置CMake最低版本
cmake_minimum_required(VERSION 2.8)
# 设置项目名
project(load_estimator)
# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

# 链接头文件
include_directories(${PROJECT_SOURCE_DIR}/include)

set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
link_directories(${LIB_DIR})
MESSAGE( STATUS "LIB_PATH = ${LIB_DIR}.")
link_libraries(USB2XXX)
link_libraries(usb-1.0)

set(serial_DIR /home/nrsl/Apps/serial/serial/build/devel/share/serial/cmake)
find_package(serial)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		std_msgs
		pcl_ros
		cv_bridge
		)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(
		#  INCLUDE_DIRS include
		#  LIBRARIES obeject_detect
		#  CATKIN_DEPENDS roscpp rpspy std_msgs
		#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
		include ${catkin_INCLUDE_DIRS}
		${catkin_INCLUDE_DIRS}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
	/tmp/usr/local/include

)


set(THIRD_PARTY_LIBS
		${serial_LIBRARIES}
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
		${serial_LIBRARIES}
# 		/tmp/usr/local/lib/libserial.so
        )


add_executable(findPossible_D435
		App/findPossible_D435.cpp)
add_dependencies(findPossible_D435 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(findPossible_D435
		${THIRD_PARTY_LIBS}
		)

#add_executable(encoder App/encoder2Pose.cpp)
#add_dependencies(encoder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(encoder
#        ${THIRD_PARTY_LIBS}
#        )


add_executable(ukf App/main.cpp src/ukf.cpp)
add_dependencies(ukf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ukf
		${THIRD_PARTY_LIBS}
		)

#add_executable(kf App/kf.cpp src/kf.cpp)
#add_dependencies(kf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(kf
#        ${THIRD_PARTY_LIBS}
#        ${serial_LIBRARIES}
#        )

add_executable(ekf App/main_EKF.cpp src/ekf.cpp)
add_dependencies(ekf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf
		${THIRD_PARTY_LIBS}
		)

add_executable(gf App/gaussianFusion.cpp src/ekf.cpp)
add_dependencies(gf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gf
		${THIRD_PARTY_LIBS}
		)
