cmake_minimum_required(VERSION 3.0.2)
project(pcl2process)

find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        cv_bridge
        pcl_ros
        tf2
        tf2_ros
        pcl_conversions
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES project2plane
        #  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pcl2process.cpp)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)
