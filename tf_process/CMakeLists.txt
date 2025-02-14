cmake_minimum_required(VERSION 3.0.2)
project(tf_process)
set(OpenCV_DIR “/home/ckyf/opencv/build”)

find_package(
        catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        sensor_msgs
        tf2
        tf2_ros
        cv_bridge
        quadrotor_msgs
)
find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)
catkin_package(
          INCLUDE_DIRS include
        #  LIBRARIES tf_process
        #  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros
        #  DEPENDS system_lib

)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/transform_process.cpp)
add_executable(imu_process_node src/imu_process.cpp src/MahonyAHRS.cpp)

target_link_libraries(
        ${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)

target_link_libraries(
        imu_process_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)