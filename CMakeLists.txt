cmake_minimum_required(VERSION 3.5)
project(vrobot_backend CXX)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_DEPENDENCIES
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cartographer
  cartographer_ros_msgs
  cartographer_ros
  std_srvs
  std_msgs        
  vrobot_map_manager
  nav2_util
)
foreach(DEPENDENCY ${THIS_PACKAGE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# find_package(cartographer REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Drogon CONFIG REQUIRED)

include_directories(
        include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)


aux_source_directory(models MODEL_SRC)
aux_source_directory(controllers CONTROLLERS_SRC)
aux_source_directory(serivces SERVICES_SRC)
aux_source_directory(src ROS_SRC)


add_library(${PROJECT_NAME}_lib SHARED
        ${MODEL_SRC}
        ${CONTROLLERS_SRC}
        ${SERVICES_SRC}
        ${ROS_SRC}
        )
target_link_libraries(${PROJECT_NAME}_lib 
        Drogon::Drogon
        ${OpenCV_LIBS}
        ${EIGEN3_LIBRARIES}
        cartographer
        -lpthread
)
ament_target_dependencies(${PROJECT_NAME}_lib
    ${THIS_PACKAGE_DEPENDENCIES}
)

add_executable(${PROJECT_NAME}_node
        main.cc
)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME}_lib
)

install(TARGETS ${PROJECT_NAME}_lib 
     LIBRARY   DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
)




ament_package()