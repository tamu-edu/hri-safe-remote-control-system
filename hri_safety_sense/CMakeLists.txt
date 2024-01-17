# minimum cmake version
cmake_minimum_required(VERSION 3.8)

# project name
project(hri_safety_sense)

# compiler specific settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-pedantic)
endif()

# enable C++ 14
set(CMAKE_CXX_STANDARD 14)

# find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# list srv files
set(srv_files
  "srv/EmergencyStop.srv"
  "srv/KeyString.srv"
  "srv/KeyValue.srv"
   )

# handles all msg and srv
rosidl_generate_interfaces(${PROJECT_NAME}
                           ${srv_files}
                           DEPENDENCIES std_msgs sensor_msgs
)

# add source code to node executable
add_executable(safe_remote_control
               src/main.cpp
               src/VscProcess.cpp
               src/VehicleInterface.c
               src/JoystickHandler.cpp 
               src/SerialInterface.c
)

# add include directories to node executable
target_include_directories(safe_remote_control
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# list target executable dependencies
ament_target_dependencies(safe_remote_control
                          rclcpp
                          std_msgs
                          sensor_msgs                      
)  

# add rosidl target interfaces to node executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(safe_remote_control ${cpp_typesupport_target})

# installs executable targets to build destination
install(TARGETS
        safe_remote_control
        DESTINATION lib/${PROJECT_NAME}
)

# installs launch directory to build destination
# install(DIRECTORY
#         launch
#         DESTINATION share/${PROJECT_NAME}/
# )

# installs package.xml
ament_package()
