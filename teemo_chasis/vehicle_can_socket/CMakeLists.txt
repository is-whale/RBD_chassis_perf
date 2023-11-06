cmake_minimum_required(VERSION 3.0.2)
project(vehicle_can_socket)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    autoware_msgs
    can_msgs
    roscpp
    diagnostic_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES byte
    CATKIN_DEPENDS autoware_msgs    
    DEPENDS 
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")


include_directories(
 include
 include/vehicle_can_socket
    ${catkin_INCLUDE_DIRS}
)

add_library(byte src/byte.cc)
add_dependencies(byte
                ${catkin_EXPORTED_TARGETS}
                )
target_link_libraries(byte ${catkin_LIBRARIES})



add_executable(vehicle_sender
         src/vehicle_sender.cpp
)

add_dependencies(vehicle_sender ${catkin_EXPORTED_TARGETS})
 target_link_libraries(vehicle_sender
     ${catkin_LIBRARIES}
 )

 target_link_libraries(vehicle_sender
     ${catkin_LIBRARIES}
 )

add_executable(vehicle_receiver
        src/vehicle_receiver.cpp
)

add_dependencies(vehicle_receiver ${catkin_EXPORTED_TARGETS})
 target_link_libraries(vehicle_receiver
     ${catkin_LIBRARIES}
 )

install(
    TARGETS
        byte
        vehicle_receiver
        vehicle_sender
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
        include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
