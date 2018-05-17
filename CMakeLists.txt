cmake_minimum_required(VERSION 2.8.3)
project(dxl_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        roscpp
        dynamixel_sdk)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dxl_interface
  CATKIN_DEPENDS  dynamixel_sdk
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

set (PROJECT_FILES
        src/dxl_interface.cpp
        src/motors_builder.cpp
        src/example_node.cpp

        include/${PROJECT_NAME}/dxl_interface.h
        include/${PROJECT_NAME}/motor.h
        include/${PROJECT_NAME}/model.h
        include/${PROJECT_NAME}/math.h
        include/${PROJECT_NAME}/spec.h
        include/${PROJECT_NAME}/motors_builder.h
        include/${PROJECT_NAME}/protocol.h
        include/${PROJECT_NAME}/exceptions

        )

## Declare a C++ library
 add_library(${PROJECT_NAME}
        ${PROJECT_FILES}
 )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${${PROJECT_NAME}_gencpp} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(${PROJECT_NAME} #_node
   ${catkin_LIBRARIES}
 )

add_executable(example_node
        ${PROJECT_FILES}
        )

## Specify libraries to link a library or executable target against
target_link_libraries(
        example_node
        ${catkin_LIBRARIES}
)

 install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
