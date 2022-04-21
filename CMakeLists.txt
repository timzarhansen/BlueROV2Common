cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CATKIN_ENABLE_TESTING OFF)
project(bluerov2common)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roscpp message_generation cv_bridge underwaterslam ping360_sonar commonbluerovmsg)
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Qt5 COMPONENTS Core Gui  Widgets Gamepad REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
#add_message_files(
#    FILES
#    leakageDetection.msg
#    desiredStateForRobot.msg
#    staterobotforevaluation.msg
#   Message1.msg
#   Message2.msg
#)
#add_message_files(DIRECTORY msg FILES Num.msg)


## Generate services in the 'srv' folder
#add_service_files(
#    FILES
#    cameraAngle.srv
#    lightDensity0to10.srv
#    resetekf.srv
#    saveGraph.srv
#)

#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
include
        ${catkin_INCLUDE_DIRS}
        ${QWT_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
)


add_executable(laserScansToPCL src/laserScan2PointCloud.cpp)
target_link_libraries(laserScansToPCL ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(conversionenutofrd src/conversionENUtoFRD.cpp)
target_link_libraries(conversionenutofrd ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(mavrosHeightCalculation src/heightPublisherFromMavros.cpp)
target_link_libraries(mavrosHeightCalculation ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(guibluerov2 src/guiBlueROV2.cpp include/mainwindow.h src/mainwindow.cpp src/rosHandlerGui.cpp include/rosHandlerGui.h include/qcustomplot.h src/qcustomplot.cpp)
target_link_libraries(guibluerov2 Qt5::Core Qt5::Gui Qt5::Widgets Qt5::PrintSupport Qt5::Gamepad Eigen3::Eigen ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(controllerbluerov2 include/controllerOfBluerov2.h src/controllerOfBluerov2.cpp src/controllerRosNode.cpp)
target_link_libraries(controllerbluerov2 ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(justChristmasPresentCalculation src/randomTestFOrWeihnachtsgeschenk.cpp )
target_link_libraries(justChristmasPresentCalculation ${catkin_LIBRARIES} Eigen3::Eigen)