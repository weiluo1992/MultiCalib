project (JointCalibration)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})


add_executable(JointMain JointMain.cpp)
add_executable(testPCL TestPCL.cpp)
target_link_libraries(testPCL ${PCL_LIBRARIES})
