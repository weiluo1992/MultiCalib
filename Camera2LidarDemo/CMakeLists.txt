project (Camera2LidarDemo)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(genFakeData GenerateFakeData.cpp)
target_link_libraries(genFakeData ${OpenCV_LIBRARIES})
