project (CameraIntrinsicCalibration)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(libCameraIntrinsicCalibration 
	CameraIntrinsicCalibration.cpp AutoImagePicker.cpp)

add_executable(CamInCalMain CamInCalMain.cpp)
target_link_libraries(CamInCalMain 
	${OpenCV_LIBRARIES} 
	libCameraIntrinsicCalibration)
