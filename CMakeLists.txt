cmake_minimum_required(VERSION 2.8)
project( uavgp_vision )

##set build mode;
set(CMAKE_BUILD_TYPE RELEASE)

## Find catkin macros and libraries，添加catkin的包
find_package(catkin REQUIRED COMPONENTS
  #message_generation
  #rospy
  #roscpp
  std_msgs
  sensor_msgs
  geometry_msgs

  cv_bridge
  image_transport
)


# include_directories(include)，添加catkin的包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)

##OpenCV
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set include files;
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${PROJECT_BINARY_DIR}
)

#set include files: *.h, *.hpp, *.hh;
#FILE(GLOB_RECURSE PROJECT_HEADERS )

# set OCR and output txt directory path
set(OCR_DIR_PATH  ${PROJECT_SOURCE_DIR})
set(TXT_FILE_PATH ${PROJECT_SOURCE_DIR})

configure_file(
	${PROJECT_SOURCE_DIR}/cmake_config/project_path_config.h.in
	${PROJECT_BINARY_DIR}/project_path_config.h
)
#--------------------------------------------------------------------
# 标识物识别
add_executable( vision_result_publisher
./src/MarkDetection/main.cpp
./src/MarkDetection/Publish.cpp
./src/MarkDetection/Functions.cpp
./src/MarkDetection/KNN_OCR.cpp
./src/MarkDetection/preprocessing.cpp
./src/MarkDetection/ReadOfflineData.cpp
./src/MarkDetection/JudgeRectKind.cpp
./src/MarkDetection/RectDectByStatisticsError.cpp
./src/MarkDetection/SubscribeAndSave.cpp
./src/MarkDetection/CameraCoordinate2NegCoordinate.cpp
./src/PointGrayCameraCapture/OpenCamera.cpp
./include/declare.h
./include/define.h
./include/KNN_OCR.h
./include/preprocessing.h
./include/DisplayScreenProcess.h
./src/MarkDetection/DisplayScreenProcess.cpp
#./src/MarkDetection/MultiThreadListener.h
)
#OpenCV库
target_link_libraries(vision_result_publisher  ${OpenCV_LIBRARIES} )
#添加多线程第三方库
target_link_libraries(vision_result_publisher -lpthread)
#catkin需要的库
target_link_libraries(vision_result_publisher ${catkin_LIBRARIES})


#-------------------------------------------------------------------
# 相机采集
add_executable( camera_publisher
./src/PointGrayCameraCapture/AutoSetExposureTime.cpp
./src/PointGrayCameraCapture/FlyCapture2.cpp
./include/FlyCapture2.h
./src/PointGrayCameraCapture/DetectRectToGetImageLightness.cpp
./src/PointGrayCameraCapture/DetectRectToGetImageLightness.h
)
#添加point gray相机动态链接库
target_link_libraries(camera_publisher  flycapture)
#OpenCV库
target_link_libraries( camera_publisher  ${OpenCV_LIBS} )
#添加多线程第三方库
target_link_libraries(camera_publisher -lpthread)
#catkin需要的库
target_link_libraries(camera_publisher ${catkin_LIBRARIES})


#--------------------------------------------------------------------
# DJI guidance capture
link_directories(${PROJECT_SOURCE_DIR}/lib/x64)
link_directories(${PROJECT_SOURCE_DIR}/lib/XU3)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(guidance_capture
${PROJECT_SOURCE_DIR}/src/GuidanceCapture/GuidanceNode.cpp
${PROJECT_SOURCE_DIR}/src/GuidanceCapture/DJI_utility.cpp
)
target_link_libraries(guidance_capture DJI_guidance usb-1.0 ${catkin_LIBRARIES} ${OpenCV_LIBS})


#-------------------------------------------------------------------
# guidance  数据处理
add_executable( guidance_data_process
./src/StereoProcess/StereoApperceive.cpp
./src/StereoProcess/PointCloudProcessing.cpp
./src/StereoProcess/StereoMatching.cpp
./src/StereoProcess/GetROI_AverageVal.cpp
./src/StereoProcess/ImageSegmentation.cpp
./src/StereoProcess/main.cpp
./include/declare.h
./include/StereoMatching.h
)
#OpenCV库
target_link_libraries( guidance_data_process  ${OpenCV_LIBS} )
#添加多线程第三方库
target_link_libraries(guidance_data_process -lpthread)
#catkin需要的库
target_link_libraries(guidance_data_process ${catkin_LIBRARIES})
#PCL库
target_link_libraries(guidance_data_process ${PCL_LIBRARIES})
#----------------------------------------------------------------------
