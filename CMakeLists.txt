cmake_minimum_required(VERSION 3.00)
#project name  
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
PROJECT(pcl_icp_test)
find_package(JNI REQUIRED)
find_package(PCL 1.8 REQUIRED)
#head file path  
INCLUDE_DIRECTORIES( 
${JNI_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_CXX_FLAGS "-std=c++14")
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS}) 
SET(pcl_icp_ 
"src/com_movlad_pcl_io_PointCloud3dReader.cpp"
"src/com_movlad_pcl_io_PointCloud3dReader.h"
"src/com_movlad_pcl_io_PointCloud3dWriter.cpp"
"src/com_movlad_pcl_io_PointCloud3dWriter.h"
"src/com_movlad_pcl_Normal.cpp"
"src/com_movlad_pcl_Normal.h"
"src/com_movlad_pcl_NormalCloud.cpp"
"src/com_movlad_pcl_NormalCloud.h"
"src/com_movlad_pcl_NormalEstimation.cpp"
"src/com_movlad_pcl_NormalEstimation.h"
"src/com_movlad_pcl_Point2d.cpp"
"src/com_movlad_pcl_Point2d.h"
"src/com_movlad_pcl_Point3d.cpp"
"src/com_movlad_pcl_Point3d.h"
"src/com_movlad_pcl_PointCloud3d.cpp"
"src/com_movlad_pcl_PointCloud3d.h"
"src/com_movlad_pcl_RandomSampleConsensus.cpp"
"src/com_movlad_pcl_RandomSampleConsensus.h"
"src/com_movlad_pcl_SampleConsensus.cpp"
"src/com_movlad_pcl_SampleConsensus.h"
"src/com_movlad_pcl_SampleConsensusModel.cpp"
"src/com_movlad_pcl_SampleConsensusModel.h"
"src/com_movlad_pcl_SampleConsensusModelCone.cpp"
"src/com_movlad_pcl_SampleConsensusModelCone.h"
"src/com_movlad_pcl_SampleConsensusModelCylinder.cpp"
"src/com_movlad_pcl_SampleConsensusModelCylinder.h"
"src/com_movlad_pcl_SampleConsensusModelLine.cpp"
"src/com_movlad_pcl_SampleConsensusModelLine.h"
"src/com_movlad_pcl_SampleConsensusModelPlane.cpp"
"src/com_movlad_pcl_SampleConsensusModelPlane.h"
"src/com_movlad_pcl_SampleConsensusModelSphere.cpp"
"src/com_movlad_pcl_SampleConsensusModelSphere.h"
"src/com_movlad_pcl_SampleConsensusModelStick.cpp"
"src/com_movlad_pcl_SampleConsensusModelStick.h"
"src/com_movlad_pcl_visualization_Visualizer.cpp"
"src/com_movlad_pcl_visualization_Visualizer.h"
"src/com_movlad_pcl_visualization_Visualizer3d.cpp"
"src/com_movlad_pcl_visualization_Visualizer3d.h"
"src/pcl_icp.h"
"src/pcl_icp.cpp"
"src/pcl_pointcloudxyz.cpp"
"src/pcl_pointxyz.cpp"
)

#SET(CMAKE_BUILD_TYPE "Debug")  
#SET(CMAKE_BUILD_TYPE "Release") 
remove_definitions(-DBOOST_NO_EXCEPTIONS) 

ADD_LIBRARY(pcl_icp_test SHARED ${pcl_icp_test})
TARGET_LINK_LIBRARIES( ${PCL_LIBRARIES})
