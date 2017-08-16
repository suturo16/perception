cmake_minimum_required(VERSION 2.8.3)
project(percepteros)
find_package(catkin REQUIRED PCL REQUIRED robosherlock suturo_perception_msgs)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE percepteros)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/percepteros/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_BUILD_TYPE debug)
################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)
################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)
catkin_package()
################################################################################
## Package dependencies                                                       ##
################################################################################
#Add required catkin packages here
#find_package(catkin REQUIRED COMPONENTS xxx)
#Add required system packages here
#find_package(xxx)
################################################################################
## Build                                                                      ##
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        ${RS_INCLUDE_DIRS_LIST}
        ${catkin_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
)
################################################################################
## Update analysis engines, typesystem and include all relevant files         ##
################################################################################
##Update xml list of annotators inside analysis engines
update_analysis_engines(robosherlock)
## generate classes from the typesystem xml files
generate_type_system(robosherlock)
#find all relevant files
find_additional_files()
################################################################################
## Sub Projects                                                               ##
################################################################################
#If you want to divide your projects into subprojects include the subdirectories
#each containing a CMakeLists.txt here
#add_subdirectory(src/xxx)

add_library(percepteros_orf src/RegionDefinition.cpp)
add_dependencies(percepteros_orf yaml-cpp)

rs_add_library(rs_cylinderAnnotator src/CylinderAnnotator.cpp)
target_link_libraries(rs_cylinderAnnotator ${CATKIN_LIBRARIES})

rs_add_library(rs_szeneRecorder src/SzeneRecorder.cpp)
target_link_libraries(rs_szeneRecorder ${CATKIN_LIBRARIES})

rs_add_library(rs_rosPublisher src/ROSPublisher.cpp)
target_link_libraries(rs_rosPublisher ${CATKIN_LIBRARIES})

rs_add_library(rs_trayAnnotator src/TrayAnnotator.cpp)
target_link_libraries(rs_trayAnnotator ${CATKIN_LIBRARIES})

rs_add_library(rs_cakeAnnotator src/CakeAnnotator.cpp)
target_link_libraries(rs_cakeAnnotator ${CATKIN_LIBRARIES})

rs_add_library(rs_kinectFusion src/KinectFusion.cpp)
target_link_libraries(rs_kinectFusion ${CATKIN_LIBRARIES})

rs_add_library(rs_incrementalPointRegistration src/IncrementalPointRegistration.cpp)
target_link_libraries(rs_incrementalPointRegistration ${CATKIN_LIBRARIES})

rs_add_library(rs_knifeAnnotator src/KnifeAnnotator.cpp)
target_link_libraries(rs_knifeAnnotator ${CATKIN_LIBRARIES})

rs_add_library(rs_spatulAnnotator src/SpatulAnnotator.cpp)
target_link_libraries(rs_spatulAnnotator ${CATKIN_LIBRARIES})

rs_add_library(rs_plateAnnotator src/PlateAnnotator.cpp)
target_link_libraries(rs_plateAnnotator ${CATKIN_LIBRARIES})

rs_add_library(rs_boardAnnotator src/BoardAnnotator.cpp)
target_link_libraries(rs_boardAnnotator ${CATKIN_LIBRARIES})

rs_add_library(rs_colorClusterer src/ColorClusterer.cpp)
target_link_libraries(rs_colorClusterer ${CATKIN_LIBRARIES})

rs_add_library(rs_spatulaRecognition src/SpatulaRecognition.cpp)
target_link_libraries(rs_spatulaRecognition ${CATKIN_LIBRARIES} percepteros_orf)

rs_add_library(rs_objectRegionFilter src/ObjectRegionFilter.cpp)
target_link_libraries(rs_objectRegionFilter ${CATKIN_LIBRARIES} yaml-cpp)

rs_add_executable(caterrosRun src/CaterrosRun.cpp src/CaterrosPipelineManager.cpp src/CaterrosControlledAnalysisEngine.cpp)
target_link_libraries(caterrosRun ${CATKIN_LIBRARIES})

