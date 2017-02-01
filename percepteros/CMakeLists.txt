cmake_minimum_required(VERSION 2.8.3)
project(percepteros)
find_package(catkin REQUIRED robosherlock suturo_perception_msgs)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE percepteros)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/percepteros/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1 -pthread")
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
rs_add_library(rs_cylinderAnnotator src/CylinderAnnotator.cpp)
target_link_libraries(rs_cylinderAnnotator ${CATKIN_LIBRARIES})

rs_add_library(rs_rosPublisher src/ROSPublisher.cpp)
target_link_libraries(rs_rosPublisher ${CATKIN_LIBRARIES})

rs_add_library(rs_trayAnnotator src/TrayAnnotator.cpp)
target_link_libraries(rs_trayAnnotator ${CATKIN_LIBRARIES})

rs_add_library(rs_cakeAnnotator src/CakeAnnotator.cpp)
target_link_libraries(rs_cakeAnnotator ${CATKIN_LIBRARIES})

rs_add_executable(caterrosRun src/CaterrosRun.cpp src/CaterrosPipelineManager.cpp src/CaterrosControlledAnalysisEngine.cpp)
target_link_libraries(caterrosRun ${CATKIN_LIBRARIES})
