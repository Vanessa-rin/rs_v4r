cmake_minimum_required(VERSION 2.8.3)
project(rs_v4r)
find_package(catkin REQUIRED robosherlock suturo_perception_msgs geometry_msgs)
################################################################################
## Constants for project                                                      ##
################################################################################
set(NAMESPACE rs_v4r)
set(TYPESYSTEM_CPP_PATH ${PROJECT_SOURCE_DIR}/include/rs_v4r/types)
set(TYPESYSTEM_XML_PATH ${PROJECT_SOURCE_DIR}/descriptors/typesystem)
set(ANNOTATOR_PATH      ${PROJECT_SOURCE_DIR}/descriptors/annotators)
set(ENGINE_PATH         ${PROJECT_SOURCE_DIR}/descriptors/analysis_engines)
set(CMAKE_MODULE_PATH   ${PROJECT_SOURCE_DIR}/cmake)
################################################################################
## Configure project configuration files                                      ##
################################################################################
configure_file(${RS_PROJECT_CONFIG} ${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.cmake @ONLY)
################################################################################
## Find all include directories                                               ##
################################################################################
find_include_dirs(RS_INCLUDE_DIRS_LIST)
catkin_package(
        INCLUDE_DIRS include ${RS_INCLUDE_DIRS_LIST} ${catkin_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
        CFG_EXTRAS ${PROJECT_NAME}_config.cmake
        CATKIN_DEPENDS suturo_perception_msgs geometry_msgs
)
################################################################################
## Package dependencies                                                       ##
################################################################################
#Add required catkin packages here
find_package(catkin REQUIRED robosherlock suturo_perception_msgs geometry_msgs actionlib_msgs image_transport)
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
add_library(${PROJECT_NAME} SHARED src/v4rProcessManager.cpp src/suturo_conversion.cpp include/rs_v4r/suturo_conversion.h)
target_link_libraries(${PROJECT_NAME} ${CATKIN_LIBRARIES})
rs_add_library(rs_v4rRecognitionAnnotator src/v4rRecognitionAnnotator.cpp)
