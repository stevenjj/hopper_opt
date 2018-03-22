cmake_minimum_required(VERSION 2.8.3)
project(hopper_opt)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
tf
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lcp_val_simulator
   CATKIN_DEPENDS 
       sensor_msgs
       tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
# After installing the sejong library via sudo make install, we have to find the installed libraries and all the included files

set(Sejong_Library_Path /usr/local/lib/SDC_Toolkit)
set(Sejong_Include_Path /usr/local/include/SDC_Toolkit/)
set(Sejong_Eigen_Path ~/Sejong_Dynamic_Control_Toolkit/ExternalSource)

find_library(SJUtils NAMES libSJutils.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJurdf NAMES libSJurdf.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJrbdl NAMES libSJrbdl.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJMobyLCP NAMES libSJMobyLCP.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJsnopt NAMES libSJsnopt.so PATHS ${Sejong_Library_Path}  REQUIRED)

include_directories(${Sejong_Include_Path})
include_directories(${Sejong_Eigen_Path})

include_directories(include)
include_directories(src/hdt/hopper_dynamic_model)

set(hopper_model_sources src/hdt/hopper_dynamic_model/Hopper_Definition.h
						 src/hdt/hopper_dynamic_model/HopperModel.hpp
						 src/hdt/hopper_dynamic_model/HopperModel.cpp)			


set(hdt_container_sources src/hdt/containers/opt_variable.cpp
						  src/hdt/containers/opt_variable_manager.cpp)
#						  src/hdt/containers/adt_constraint_list.cpp
#						  src/hdt/containers/adt_contact_list.cpp)

#set(adt_snopt_wrapper_sources src/adt/adt_snopt_wrapper.cpp)


#--------------------------------------------
# Test Hopper Model
#--------------------------------------------
add_executable(test_hopper_model  src/small_tests/test_hopper_model.cpp ${hopper_model_sources})
target_link_libraries(test_hopper_model  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})								 
#--------------------------------------------

#--------------------------------------------
# Test Hopper Optimization Object
#--------------------------------------------
#add_executable(test_hopper_opt  src/small_tests/test_hopper_problem.cpp ${hopper_model_sources}
#																		${adt_container_sources}
#																		${adt_optimization_problem_sources}
#																		${adt_objective_function_sources}	
#																		${adt_snopt_wrapper_sources}	
#)
#target_link_libraries(test_hopper_opt  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})								 


#--------------------------------------------
# Test Variable Containers Model 
#--------------------------------------------
add_executable(test_containers  src/small_tests/test_containers.cpp  ${hdt_container_sources})
target_link_libraries(test_containers  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})					 
#--------------------------------------------




		 
