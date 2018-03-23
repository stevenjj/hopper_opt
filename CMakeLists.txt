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
include_directories(src/hopper_dynamic_model)

set(hopper_model_sources src/hopper_dynamic_model/Hopper_Definition.h
						 src/hopper_dynamic_model/HopperModel.hpp
						 src/hopper_dynamic_model/HopperModel.cpp)			


set(container_sources src/optimization/containers/opt_variable.cpp
						  src/optimization/containers/opt_variable_manager.cpp
						  src/optimization/containers/constraint_list.cpp
						  src/optimization/containers/contact_list.cpp)

set(hopper_opt_problem_source src/optimization/optimization_problems/hopper_jump_opt_problem.cpp)

set(hopper_constraints src/optimization/hard_constraints/2d_hopper/hopper_dynamics_constraint.cpp
					   src/optimization/hard_constraints/2d_hopper/hopper_time_integration_constraint.cpp)

set(hopper_objective_func_sources src/optimization/objective_functions/2d_hopper/hopper_min_torque_objective_func.cpp)


set(snopt_wrapper_sources src/optimization/snopt_wrapper.cpp)


#--------------------------------------------
# Test Hopper Model
#--------------------------------------------
add_executable(test_hopper_model  src/small_tests/test_hopper_model.cpp ${hopper_model_sources})
target_link_libraries(test_hopper_model  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})								 
#--------------------------------------------

#--------------------------------------------
# Test Variable Containers Model 
#--------------------------------------------
add_executable(test_containers  src/small_tests/test_containers.cpp  ${container_sources})
target_link_libraries(test_containers  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})					 
#--------------------------------------------

#--------------------------------------------
# Test Hopper Optimization Object
#--------------------------------------------
add_executable(test_hopper_opt_obj  src/small_tests/test_hopper_prob_obj.cpp ${hopper_model_sources}
																		 ${container_sources}
																		 ${hopper_opt_problem_source}
																		 ${hopper_constraints}
																		 ${hopper_objective_func_sources}
)
target_link_libraries(test_hopper_opt_obj  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})								 

#--------------------------------------------
# Test Hopper Jump Optimization
#--------------------------------------------
add_executable(test_hopper_stand_traj  src/small_tests/test_hopper_stand_traj.cpp ${hopper_model_sources}
										 									  ${container_sources}
																		      ${hopper_opt_problem_source}
																		      ${hopper_constraints}
																		      ${hopper_objective_func_sources}
																		      ${snopt_wrapper_sources}
)
target_link_libraries(test_hopper_stand_traj  ${SJUtils} ${SJurdf} ${SJrbdl} ${SJsnopt})								 



		 