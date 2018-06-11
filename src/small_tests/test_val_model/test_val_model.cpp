#include <Utils/utilities.hpp>

#include "RobotModel.hpp"
#include "valkyrie_definition.h"

int main(int argc, char **argv){

	std::cout << "[Main]Testing Valkyrie Model" << std::endl;
	RobotModel*	robot_model;

	robot_model = RobotModel::GetRobotModel();	

}