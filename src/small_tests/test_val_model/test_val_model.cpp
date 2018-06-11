#include <Utils/utilities.hpp>

#include "ValkyrieRobotModel.hpp"
#include "valkyrie_definition.h"

int main(int argc, char **argv){

	std::cout << "[Main]Testing Valkyrie Model" << std::endl;
	std::cout << "[Main] Constructing model ..." << std::endl;
	ValkyrieRobotModel*	robot_model;
	robot_model = ValkyrieRobotModel::GetValkyrieRobotModel();	

}