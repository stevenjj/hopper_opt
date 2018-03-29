#include <Utils/utilities.hpp>
#include "DracoModel.hpp"
#include "DracoP1Rot_Definition.h"

int main(int argc, char **argv){

	std::cout << "[Main]Testing Draco Model Object" << std::endl;
	DracoModel* robot_model;
	robot_model = DracoModel::GetDracoModel();

	sejong::Vector q_state;
	sejong::Vector qdot_state;

	q_state.resize(NUM_QDOT);
	qdot_state.resize(NUM_QDOT);
	q_state.setZero();
	qdot_state.setZero();

	q_state[0] = 0.01;
	q_state[1] = 0.87 - 0.19;
	q_state[SJJointID::bodyPitch] = -1.0;
	q_state[SJJointID::kneePitch] = 2.0;
	q_state[SJJointID::anklePitch] = -1.0;

	// q_state[1] = 0.831165;
	// q_state[SJJointID::bodyPitch] = -M_PI/4.0;
	// q_state[SJJointID::kneePitch] = M_PI/2.0;
	// q_state[SJJointID::anklePitch] = -M_PI/4.0;	


	std::cout << "[Main] Updating Robot Model" << std::endl;
	robot_model->UpdateModel(q_state, qdot_state);
	std::cout << "[Main] Update finished" << std::endl;


}