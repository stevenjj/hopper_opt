#include <Utils/utilities.hpp>
#include <hopper_actuator_model/hopper_actuator_model.hpp>
#include <iostream>


int main(int argc, char **argv){
	std::cout << "[Main] Testing Hopper Actuator model" << std::endl;
	HopperActuatorModel* actuator_model;
	actuator_model = HopperActuatorModel::GetActuatorModel();

	sejong::Matrix M_act;
	sejong::Matrix B_act;
	sejong::Matrix K_act;	
	sejong::Matrix L_act;
	actuator_model->getMassMatrix(M_act);
	actuator_model->getDampingMatrix(B_act);	
	actuator_model->getStiffnessMatrix(K_act);
	sejong::Vector z_act; z_act.resize(NUM_ACT_JOINT); z_act.setZero();
	actuator_model->getFullJacobian_dzdq(z_act, L_act);

	sejong::pretty_print(M_act, std::cout, "M_act");		
	sejong::pretty_print(B_act, std::cout, "B_act");		
	sejong::pretty_print(K_act, std::cout, "K_act");		
	sejong::pretty_print(L_act, std::cout, "L_act");		
}