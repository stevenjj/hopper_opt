#include <Utils/utilities.hpp>
#include <iostream>
#include <hopper_combined_dynamics_model/hopper_combined_dynamics_model.hpp>

int main(int argc, char **argv){
	std::cout << "[Main] Testing Hopper Combined model" << std::endl;

	sejong::Vector q_state = sejong::Vector::Zero(NUM_Q);
	sejong::Vector qdot_state = sejong::Vector::Zero(NUM_QDOT);
	q_state[0] = 0.5;
	q_state[1] = -0.5;

	Hopper_Combined_Dynamics_Model* hopper_combined_model = Hopper_Combined_Dynamics_Model::GetCombinedModel();

	sejong::Vector q_virt = q_state.head(NUM_VIRTUAL);
	sejong::Vector z_state;
	sejong::Vector delta_state; delta_state.resize(NUM_ACT_JOINT); delta_state.setZero();	

	sejong::Vector qdot_virt = qdot_state.head(NUM_VIRTUAL);
	sejong::Vector zdot_state; zdot_state.resize(NUM_ACT_JOINT); zdot_state.setZero();
	sejong::Vector delta_dot_state;	delta_dot_state.resize(NUM_ACT_JOINT); delta_dot_state.setZero();	

    hopper_combined_model->actuator_model->getFull_act_pos_z(q_state.tail(NUM_ACT_JOINT), z_state);
    sejong::pretty_print(z_state, std::cout, "z_state");

    sejong::Vector x_state; x_state.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);
    sejong::Vector xdot_state; xdot_state.resize(NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT);


    x_state.head(NUM_VIRTUAL) = q_virt;
    x_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = z_state;
	x_state.tail(NUM_ACT_JOINT) = delta_state;

	xdot_state.head(NUM_VIRTUAL) = qdot_virt;
    xdot_state.segment(NUM_VIRTUAL, NUM_ACT_JOINT) = zdot_state;
	xdot_state.tail(NUM_ACT_JOINT) = delta_dot_state;	    


}