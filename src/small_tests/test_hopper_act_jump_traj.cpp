#include <iostream>
#include <Utils/utilities.hpp>

#include <optimization/optimization_problems/2d_hopper_act/hopper_act_jump_prob.hpp>
#include <hopper_combined_dynamics_model/hopper_combined_dynamics_model.hpp>

#include <optimization/snopt_wrapper.hpp>


void parse_output(Optimization_Problem_Main* opt_prob){
	Opt_Variable_Manager* var_manager;
	Hopper_Combined_Dynamics_Model* combined_model = Hopper_Combined_Dynamics_Model::GetCombinedModel();

	opt_prob->get_var_manager(var_manager);
	
	sejong::Vector x_states;
	sejong::Vector xdot_states;	

	sejong::Vector q_states;
	sejong::Vector qdot_states;

	sejong::Vector torque_states;	
	sejong::Vector Fr_states;	
	double h_dt = -1.0;

	sejong::Vector pos;

	for(size_t k = 0; k < var_manager->total_knotpoints+1; k++){

	 	std::cout << "--------------------------" << std::endl;
	 	std::cout << "knotpoint = " << k << std::endl;
	 	var_manager->get_x_states(k, x_states); 	
	 	var_manager->get_xdot_states(k, xdot_states); 	

		combined_model->convert_x_to_q(x_states, q_states);


		std::cout << "q[1] = " << combined_model->actuator_model->get_joint_pos_q(0, x_states[1]) << std::endl;


		combined_model->robot_model->getPosition(q_states, SJ_Hopper_LinkID::LK_foot, pos);


	 	sejong::pretty_print(x_states, std::cout, "x_states");
	 	sejong::pretty_print(xdot_states, std::cout, "xdot_states");
	 	sejong::pretty_print(q_states, std::cout, "q_states");
	 	if (k != 0){
		 	var_manager->get_u_states(k, torque_states); 		 	
		 	var_manager->get_var_reaction_forces(k, Fr_states); 		 	
		 	var_manager->get_var_knotpoint_dt(k-1, h_dt);
		 	sejong::pretty_print(torque_states, std::cout, "torque_states");	
		 	sejong::pretty_print(Fr_states, std::cout, "Fr_states");	 		 	
		 	std::cout << "h_dt = " << h_dt << std::endl;
	 	}
	 	sejong::pretty_print(pos, std::cout, "pos");	 	
	}	

}

int main(int argc, char **argv)
{
	std::cout << "[Main] Running Hopper Stand Optimization Problem" << std::endl;
	Optimization_Problem_Main* 	opt_problem = new Hopper_Act_Jump_Opt();

	snopt_wrapper::solve_problem_no_gradients(opt_problem);
	parse_output(opt_problem);
	
	delete opt_problem;
	return 0;
}
