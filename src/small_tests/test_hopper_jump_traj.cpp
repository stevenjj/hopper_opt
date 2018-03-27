#include <iostream>
#include <Utils/utilities.hpp>

#include <optimization/optimization_problems/2d_hopper/hopper_jump_opt_problem.hpp>
#include <optimization/snopt_wrapper.hpp>


void parse_output(Optimization_Problem_Main* opt_prob){
	Opt_Variable_Manager* var_manager;

	opt_prob->get_var_manager(var_manager);
	sejong::Vector q_states;
	sejong::Vector qdot_states;
	sejong::Vector torque_states;	
	sejong::Vector Fr_states;	
	double h_dt = -1.0;

	for(size_t k = 0; k < var_manager->total_knotpoints+1; k++){

	 	std::cout << "--------------------------" << std::endl;
	 	std::cout << "knotpoint = " << k << std::endl;
	 	var_manager->get_q_states(k, q_states); 	
	 	var_manager->get_qdot_states(k, qdot_states); 	

	 	sejong::pretty_print(q_states, std::cout, "q_states");
	 	sejong::pretty_print(qdot_states, std::cout, "qdot_states");
	 	if (k != 0){
		 	var_manager->get_u_states(k, torque_states); 		 	
		 	var_manager->get_var_reaction_forces(k, Fr_states); 		 	
		 	var_manager->get_var_knotpoint_dt(k-1, h_dt);
		 	sejong::pretty_print(torque_states, std::cout, "torque_states");	
		 	sejong::pretty_print(Fr_states, std::cout, "Fr_states");	 		 	
		 	std::cout << "h_dt = " << h_dt << std::endl;
	 	}
	}	

}

int main(int argc, char **argv)
{
	std::cout << "[Main] Running Hopper Jump Optimization Problem" << std::endl;
	Optimization_Problem_Main* 	opt_problem = new Hopper_Jump_Opt();

	snopt_wrapper::solve_problem_no_gradients(opt_problem);
	parse_output(opt_problem);
	
	delete opt_problem;
	return 0;
}
