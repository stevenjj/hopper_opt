#include <optimization/objective_functions/2d_hopper_act/hopper_act_min_torque_objective_func.hpp>
#include <Utils/utilities.hpp>

Hopper_Act_Min_Torque_Objective_Function::Hopper_Act_Min_Torque_Objective_Function(){
	combined_model = Hopper_Combined_Dynamics_Model::GetCombinedModel();
}

Hopper_Act_Min_Torque_Objective_Function::~Hopper_Act_Min_Torque_Objective_Function(){
	std::cout << "[Hopper_Act_Min_Torque_Objective_Function Destructor] called" << std::endl;
}

void Hopper_Act_Min_Torque_Objective_Function::set_var_manager(Opt_Variable_Manager& var_manager){
	num_u = var_manager.get_num_u_vars();
	N_total_knotpoints = var_manager.total_knotpoints;
	Q_u = sejong::Matrix::Identity(num_u, num_u);
}

void Hopper_Act_Min_Torque_Objective_Function::evaluate_objective_function(Opt_Variable_Manager& var_manager, double &result){
	sejong::Vector x_states;
	sejong::Vector x_states_prev;
	sejong::Vector qdot_states;
	sejong::Vector q_init_states;

	sejong::Vector u_states;	
	sejong::Vector Fr_states;	


	double cost = 0.0;
	double h_k = 1.0; 

	// add q_state to cost.

	sejong::Vector q_states;
	sejong::Vector q_states_prev;

	for(size_t k = 1; k < N_total_knotpoints + 1; k++){
		var_manager.get_u_states(k, u_states);
		var_manager.get_var_reaction_forces(k, Fr_states);
		var_manager.get_x_states(k, x_states);
		var_manager.get_x_states(k-1, x_states_prev);

		cost += u_states.transpose()*Q_u*u_states;
		cost += (x_states - x_states_prev).transpose()*(x_states - x_states_prev);
		cost += (Fr_states.transpose()*Fr_states);		

		// combined_model->convert_x_to_q(x_states, q_states);
		// combined_model->convert_x_to_q(x_states_prev, q_states_prev);		
		// cost += (q_states - q_states_prev).transpose()*(q_states - q_states_prev);

		//cost += u_states.transpose()*zdot_states;
		cost *= h_k;
		//std::cout << "cost = " << cost << std::endl;
	}
	result = cost;
}


void Hopper_Act_Min_Torque_Objective_Function::evaluate_objective_gradient(Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Hopper_Act_Min_Torque_Objective_Function::evaluate_sparse_A_matrix(Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}

