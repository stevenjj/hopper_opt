#include <optimization/objective_functions/2d_hopper/hopper_min_torque_objective_func.hpp>
#include <Utils/utilities.hpp>

Hopper_Min_Torque_Objective_Function::Hopper_Min_Torque_Objective_Function(){}

Hopper_Min_Torque_Objective_Function::~Hopper_Min_Torque_Objective_Function(){
	std::cout << "[Hopper_Min_Torque_Objective_Function Destructor] called" << std::endl;
}

void Hopper_Min_Torque_Objective_Function::set_var_manager(Opt_Variable_Manager& var_manager){
	num_u = var_manager.get_num_u_vars();
	N_total_knotpoints = var_manager.total_knotpoints;
	Q_u = sejong::Matrix::Identity(num_u, num_u);
}

void Hopper_Min_Torque_Objective_Function::evaluate_objective_function(Opt_Variable_Manager& var_manager, double &result){
	sejong::Vector u_states;	

	double cost = 0.0;
	double h_k = 1.0; 

	for(size_t k = 1; k < N_total_knotpoints + 1; k++){
		var_manager.get_u_states(k, u_states);

		cost += u_states.transpose()*Q_u*u_states;
		//cost += u_states.transpose()*zdot_states;
		//cost *= h_k;
		std::cout << "cost = " << cost << std::endl;
	}
	result = cost;
}


void Hopper_Min_Torque_Objective_Function::evaluate_objective_gradient(Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Hopper_Min_Torque_Objective_Function::evaluate_sparse_A_matrix(Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}
void Hopper_Min_Torque_Objective_Function::setQ_vals(const int &i, const int &j, const double &value){}

