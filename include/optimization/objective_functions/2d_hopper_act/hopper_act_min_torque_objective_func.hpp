#ifndef HOPPER_ACT_MIN_TORQUE_OBJ_FUNC_H
#define HOPPER_ACT_MIN_TORQUE_OBJ_FUNC_H

#include <optimization/objective_functions/objective_function_main.hpp>
#include <hopper_combined_dynamics_model/hopper_combined_dynamics_model.hpp>

class Hopper_Act_Min_Torque_Objective_Function: public Objective_Function{
public:
	Hopper_Act_Min_Torque_Objective_Function();
	~Hopper_Act_Min_Torque_Objective_Function();

	Hopper_Combined_Dynamics_Model* combined_model;

	void evaluate_objective_function(Opt_Variable_Manager& var_manager, double &result);
	void evaluate_objective_gradient(Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA) ;

	void set_var_manager(Opt_Variable_Manager& var_manager);

	std::string objective_function_name = "Hopper_Act_Min_Torque_Objective_Function";	

	sejong::Matrix Q_u; // Cost matrix for current input

	int num_u;
	int N_total_knotpoints;
};



#endif