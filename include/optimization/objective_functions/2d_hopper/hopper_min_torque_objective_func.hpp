#ifndef HOPPER_MIN_TORQUE_OBJ_FUNC_H
#define HOPPER_MIN_TORQUE_OBJ_FUNC_H

#include <optimization/objective_functions/objective_function_main.hpp>
class Hopper_Min_Torque_Objective_Function: public Objective_Function{
public:
	Hopper_Min_Torque_Objective_Function();
	~Hopper_Min_Torque_Objective_Function();

	void evaluate_objective_function(Opt_Variable_Manager& var_manager, double &result);
	void evaluate_objective_gradient(Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA) ;

	void set_var_manager(Opt_Variable_Manager& var_manager);
	void setQ_vals(const int &i, const int &j, const double &value);

	std::string objective_function_name = "Hopper_Min_Torque_Objective_Function";	

	sejong::Matrix Q_u; // Cost matrix for current input

	int num_u;
	int N_total_knotpoints;
};



#endif