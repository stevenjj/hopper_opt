#ifndef OBJ_FUNC_MAIN_H
#define OBJ_FUNC_MAIN_H

#include <Utils/wrap_eigen.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <optimization/containers/opt_variable.hpp>
#include <optimization/containers/opt_variable_manager.hpp>
#include <optimization/optimization_constants.hpp>

class Objective_Function{
public:
	Objective_Function(){}
	virtual ~Objective_Function(){
		std::cout << "Objective Function Destructor called" << std::endl;
	}
	virtual void evaluate_objective_function(Opt_Variable_Manager& var_manager, double &result) {}
	virtual void evaluate_objective_gradient(Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG) {}
	virtual void evaluate_sparse_A_matrix(Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA) {}	

	virtual void set_var_manager(Opt_Variable_Manager& var_manager){}

	// Each constraint containts its bounds
	double F_low = -OPT_INFINITY;
	double F_upp = OPT_INFINITY;

	std::string objective_function_name = "undefined objective function";	
	int objective_function_index = -1; // Modified after all constraints have been specified
	
};



#endif