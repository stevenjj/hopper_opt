#ifndef CONSTRAINT_FUNC_H
#define CONSTRAINT_FUNC_H

#include <Utils/wrap_eigen.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <hdt/containers/opt_variable.hpp>
#include <hdt/containers/opt_variable_manager.hpp>
#include <hdt/optimization_constants.hpp>

class Constraint_Function{
public:
	Constraint_Function(){}
	virtual ~Constraint_Function(){
		std::cout << "Constraint Function Destructor called" << std::endl;
	}
	virtual void evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec) {}
	virtual void evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG) {}
	virtual void evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA) {}	

	// Each constraint containts its bounds
	std::vector<double> F_low;
	std::vector<double> F_upp;

	std::string constraint_name = "undefined constraint";	

	int des_knotpoint = -1; // for time independent constraints only
	int constraint_index = -1; // Modified by the Constraint List Holder
	int constraint_size = 0; // Modified by the Object Constructor
	
	virtual int get_constraint_size(){ 
		constraint_size = F_low.size(); 
		return constraint_size;
	}
	virtual int get_constraint_index(){ return constraint_index;}	

	// virtual void test_function(){}
	// virtual	void test_function2(const sejong::Vector &q, const sejong::Vector &qdot, sejong::Matrix &B_out, sejong::Vector &c_out){}
};

#endif