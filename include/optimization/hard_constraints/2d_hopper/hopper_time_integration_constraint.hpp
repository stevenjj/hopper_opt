#ifndef HOPPER_TIME_INTEGRATION_CONSTRAINT_H
#define HOPPER_TIME_INTEGRATION_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <optimization/hard_constraints/constraint_main.hpp>
#include <optimization/containers/constraint_list.hpp>

#include "HopperModel.hpp"

class Hopper_Back_Euler_Time_Integration_Constraint: public Constraint_Function{
public:
	Hopper_Back_Euler_Time_Integration_Constraint();
	~Hopper_Back_Euler_Time_Integration_Constraint();

	HopperModel* robot_model;	

	void evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:

	void Initialization();
	void initialize_Flow_Fupp();

};
#endif