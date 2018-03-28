#ifndef HOPPER_ACT_TIME_INTEGRATION_CONSTRAINT_H
#define HOPPER_ACT_TIME_INTEGRATION_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <optimization/hard_constraints/constraint_main.hpp>
#include <optimization/containers/constraint_list.hpp>

class Hopper_Act_Back_Euler_Time_Integration_Constraint: public Constraint_Function{
public:
	Hopper_Act_Back_Euler_Time_Integration_Constraint();
	~Hopper_Act_Back_Euler_Time_Integration_Constraint();

	void evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:

	void Initialization();
	void initialize_Flow_Fupp();

};
#endif