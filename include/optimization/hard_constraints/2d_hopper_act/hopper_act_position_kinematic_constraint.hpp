#ifndef HOPPER_ACT_POS_1D_KINEMATIC_CONSTRAINT_H
#define HOPPER_ACT_POS_1D_KINEMATIC_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <optimization/hard_constraints/constraint_main.hpp>
#include <hopper_combined_dynamics_model/hopper_combined_dynamics_model.hpp>

#define Z_DIM 0

class Hopper_Act_Position_Kinematic_Constraint: public Constraint_Function{
public:
	Hopper_Act_Position_Kinematic_Constraint(int knotpoint_in, int link_id_in, int dim_in, double l_bound_in, double u_bound_in);	
	Hopper_Act_Position_Kinematic_Constraint(int link_id_in, int dim_in, double l_bound_in, double u_bound_in);
	~Hopper_Act_Position_Kinematic_Constraint();

	int link_id;
	int dim;

	double l_bound;
	double u_bound;

	Hopper_Combined_Dynamics_Model* combined_model;	

	sejong::Vector pos;

	void evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:
	void Initialization();
	void initialize_Flow_Fupp();


};
#endif