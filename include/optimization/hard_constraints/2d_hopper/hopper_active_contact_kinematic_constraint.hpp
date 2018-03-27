#ifndef HOPPER_ACTIVE_CONTACT_KINEMATIC_CONSTRAINT_H
#define HOPPER_ACTIVE_CONTACT_KINEMATIC_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include "HopperModel.hpp"
#include <optimization/hard_constraints/constraint_main.hpp>
#include <optimization/containers/contact_list.hpp>

class Active_Contact_Kinematic_Constraint: public Constraint_Function{
public:
	Active_Contact_Kinematic_Constraint(int knotpoint_in, Contact_List* contact_list_obj_in, int contact_index_in);	
	~Active_Contact_Kinematic_Constraint();

	double l_bound;
	double u_bound;

	HopperModel* robot_model;	
	int contact_index;

	void setContact_List(Contact_List* contact_list_in);

	void evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:
	Contact_List* contact_list_obj;

	void Initialization();
	void initialize_Flow_Fupp();
	void update_states(const int &knotpoint, Opt_Variable_Manager& var_manager);
};
#endif