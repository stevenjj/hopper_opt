#ifndef HOPPER_FLOOR_CONTACT_LCP_CONSTRAINT_H
#define HOPPER_FLOOR_CONTACT_LCP_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <optimization/hard_constraints/constraint_main.hpp>
#include <optimization/containers/contact_list.hpp>

#include "HopperModel.hpp"

class Hopper_Floor_Contact_LCP_Constraint: public Constraint_Function{
public:
	Hopper_Floor_Contact_LCP_Constraint();
	Hopper_Floor_Contact_LCP_Constraint(Contact_List* contact_list_in, int index_in);	
	~Hopper_Floor_Contact_LCP_Constraint();

	HopperModel* robot_model;	

	void setContact_List(Contact_List* contact_list_in);
	void setContact_index(int index_in);	

	void evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:
	const int num_constraints = 2;
	Contact_List* contact_list_obj;
	int contact_index = -1;	

	// int num_lcp_vars = 4;
	// int num_lcps = 2;	

	void Initialization();
	void initialize_Flow_Fupp();

	void UpdateModel(const int& knotpoint, const int& opt_mode, const sejong::Vector &q, const sejong::Vector &qdot);
};
#endif