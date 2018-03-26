#ifndef HOPPER_HYBRID_DYNAMICS_CONSTRAINT_H
#define HOPPER_HYBRID_DYNAMICS_CONSTRAINT_H

#include <Utils/wrap_eigen.hpp>

#include <string>
#include <iostream>

#include <optimization/hard_constraints/constraint_main.hpp>
#include <optimization/containers/opt_variable_manager.hpp>
#include <optimization/containers/contact_list.hpp>
#include <optimization/containers/contact_mode_schedule.hpp>

#include "HopperModel.hpp"

class Hopper_Hybrid_Dynamics_Constraint: public Constraint_Function{
public:
	Hopper_Hybrid_Dynamics_Constraint();
	Hopper_Hybrid_Dynamics_Constraint(Contact_List* contact_list_in, Contact_Mode_Schedule* contact_mode_schedule_in);	
	~Hopper_Hybrid_Dynamics_Constraint();

	HopperModel* robot_model;	

	sejong::Matrix Jc; // Contact Jacobian
	sejong::Matrix Sa; // Actuation selection matrix

	void setContact_List(Contact_List* contact_list_in);
	void setContact_Mode_Schedule(Contact_Mode_Schedule* contact_mode_schedule_in);	

	void evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec);
	void evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG);
	void evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA);	


private:
	Contact_List* contact_list_obj;
	Contact_Mode_Schedule* contact_mode_schedule_obj;	

	void Initialization();
	void initialize_Flow_Fupp();

	void set_inactive_contacts_to_zero_force(const int& knotpoint, sejong::Vector &Fr_all);
	void Update_Contact_Jacobian_Jc(sejong::Vector &q_state);
};
#endif