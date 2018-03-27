#include <Utils/utilities.hpp>
#include <optimization/hard_constraints/2d_hopper/hopper_active_contact_kinematic_constraint.hpp>
#include <optimization/optimization_constants.hpp>
#include "Hopper_Definition.h"

Active_Contact_Kinematic_Constraint::Active_Contact_Kinematic_Constraint(int knotpoint_in, Contact_List* contact_list_obj_in, int contact_index_in){
	des_knotpoint = knotpoint_in;
	contact_index = contact_index_in;
	setContact_List(contact_list_obj_in);
	Initialization();	
}

Active_Contact_Kinematic_Constraint::~Active_Contact_Kinematic_Constraint(){
}

void Active_Contact_Kinematic_Constraint::Initialization(){
	robot_model = HopperModel::GetRobotModel();
	initialize_Flow_Fupp();
}

void Active_Contact_Kinematic_Constraint::initialize_Flow_Fupp(){
	// Phi(q) = 0.0 <=> Contact is active.
	F_low.push_back(0.0);
	F_upp.push_back(0.0);	
	constraint_size = F_low.size();
}

void Active_Contact_Kinematic_Constraint::setContact_List(Contact_List* contact_list_in){
	contact_list_obj = contact_list_in;
}

void Active_Contact_Kinematic_Constraint::evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
	// Get contact link
	Contact* current_contact = contact_list_obj->get_contact(contact_index);
	int contact_link_id = current_contact->contact_link_id;

  // Get current robot states
  sejong::Vector q_state;
  sejong::Vector qdot_state;
  var_manager.get_q_states(knotpoint, q_state);
  var_manager.get_qdot_states(knotpoint, qdot_state);

  // Update the robot model
  robot_model->UpdateModel(q_state, qdot_state);  

  // Get distance to contact
  double distance_from_ground = 0.0;
  current_contact->signed_distance_to_contact(q_state, distance_from_ground);

  //std::cout << "knotpoint = " << knotpoint << std::endl;
  //sejong::pretty_print(q_state, std::cout, "q_state");
  //std::cout << "Contact Link ID = " << contact_link_id << ", distance to contact = " << distance_from_ground << std::endl;

  F_vec.push_back(distance_from_ground);

}



void Active_Contact_Kinematic_Constraint::evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Active_Contact_Kinematic_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}	