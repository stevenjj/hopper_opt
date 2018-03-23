#include <Utils/utilities.hpp>
#include <optimization/hard_constraints/2d_hopper/hopper_contact_lcp_constraint.hpp>
#include <optimization/optimization_constants.hpp>
#include "Hopper_Definition.h"

Hopper_Floor_Contact_LCP_Constraint::Hopper_Floor_Contact_LCP_Constraint(){
  Initialization();  
}

Hopper_Floor_Contact_LCP_Constraint::Hopper_Floor_Contact_LCP_Constraint(Contact_List* contact_list_in, int index_in){
  setContact_List(contact_list_in);
  setContact_index(index_in);
  Initialization();
}

Hopper_Floor_Contact_LCP_Constraint::~Hopper_Floor_Contact_LCP_Constraint(){
  std::cout << "[Hopper_Floor_Contact_LCP_Constraint] Destructor called" << std::endl;  
}

void Hopper_Floor_Contact_LCP_Constraint::Initialization(){
  std::cout << "[Hopper_Floor_Contact_LCP_Constraint] Initialization called" << std::endl;
  constraint_name = "Hopper Contact LCP Constraint";  
  robot_model = HopperModel::GetRobotModel();  
	initialize_Flow_Fupp();	
}

void Hopper_Floor_Contact_LCP_Constraint::initialize_Flow_Fupp(){
	// Phi(q)*||Fr|| = 0. Try: Phi(q)*||Fr|| <= 0
  F_low.push_back(0.0); 
  F_upp.push_back(0.0);
  // Phi(q) >= 0
  F_low.push_back(0.0); 
  F_upp.push_back(0.0);
  constraint_size = F_low.size();
}

void Hopper_Floor_Contact_LCP_Constraint::setContact_List(Contact_List* contact_list_in){
  contact_list_obj = contact_list_in;
}
void Hopper_Floor_Contact_LCP_Constraint::setContact_index(int index_in){
  contact_index = index_in;  
}  

void Hopper_Floor_Contact_LCP_Constraint::evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
  Contact* current_contact = contact_list_obj->get_contact(contact_index);
  int contact_link_id = current_contact->contact_link_id;

  // Get robot virtual states and actuator z position states---------------------------------------------------------------------------
  sejong::Vector q_state;
  sejong::Vector qdot_state;
  var_manager.get_q_states(knotpoint, q_state);
  var_manager.get_qdot_states(knotpoint, qdot_state);

  // Update the robot model
  robot_model->UpdateModel(q_state, qdot_state);  

  // Get Fr_states--------------------------------------------------------------------------
  sejong::Vector Fr_all;
  var_manager.get_var_reaction_forces(knotpoint, Fr_all);
  int current_contact_size = current_contact->contact_dim;

  // Extract the segment of Reaction Forces corresponding to this contact-------------------
  int index_offset = 0;
  for (size_t i = 0; i < contact_index; i++){
    index_offset += contact_list_obj->get_contact(i)->contact_dim;   
  }
  sejong::Vector Fr_contact = Fr_all.segment(index_offset, current_contact_size);


  // Compute the 2 norm of the force
  double Fr_l2_norm_squared = std::pow(Fr_contact.lpNorm<2>(), 2);

  // Get the contact position vec
  sejong::Vector contact_pos_vec;
  robot_model->getPosition(q_state, contact_link_id, contact_pos_vec);

  double phi_contact_dis = contact_pos_vec[0];
  double complimentary_constraint = phi_contact_dis*Fr_l2_norm_squared;

  F_vec.push_back(complimentary_constraint); // Phi*||Fr|| = 0
  F_vec.push_back(phi_contact_dis);    // Phi >= 0

}

void Hopper_Floor_Contact_LCP_Constraint::evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){
}

void Hopper_Floor_Contact_LCP_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){
}
