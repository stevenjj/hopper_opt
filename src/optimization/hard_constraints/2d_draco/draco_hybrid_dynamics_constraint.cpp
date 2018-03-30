#include <optimization/hard_constraints/2d_draco/draco_hybrid_dynamics_constraint.hpp>
#include "DracoP1Rot_Definition.h"
#include <Utils/utilities.hpp>
#include <algorithm>

Draco_Hybrid_Dynamics_Constraint::Draco_Hybrid_Dynamics_Constraint(){
	Initialization();
}

Draco_Hybrid_Dynamics_Constraint::Draco_Hybrid_Dynamics_Constraint(Contact_List* contact_list_in, Contact_Mode_Schedule* contact_mode_schedule_in){ 
	Initialization();
	setContact_List(contact_list_in);
	setContact_Mode_Schedule(contact_mode_schedule_in);
}

Draco_Hybrid_Dynamics_Constraint::~Draco_Hybrid_Dynamics_Constraint(){
		std::cout << "[Draco_Hybrid_Dynamics_Constraint] Destructor called" << std::endl;
}

void Draco_Hybrid_Dynamics_Constraint::Initialization(){
	constraint_name = "Draco_Hybrid_Dynamics_Constraint";	
	robot_model = DracoModel::GetDracoModel();	

	Sa.resize(NUM_ACT_JOINT, NUM_QDOT); 
	Sa.setZero();
	Sa.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);  

	initialize_Flow_Fupp();	
  std::cout << "[Draco_Hybrid_Dynamics_Constraint] Initialized" << std::endl;  
}

void Draco_Hybrid_Dynamics_Constraint::setContact_List(Contact_List* contact_list_in){
  contact_list_obj = contact_list_in;
}
void Draco_Hybrid_Dynamics_Constraint::setContact_Mode_Schedule(Contact_Mode_Schedule* contact_mode_schedule_in){
  contact_mode_schedule_obj = contact_mode_schedule_in;
}

void Draco_Hybrid_Dynamics_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();

  // Aqddot + b + g - Jc^T Fr = [0, tau]^T
	for(size_t i = 0; i < (NUM_QDOT); i++){
		F_low.push_back(0.0);	
		F_upp.push_back(0.0);
	}

	constraint_size = F_low.size();
}

void Draco_Hybrid_Dynamics_Constraint::Update_Contact_Jacobian_Jc(sejong::Vector &q_state){
  // Stack the contact Jacobians
  // Extract the segment of Reaction Forces corresponding to this contact-------------------
  sejong::Matrix J_tmp;
  int prev_row_size = 0;
  for (size_t i = 0; i < contact_list_obj->get_size(); i++){
    // Get the Jacobian for the current contact
    contact_list_obj->get_contact(i)->getContactJacobian(q_state, J_tmp);   
    Jc.conservativeResize(prev_row_size + J_tmp.rows(), NUM_QDOT);
   	Jc.block(prev_row_size, 0, J_tmp.rows(), NUM_QDOT) = J_tmp;
   	prev_row_size += J_tmp.rows();
  }

}

void Draco_Hybrid_Dynamics_Constraint::set_inactive_contacts_to_zero_force(const int& knotpoint, sejong::Vector &Fr_all){
  // Get all the active_contacts
  std::vector<int> active_contacts;
  contact_mode_schedule_obj->get_active_contacts(knotpoint, active_contacts);

  // std::cout << "" <<std::endl;
  // std::cout << "Knotpoint = " << knotpoint << " has active contacts with indices: ";
  // if (active_contacts.size() == 0){
  //   std::cout << "no active contacts" << std::endl;
  // }else{
  //   for (size_t i = 0; i < active_contacts.size(); i++){
  //     std::cout << active_contacts[i] << ",";
  //   }
  //   std::cout << "" <<std::endl;
  // }


  // Go through all the contacts. If it is not in the active_contacts list, set the contact forces to 0.
  Contact* current_contact;
  int current_contact_size;
  sejong::Vector Fr_contact;

  for (size_t contact_index = 0; contact_index < contact_list_obj->get_size(); contact_index++){
  // Get the current contact    
    current_contact = contact_list_obj->get_contact(contact_index);
    current_contact_size = current_contact->contact_dim;

    // Extract the segment of Reaction Forces corresponding to this contact-------------------
    int index_offset = 0;
    for (size_t i = 0; i < contact_index; i++){
      index_offset += contact_list_obj->get_contact(i)->contact_dim;   
    }

    // The contact forces for this contact
    // Fr_contact = Fr_all.segment(index_offset, current_contact_size);

    // Check if this is an active contact:
    if (!(std::find(active_contacts.begin(), active_contacts.end(), contact_index) != active_contacts.end())) {
      /* Contact index is not in this list */
      // Set the contact forces to 0.
       Fr_all.segment(index_offset, current_contact_size) = sejong::Vector::Zero(current_contact_size);
    }

  }


}


void Draco_Hybrid_Dynamics_Constraint::evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
  F_vec.clear();

  sejong::Vector q_state_k; 
  sejong::Vector q_state_k_prev;   
  sejong::Vector qdot_state_k; 
  sejong::Vector qdot_state_k_prev; 

  double h_k;
  sejong::Vector u_state_k;
  sejong::Vector Fr_state_k;
  sejong::Vector dynamics_k; 
  var_manager.get_var_knotpoint_dt(knotpoint - 1, h_k);

  var_manager.get_var_states(knotpoint, q_state_k, qdot_state_k);
  var_manager.get_var_states(knotpoint - 1, q_state_k_prev, qdot_state_k_prev);
  var_manager.get_u_states(knotpoint, u_state_k);
  var_manager.get_var_reaction_forces(knotpoint, Fr_state_k);  

  set_inactive_contacts_to_zero_force(knotpoint, Fr_state_k);

  sejong::Matrix A_mat;
  sejong::Vector coriolis;
  sejong::Vector gravity;

  // Update the model then update the contact jacobian
  robot_model->UpdateModel(q_state_k, qdot_state_k);
  robot_model->getMassInertia(A_mat);
  robot_model->getCoriolis(coriolis);  
  robot_model->getGravity(gravity);  
  Update_Contact_Jacobian_Jc(q_state_k); 

  // Aqddot + b + g - Jc^T F = Sa^T * torque
  dynamics_k = A_mat*(qdot_state_k - qdot_state_k_prev)/h_k + coriolis + gravity - Jc.transpose()*Fr_state_k - Sa.transpose()*u_state_k;

  for(size_t i = 0; i < dynamics_k.size(); i++){
    //std::cout << "dynamics constraint " << i << ", value = " << dynamics_k[i] << std::endl;
    F_vec.push_back(dynamics_k[i]);    
  }





}

void Draco_Hybrid_Dynamics_Constraint::evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Draco_Hybrid_Dynamics_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}


