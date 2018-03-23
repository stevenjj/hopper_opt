#include <optimization/hard_constraints/2d_hopper/hopper_dynamics_constraint.hpp>
#include "Hopper_Definition.h"
#include <Utils/utilities.hpp>

Hopper_Dynamics_Constraint::Hopper_Dynamics_Constraint(){
	Initialization();
}

Hopper_Dynamics_Constraint::Hopper_Dynamics_Constraint(Contact_List* contact_list_in){ 
	Initialization();
	setContact_List(contact_list_in);
}

Hopper_Dynamics_Constraint::~Hopper_Dynamics_Constraint(){
		std::cout << "[Hopper_Dynamics_Constraint] Destructor called" << std::endl;
}

void Hopper_Dynamics_Constraint::Initialization(){
	constraint_name = "Hopper_Dynamics_Constraint";	
	robot_model = HopperModel::GetRobotModel();	

  Sa.resize(NUM_ACT_JOINT, NUM_QDOT); 
  Sa.setZero();
  Sa.block(0, NUM_VIRTUAL, NUM_ACT_JOINT, NUM_ACT_JOINT) = sejong::Matrix::Identity(NUM_ACT_JOINT, NUM_ACT_JOINT);  

	initialize_Flow_Fupp();	
  std::cout << "[Hopper_Dynamics_Constraint] Initialized" << std::endl;  
}

void Hopper_Dynamics_Constraint::setContact_List(Contact_List* contact_list_in){
  contact_list_obj = contact_list_in;
}

void Hopper_Dynamics_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();

  // Mxddot + Bxdot + Kx - impedance = 0
	for(size_t i = 0; i < (NUM_QDOT); i++){
		F_low.push_back(0.0);	
		F_upp.push_back(0.0);
	}

	constraint_size = F_low.size();
}

void Hopper_Dynamics_Constraint::Update_Contact_Jacobian_Jc(sejong::Vector &q_state){
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



void Hopper_Dynamics_Constraint::evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
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

  sejong::Matrix A_mat;
  sejong::Vector coriolis;
  sejong::Vector gravity;

  // Update the model then update the contact jacobian
  robot_model->UpdateModel(q_state_k, qdot_state_k);
  robot_model->getMassInertia(A_mat);
  robot_model->getCoriolis(coriolis);  
  robot_model->getGravity(gravity);  
  // Update_Contact_Jacobian_Jc(x_state_k); 

  // Aqddot + b + g - Jc^T F = Sa^T * torque
  dynamics_k = A_mat*(qdot_state_k - qdot_state_k_prev)/h_k + coriolis + gravity - Sa.transpose()*u_state_k;

  for(size_t i = 0; i < dynamics_k.size(); i++){
    //std::cout << " constraint " << i << ", value = " << dynamics_k[i] << std::endl;
    F_vec.push_back(dynamics_k[i]);    
  }





}

void Hopper_Dynamics_Constraint::evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Hopper_Dynamics_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}


