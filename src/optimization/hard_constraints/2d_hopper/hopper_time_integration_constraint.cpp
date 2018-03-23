#include <optimization/hard_constraints/2d_hopper/hopper_time_integration_constraint.hpp>
#include "Hopper_Definition.h"
#include <Utils/utilities.hpp>

Hopper_Back_Euler_Time_Integration_Constraint::Hopper_Back_Euler_Time_Integration_Constraint(){
	Initialization();
}

Hopper_Back_Euler_Time_Integration_Constraint::~Hopper_Back_Euler_Time_Integration_Constraint(){
		std::cout << "[Hopper_Back_Euler_Time_Integration_Constraint] Destructor called" << std::endl;
}

void Hopper_Back_Euler_Time_Integration_Constraint::Initialization(){
	constraint_name = "Hopper_Back_Euler_Time_Integration_Constraint";	
	robot_model = HopperModel::GetRobotModel();	

	initialize_Flow_Fupp();	
  std::cout << "[Hopper_Back_Euler_Time_Integration_Constraint] Initialized" << std::endl;  
}

void Hopper_Back_Euler_Time_Integration_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();

  // q[k] = qdot[k]*h[k] + q[k-1]
	for(size_t i = 0; i < NUM_Q; i++){
		F_low.push_back(0.0);	
		F_upp.push_back(0.0);
	}

	constraint_size = F_low.size();
}


void Hopper_Back_Euler_Time_Integration_Constraint::evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
  F_vec.clear();

  sejong::Vector integration_constraint; 
  sejong::Vector q_state_k; 
  sejong::Vector q_state_k_prev;   
  sejong::Vector qdot_state_k; 
  double h_k;

  var_manager.get_var_knotpoint_dt(knotpoint - 1, h_k);
  var_manager.get_q_states(knotpoint, q_state_k);
  var_manager.get_q_states(knotpoint-1, q_state_k_prev);  
  var_manager.get_qdot_states(knotpoint, qdot_state_k);  

  integration_constraint = q_state_k - qdot_state_k*h_k - q_state_k_prev;

  sejong::pretty_print(q_state_k, std::cout, "q_state_k");
  sejong::pretty_print(qdot_state_k, std::cout, "qdot_state_k");
  sejong::pretty_print(q_state_k_prev, std::cout, "q_state_k_prev");    

  for(size_t i = 0; i < integration_constraint.size(); i++){
    //std::cout << " constraint " << i << ", value = " << integration_constraint[i] << std::endl;
    F_vec.push_back(integration_constraint[i]);    
  }

}

void Hopper_Back_Euler_Time_Integration_Constraint::evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Hopper_Back_Euler_Time_Integration_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}


