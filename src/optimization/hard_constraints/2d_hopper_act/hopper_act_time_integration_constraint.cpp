#include <optimization/hard_constraints/2d_hopper_act/hopper_act_time_integration_constraint.hpp>
#include "Hopper_Definition.h"
#include <Utils/utilities.hpp>

Hopper_Act_Back_Euler_Time_Integration_Constraint::Hopper_Act_Back_Euler_Time_Integration_Constraint(){
	Initialization();
}

Hopper_Act_Back_Euler_Time_Integration_Constraint::~Hopper_Act_Back_Euler_Time_Integration_Constraint(){
		std::cout << "[Hopper_Act_Back_Euler_Time_Integration_Constraint] Destructor called" << std::endl;
}

void Hopper_Act_Back_Euler_Time_Integration_Constraint::Initialization(){
	constraint_name = "Hopper_Act_Back_Euler_Time_Integration_Constraint";	

	initialize_Flow_Fupp();	
  std::cout << "[Hopper_Act_Back_Euler_Time_Integration_Constraint] Initialized" << std::endl;  
}

void Hopper_Act_Back_Euler_Time_Integration_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();

  // x[k] = xdot[k]*h[k] + x[k-1]
	for(size_t i = 0; i < NUM_VIRTUAL + NUM_ACT_JOINT + NUM_ACT_JOINT; i++){
		F_low.push_back(0.0);	
		F_upp.push_back(0.0);
	}

	constraint_size = F_low.size();
}


void Hopper_Act_Back_Euler_Time_Integration_Constraint::evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
  F_vec.clear();

  sejong::Vector integration_constraint; 
  sejong::Vector x_state_k; 
  sejong::Vector x_state_k_prev;   
  sejong::Vector xdot_state_k; 
  double h_k;

  var_manager.get_var_knotpoint_dt(knotpoint - 1, h_k);
  var_manager.get_x_states(knotpoint, x_state_k);
  var_manager.get_x_states(knotpoint-1, x_state_k_prev);  
  var_manager.get_xdot_states(knotpoint, xdot_state_k);  

  integration_constraint = x_state_k - xdot_state_k*h_k - x_state_k_prev;

  for(size_t i = 0; i < integration_constraint.size(); i++){
    std::cout << "integration constraint " << i << ", value = " << integration_constraint[i] << std::endl;    
    F_vec.push_back(integration_constraint[i]);    
  }

}

void Hopper_Act_Back_Euler_Time_Integration_Constraint::evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Hopper_Act_Back_Euler_Time_Integration_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}


