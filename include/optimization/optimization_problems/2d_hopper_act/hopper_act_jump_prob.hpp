#ifndef HOPPER_ACT_JUMP_OPTIMIZATION_PROBLEM_H
#define HOPPER_ACT_JUMP_OPTIMIZATION_PROBLEM_H

#include <optimization/optimization_problems/opt_problem_main.hpp>
#include <optimization/containers/opt_variable.hpp>
#include <optimization/containers/opt_variable_manager.hpp>
#include <optimization/containers/constraint_list.hpp>
#include <optimization/containers/contact_list.hpp>
#include <optimization/containers/contact_mode_schedule.hpp>

#include <optimization/objective_functions/objective_function_main.hpp>
//#include <optimization/objective_functions/2d_hopper/hopper_min_torque_objective_func.hpp>
#include <optimization/objective_functions/2d_hopper_act/hopper_act_min_torque_objective_func.hpp>


#include <hopper_combined_dynamics_model/hopper_combined_dynamics_model.hpp>
#include "HopperModel.hpp"
#include "Hopper_Definition.h"


class Hopper_Act_Jump_Opt: public Optimization_Problem_Main{
public:
  Hopper_Act_Jump_Opt();
  ~Hopper_Act_Jump_Opt();	

  Opt_Variable_Manager    			   opt_var_manager;
  Hopper_Combined_Dynamics_Model*  combined_model;

  Contact_List 								  contact_list;
  Contact_Mode_Schedule         contact_mode_schedule;

  Constraint_List 							td_constraint_list; // Time Dependent Constraint List, exists for a particular timestep
  Constraint_List 							ti_constraint_list;	// Time Independent Constraint List, exists for all timesteps

  sejong::Vector 								robot_q_init;
  sejong::Vector 								robot_qdot_init; 

  sejong::Vector                act_z_init;
  sejong::Vector                act_zdot_init;
  sejong::Vector                act_delta_init;
  sejong::Vector                act_delta_dot_init; 

  int 										      N_total_knotpoints;

  double										    h_dt_min;
  double										    max_normal_force;
  double										    max_tangential_force;	  	

  //Objective_Function				    objective_function;
  Hopper_Act_Min_Torque_Objective_Function            objective_function;  
//  Hopper_Min_Torque_Objective_Function            objective_function;


  int constraint_size; // Unused

  void get_var_manager(Opt_Variable_Manager* &var_manager_out){
    var_manager_out = &opt_var_manager;
  }

  // Interface to SNOPT -------------------------------------------------------------------

  void get_init_opt_vars(std::vector<double> &x_vars);   	
  void get_opt_vars_bounds(std::vector<double> &x_low, std::vector<double> &x_upp);   	  	

  void update_opt_vars(std::vector<double> &x_vars); 	  		
  void get_current_opt_vars(std::vector<double> &x_vars_out);   	  	  	

  void get_F_bounds(std::vector<double> &F_low, std::vector<double> &F_upp);
  void get_F_obj_Row(int &obj_row);	

  void compute_F(std::vector<double> &F_eval);
  void compute_F_constraints(std::vector<double> &F_eval);
  void compute_F_objective_function(double &result_out);

  void compute_G(std::vector<double> &G_eval, std::vector<int> &iGfun, std::vector<int> &jGvar, int &neG);
  void compute_A(std::vector<double> &A_eval, std::vector<int> &iAfun, std::vector<int> &jAvar, int &neA);

private:
  void Initialization();
  void initialize_starting_configuration();
  void initialize_contact_list();
  void initialize_contact_mode_schedule();
  void initialize_td_constraint_list();
  void initialize_ti_constraint_list();  

  void initialize_opt_vars();

  void initialize_specific_variable_bounds();

  void initialize_objective_func();


};

#endif