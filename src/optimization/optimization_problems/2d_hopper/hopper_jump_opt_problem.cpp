#include <Utils/utilities.hpp>
#include <optimization/optimization_problems/2d_hopper/hopper_jump_opt_problem.hpp>
#include <optimization/optimization_constants.hpp>

#include <optimization/hard_constraints/2d_hopper/hopper_dynamics_constraint.hpp>
#include <optimization/hard_constraints/2d_hopper/hopper_hybrid_dynamics_constraint.hpp>
#include <optimization/hard_constraints/2d_hopper/hopper_time_integration_constraint.hpp>
#include <optimization/hard_constraints/2d_hopper/hopper_contact_lcp_constraint.hpp>
#include <optimization/hard_constraints/2d_hopper/hopper_active_contact_kinematic_constraint.hpp>


#include <optimization/contacts/2d_hopper/hopper_foot_contact.hpp>

// #include <adt/contacts/adt_draco_contact_toe.hpp>
// #include <adt/contacts/adt_draco_contact_heel.hpp>

// #include <adt/hard_constraints/adt_floor_2d_contact_lcp_constraint.hpp>
// #include <adt/hard_constraints/adt_friction_cone_2d_constraint.hpp>
// #include <adt/hard_constraints/adt_linear_back_euler_time_integration_constraint.hpp>
// #include <adt/hard_constraints/adt_dynamics_constraint.hpp>
// #include <adt/hard_constraints/adt_position_kinematic_constraint.hpp>
// #include <adt/hard_constraints/adt_com_kinematic_constraint.hpp>
// #include <adt/hard_constraints/adt_timestep_constraint.hpp>

#include <string>

Hopper_Jump_Opt::Hopper_Jump_Opt(){
	problem_name = "Hopper Jump Optimization Problem";

	robot_q_init.resize(NUM_Q); 
	robot_qdot_init.resize(NUM_QDOT);	

	robot_q_init.setZero(); 
	robot_qdot_init.setZero();

	Initialization();
}

Hopper_Jump_Opt::~Hopper_Jump_Opt(){
	std::cout << "[Hopper_Jump_Opt] Destructor Called" << std::endl;
}


// Problem Specific Initialization -------------------------------------
void Hopper_Jump_Opt::Initialization(){
	robot_model = HopperModel::GetRobotModel();

	N_total_knotpoints = 6; //6;

	h_dt_min = 0.001; // Minimum knotpoint timestep
	max_normal_force = 1e10;//10000; // Newtons
	max_tangential_force = 10000; // Newtons  	  	

	initialize_starting_configuration();
	initialize_contact_list();
  initialize_contact_mode_schedule();
	initialize_opt_vars();
	initialize_specific_variable_bounds();


	initialize_ti_constraint_list();
 	initialize_td_constraint_list();
	initialize_objective_func();

}
void Hopper_Jump_Opt::initialize_starting_configuration(){
 // Set Virtual Joints
  // z_virt
  robot_q_init[0] = 0.5;
  // z_leg
  robot_q_init[1] = -robot_q_init[0];

}

void Hopper_Jump_Opt::initialize_contact_list(){
  Hopper_Foot_Contact* foot_contact = new Hopper_Foot_Contact();
  contact_list.append_contact(foot_contact);
}

void Hopper_Jump_Opt::initialize_contact_mode_schedule(){
  int foot_contact_index = 0;
  std::vector<int> mode_0_active_contacts; // support phase 
  std::vector<int> mode_1_active_contacts; // flight phase
  std::vector<int> mode_2_active_contacts; // support phase

  mode_0_active_contacts.push_back(foot_contact_index);
  // mode 1 has no active contracts
  mode_2_active_contacts.push_back(foot_contact_index);

  int mode_0_start_time = 1;  int mode_0_final_time = 2;
  int mode_1_start_time = 3;  int mode_1_final_time = 4;
  int mode_2_start_time = 5;  int mode_2_final_time = 6;
  
  contact_mode_schedule.add_new_mode(mode_0_start_time, mode_0_final_time, mode_0_active_contacts);
  contact_mode_schedule.add_new_mode(mode_1_start_time, mode_1_final_time, mode_1_active_contacts);  
  contact_mode_schedule.add_new_mode(mode_2_start_time, mode_2_final_time, mode_2_active_contacts);  

}


void Hopper_Jump_Opt::initialize_ti_constraint_list(){
  int foot_contact_index = 0;
	//ti_constraint_list.append_constraint(new Hopper_Dynamics_Constraint(&contact_list)); 
  ti_constraint_list.append_constraint(new Hopper_Hybrid_Dynamics_Constraint(&contact_list, &contact_mode_schedule));   
  ti_constraint_list.append_constraint(new Hopper_Back_Euler_Time_Integration_Constraint());
  //ti_constraint_list.append_constraint(new Hopper_Floor_Contact_LCP_Constraint(&contact_list, foot_contact_index));
}

void Hopper_Jump_Opt::initialize_td_constraint_list(){
  // Initialize mode schedule kinematic constraints ------------------------------------------------------------------------
  std::vector<int> active_contacts;
  int contact_index;
  for (size_t knotpoint = 1; knotpoint < N_total_knotpoints + 1; knotpoint++){
    active_contacts.clear();
    // Get active contacts list
    contact_mode_schedule.get_active_contacts(knotpoint, active_contacts);

    //for each active contact, add the kinematic constraint for having this contact active
    for(int i = 0; i < active_contacts.size(); i++){
      contact_index = active_contacts[i];
      td_constraint_list.append_constraint(new Active_Contact_Kinematic_Constraint(knotpoint, &contact_list, contact_index));
    }

  }

}



void Hopper_Jump_Opt::initialize_opt_vars(){
	// Optimization Variable Order:
	// opt_init = [q_virt, z, qdot_virt, zdot, delta, delta_dot]
	// opt_td_k = [q_virt_k, z_k, qdot_virt_k, zdot_k, delta_k, delta_dot_k, u_k, Fij_k, Bij_k]
	// opt_var = [opt_init, opt_td_1, opt_td_2, ..., opt_td_k]

	// Append to opt_var_manager the initial configuration
	// ------------------------------------------------------------
	// Set Initial Conditions
	// ------------------------------------------------------------
	// At knotpoint 0, we initialize the joint positions of the robot.
	// [q]
	for(size_t i = 0; i < NUM_Q; i++){
        opt_var_manager.append_variable(new Opt_Variable("q_state_" + std::to_string(i), VAR_TYPE_Q, 0, robot_q_init[i], robot_q_init[i] - OPT_ZERO_EPS, robot_q_init[i] + OPT_ZERO_EPS) );
     } 
	// [qdot]
	for(size_t i = 0; i < NUM_QDOT; i++){
	    opt_var_manager.append_variable(new Opt_Variable("qdot_state_" + std::to_string(i), VAR_TYPE_QDOT, 0, robot_qdot_init[i], robot_qdot_init[i] - OPT_ZERO_EPS, robot_qdot_init[i] + OPT_ZERO_EPS) );
	}

	// set variable manager initial condition offset = NUM_VIRTUAL*2 + (NUM_STATES_PER_ACTUATOR*NUM_ACT)*2 
	int initial_conditions_offset = NUM_Q + NUM_QDOT;
	std::cout << "[Hopper_Jump_Opt] Predicted Number of states : " << initial_conditions_offset << std::endl;
	std::cout << "[Hopper_Jump_Opt] Actual : " << opt_var_manager.get_size() << std::endl;	 
	// ****
	opt_var_manager.initial_conditions_offset = initial_conditions_offset;
	// ****

	// ------------------------------------------------------------------
	// Set Time Independent Variables
	// ------------------------------------------------------------------
	for(size_t k = 1; k < N_total_knotpoints + 1; k++){
  	  opt_var_manager.append_variable(new Opt_Variable("q_state_" + std::to_string(0), VAR_TYPE_Q, k, robot_q_init[0], 0, 10) );
   	  opt_var_manager.append_variable(new Opt_Variable("q_state_" + std::to_string(1), VAR_TYPE_Q, k, robot_q_init[1], -0.75, 0.0) );
	    opt_var_manager.append_variable(new Opt_Variable("qdot_state_" + std::to_string(0), VAR_TYPE_QDOT, k, 0.0, -10, 10) );
	    opt_var_manager.append_variable(new Opt_Variable("qdot_state_" + std::to_string(1), VAR_TYPE_QDOT, k, 0.0, -10, 10) );

		// [torque_u]
		for(size_t i = 0; i < NUM_ACT_JOINT; i++){
	        opt_var_manager.append_variable(new Opt_Variable("torque_u_" + std::to_string(i), VAR_TYPE_U, k, 0.0, -1000, 1000) );
		}

		// [Fr]
		for(size_t i = 0; i < contact_list.get_size(); i++){
			// Apply normal force constraints on z direction		        
		    opt_var_manager.append_variable(new Opt_Variable("Fr_z_" + std::to_string(i), VAR_TYPE_FR, k, 0.0, 0.0, max_normal_force) );
        // Each LCP contact has an alpha (phi(q)) and gamma (Phi(q)*Fr)
        opt_var_manager.append_variable(new Opt_Variable("alpha_c" + std::to_string(i) , VAR_TYPE_ALPHA, k, 0.0, 0.0, OPT_INFINITY) );        
        opt_var_manager.append_variable(new Opt_Variable("gamma_c" + std::to_string(i) , VAR_TYPE_GAMMA, k, 0.0, 0.0, OPT_INFINITY) );
		}
		
		// [h_dt] knotpoint timestep
        opt_var_manager.append_variable(new Opt_Variable("h_dt_" + std::to_string(k) , VAR_TYPE_H, k, h_dt_min, 0.05, 1.0) );
	}
  	// Assign total knotpoints
	opt_var_manager.total_knotpoints = N_total_knotpoints;

	// Compute the size of time independent variables
	// ****
	opt_var_manager.compute_size_time_dep_vars();
	// ****
	int size_of_time_dep_vars = NUM_Q + NUM_QDOT + contact_list.get_size() + N_total_knotpoints;
	std::cout << "[Hopper_Jump_Opt] Predicted Size of Time Dependent Vars : " << size_of_time_dep_vars << std::endl;
	std::cout << "[Hopper_Jump_Opt] Actual : " << opt_var_manager.get_size_timedependent_vars() << std::endl;	 
	int predicted_size_of_F = size_of_time_dep_vars*N_total_knotpoints;
	std::cout << "[Hopper_Jump_Opt] Predicted Size of Opt Vars: " << predicted_size_of_F << std::endl;

}

void Hopper_Jump_Opt::initialize_specific_variable_bounds(){
  // Jump at half way
  opt_var_manager.knotpoint_to_q_state_vars[N_total_knotpoints/2][0]->l_bound = 2.0;
  opt_var_manager.knotpoint_to_q_state_vars[N_total_knotpoints/2][0]->u_bound = OPT_INFINITY;

  //Set final position of the base to be at 0.7
  // opt_var_manager.knotpoint_to_q_state_vars[N_total_knotpoints][0]->l_bound = 0.7 - OPT_ZERO_EPS;
  // opt_var_manager.knotpoint_to_q_state_vars[N_total_knotpoints][0]->u_bound = 0.7 + OPT_ZERO_EPS;

  opt_var_manager.knotpoint_to_qdot_state_vars[N_total_knotpoints][0]->l_bound = -OPT_ZERO_EPS;
  opt_var_manager.knotpoint_to_qdot_state_vars[N_total_knotpoints][0]->u_bound = +OPT_ZERO_EPS;

}

void Hopper_Jump_Opt::initialize_objective_func(){
  // |F_td| = num of time dependent constraint functions
  // |F_ti| = num of time independent constraint functions	
  // T = total timesteps
  objective_function.set_var_manager(opt_var_manager);  
  objective_function.objective_function_index = ti_constraint_list.get_num_constraint_funcs()*N_total_knotpoints + td_constraint_list.get_num_constraint_funcs();

  std::cout << "[Hopper_Jump_Opt] Objective Function has index: " << objective_function.objective_function_index << std::endl;

}


// SNOPT Interface
// Remember to apply the initial conditions offset for the optimization variables 
void Hopper_Jump_Opt::get_init_opt_vars(std::vector<double> &x_vars){
  opt_var_manager.get_init_opt_vars(x_vars);
}
void Hopper_Jump_Opt::get_opt_vars_bounds(std::vector<double> &x_low, std::vector<double> &x_upp){
  opt_var_manager.get_opt_vars_bounds(x_low, x_upp);
}   	  	
void Hopper_Jump_Opt::get_current_opt_vars(std::vector<double> &x_vars_out){
  opt_var_manager.get_current_opt_vars(x_vars_out);
}
void Hopper_Jump_Opt::update_opt_vars(std::vector<double> &x_vars){
  opt_var_manager.update_opt_vars(x_vars);
} 	  		



void Hopper_Jump_Opt::get_F_bounds(std::vector<double> &F_low, std::vector<double> &F_upp){
  F_low.clear();
  F_upp.clear();  
  // Initialize Bounds for Time Independent Constraints
  for(int knotpoint = 1; knotpoint < N_total_knotpoints + 1; knotpoint++){

    for(size_t i = 0; i < ti_constraint_list.get_size(); i++){
      for(size_t j = 0; j < ti_constraint_list.get_constraint(i)->F_low.size(); j++ ){
        F_low.push_back(ti_constraint_list.get_constraint(i)->F_low[j]);
      }
      for(size_t j = 0; j < ti_constraint_list.get_constraint(i)->F_upp.size(); j++ ){
        F_upp.push_back(ti_constraint_list.get_constraint(i)->F_upp[j]);
      }
    }

  }

  // Initialize Bounds for Time Dependent Constraints
  for(size_t i = 0; i < td_constraint_list.get_size(); i++){
    for(size_t j = 0; j < td_constraint_list.get_constraint(i)->F_low.size(); j++ ){
      F_low.push_back(td_constraint_list.get_constraint(i)->F_low[j]);
    }
    for(size_t j = 0; j < td_constraint_list.get_constraint(i)->F_upp.size(); j++ ){
      F_upp.push_back(td_constraint_list.get_constraint(i)->F_upp[j]);
    }
  }


  // Initialize Bounds for the Objective Function
  F_low.push_back(objective_function.F_low);
  F_upp.push_back(objective_function.F_upp);  	
}

void Hopper_Jump_Opt::get_F_obj_Row(int &obj_row){
  obj_row = objective_function.objective_function_index;
  std::cout << "[Hopper_Jump_Opt] Objective Row = " << obj_row << std::endl;
}

void Hopper_Jump_Opt::compute_F_objective_function(double &result_out){
  objective_function.evaluate_objective_function(opt_var_manager, result_out);
  //std::cout << "[Hopper_Jump_Opt] cost = " << result_out << std::endl;
}

void Hopper_Jump_Opt::compute_F(std::vector<double> &F_eval){
  compute_F_constraints(F_eval);
  double cost = 0.0;

  compute_F_objective_function(cost);
  F_eval.push_back(cost);

}


void Hopper_Jump_Opt::compute_F_constraints(std::vector<double> &F_eval){
  std::vector<double> F_vec_const;
  //std::cout << "[Hopper_Jump_Opt] Computing F Constraints" << std::endl;

  // Compute Timestep Independent Constraints
  for(int knotpoint = 1; knotpoint < N_total_knotpoints + 1; knotpoint++){
    for(int i = 0; i < ti_constraint_list.get_size(); i++){
      F_vec_const.clear();
      ti_constraint_list.get_constraint(i)->evaluate_constraint(knotpoint, opt_var_manager, F_vec_const);
      //std::cout << " Adding TD Constraint " << ti_constraint_list.get_constraint(i)->constraint_name << std::endl;

      for(int j = 0; j < F_vec_const.size(); j++){
        //std::cout << "F_td_const[" << j <<"] = " << F_vec_const[j] << std::endl;
        // Add to F_eval
        F_eval.push_back(F_vec_const[j]);
      }
    }
  }

  // Compute Timestep Dependent Constraints
  Constraint_Function* current_constraint;
  for(size_t i = 0; i < td_constraint_list.get_size(); i++){
      F_vec_const.clear();
      current_constraint = td_constraint_list.get_constraint(i);         
      current_constraint->evaluate_constraint(current_constraint->des_knotpoint, opt_var_manager, F_vec_const);
      //std::cout << " Adding TI Constraint " << current_constraint->constraint_name << std::endl;

      //std::cout << "F_vec_const.size() = " << F_vec_const.size() << std::endl;
    for(size_t j = 0; j < current_constraint->F_low.size(); j++ ){
      //std::cout << " des knotpoint = " << current_constraint->des_knotpoint << std::endl;
      //std::cout << "F_ti_const[" << j <<"] = " << F_vec_const[j] << std::endl;      
      F_eval.push_back(F_vec_const[j]);
    }

  }

  // Debug statement  
  /*  for(int j = 0; j < F_eval.size(); j++){
      std::cout << "F_eval[" << j << "] = " << F_eval[j] << std::endl;
    }*/

}



void Hopper_Jump_Opt::compute_G(std::vector<double> &G_eval, std::vector<int> &iGfun, std::vector<int> &jGvar, int &neG){}
void Hopper_Jump_Opt::compute_A(std::vector<double> &A_eval, std::vector<int> &iAfun, std::vector<int> &jAvar, int &neA){}


