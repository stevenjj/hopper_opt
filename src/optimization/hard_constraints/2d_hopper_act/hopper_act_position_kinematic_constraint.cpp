#include <Utils/utilities.hpp>
#include <optimization/hard_constraints/2d_hopper_act/hopper_act_position_kinematic_constraint.hpp>

Hopper_Act_Position_Kinematic_Constraint::Hopper_Act_Position_Kinematic_Constraint(int knotpoint_in, int link_id_in, int dim_in, double l_bound_in, double u_bound_in){
	des_knotpoint = knotpoint_in;
	link_id = link_id_in;
	dim = dim_in;
	l_bound = l_bound_in;
	u_bound = u_bound_in;
	Initialization();
}

Hopper_Act_Position_Kinematic_Constraint::Hopper_Act_Position_Kinematic_Constraint(int link_id_in, int dim_in, double l_bound_in, double u_bound_in){
	link_id = link_id_in;
	dim = dim_in;
	l_bound = l_bound_in;
	u_bound = u_bound_in;
	Initialization();
}

Hopper_Act_Position_Kinematic_Constraint::~Hopper_Act_Position_Kinematic_Constraint(){
	std::cout << "[Hopper_Act_Position_Kinematic_Constraint] Destructor called" << std::endl;
}

void Hopper_Act_Position_Kinematic_Constraint::Initialization(){
	constraint_name = "Position Constraint on link id " + std::to_string(link_id) + " dim " + std::to_string(dim) + " kp " + std::to_string(des_knotpoint);	

	combined_model = Hopper_Combined_Dynamics_Model::GetCombinedModel();	
	initialize_Flow_Fupp();	

}


void Hopper_Act_Position_Kinematic_Constraint::initialize_Flow_Fupp(){
	F_low.clear();
	F_upp.clear();

	// Single equality Constraint
	// We want the ee position, f(q)  to be at a particular position at timestep k 
	// f(q) = des_val
	F_low.push_back(l_bound);	
	F_upp.push_back(u_bound);

	constraint_size = F_low.size();
}


void Hopper_Act_Position_Kinematic_Constraint::evaluate_constraint(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& F_vec){
	F_vec.clear();
	sejong::Vector x_state;
	sejong::Vector xdot_state;
	var_manager.get_x_states(knotpoint, x_state);		
	var_manager.get_xdot_states(knotpoint, xdot_state);			

	sejong::Vector q_state;
	sejong::Vector qdot_state;	

	combined_model->convert_x_xdot_to_q_qdot(x_state, xdot_state, q_state, qdot_state);
	combined_model->robot_model->UpdateModel(q_state, qdot_state);
	combined_model->robot_model->getPosition(q_state, link_id, pos);

	sejong::pretty_print(q_state, std::cout, "q_state");
	sejong::pretty_print(pos, std::cout, "pos");	

	F_vec.push_back(pos[dim]);
}
void Hopper_Act_Position_Kinematic_Constraint::evaluate_sparse_gradient(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& G, std::vector<int>& iG, std::vector<int>& jG){}
void Hopper_Act_Position_Kinematic_Constraint::evaluate_sparse_A_matrix(const int &knotpoint, Opt_Variable_Manager& var_manager, std::vector<double>& A, std::vector<int>& iA, std::vector<int>& jA){}

