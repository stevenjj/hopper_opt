#include <hopper_actuator_model/hopper_actuator_model.hpp>
#include <Utils/utilities.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

HopperActuatorModel* HopperActuatorModel::GetActuatorModel(){
    static HopperActuatorModel hopper_act_model;
    return & hopper_act_model;
}

// Singleton Destructor
HopperActuatorModel::~HopperActuatorModel(){
}

// Singleton Constructor
HopperActuatorModel::HopperActuatorModel(){
	Initialization();
	printf("[HopperActuatorModel] Constructed\n");
	sejong::pretty_print(q_l_bound, std::cout, "q Lower Bound");
	sejong::pretty_print(q_u_bound, std::cout, "q Upper Bound");
	sejong::pretty_print(z_l_bound, std::cout, "z Lower Bound");		
	sejong::pretty_print(z_u_bound, std::cout, "z Upper Bound");
}

void HopperActuatorModel::Initialization(){
    // Mass Elements (Kg)
    M_motor.resize(NUM_ACTUATORS); 	M_motor.setZero();
    M_spring.resize(NUM_ACTUATORS); M_spring.setZero();
    M_load.resize(NUM_ACTUATORS);  	M_load.setZero();

    // Damping Elements (N/m-s)
    B_motor.resize(NUM_ACTUATORS); B_motor.setZero();
    B_spring.resize(NUM_ACTUATORS); B_spring.setZero();
    B_load.resize(NUM_ACTUATORS); B_load.setZero();

    // Spring Elements (N/m)
    K_motor.resize(NUM_ACTUATORS); K_motor.setZero();
    K_spring.resize(NUM_ACTUATORS); K_spring.setZero();

	// Torque Constant (N-m/Amperes)	        
    K_m.resize(NUM_ACTUATORS); K_m.setZero();

	// Actuator Moment Arm (m)
    r_arm.resize(NUM_ACTUATORS); r_arm.setZero();
	// Zero spring force actuator position
    z_o.resize(NUM_ACTUATORS); z_o.setZero();		

	// Zero spring force joint position
    q_o.resize(NUM_ACTUATORS); q_o.setZero();		    

    q_l_bound.resize(NUM_ACTUATORS); q_l_bound.setZero();
    q_u_bound.resize(NUM_ACTUATORS); q_u_bound.setZero();
    z_l_bound.resize(NUM_ACTUATORS); z_l_bound.setZero();
    z_u_bound.resize(NUM_ACTUATORS); z_u_bound.setZero();   


    // Set Default Values
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
	    // Mass Elements (Kg)
	    M_motor[i] = 293.0;//293.0;
	    M_spring[i] = 1.7;
	    M_load[i] = 0.1; // Unknown

	    // Damping Elements (N/m-s)
	    B_motor[i] = 1680;
	    B_spring[i] = 0.0;
	    B_load[i] = 0.0;

	    // Spring Elements (N/m)
	    K_motor[i] = 0.0; // Motor has no spring
	    K_spring[i] = 250000.0;//8109000; // Stiff spring estimated by hcrl

		// Torque Constant (N/Amperes)	    
		K_m[i] = -157.0;

		// Actuator Moment Arm (m)
		r_arm[i] = 0.11; //  		
		// Zero spring force actuator position
		z_o[i] = 0.0;
		q_o[i] = 0.0;
	}

	// Initial Configuration for which z = 0;
	q_o[SJ_Hopper_ActuatorID::act_leg_extend] = -0.5;

	// Lower Bound of joint configuration
	q_l_bound[SJ_Hopper_ActuatorID::act_leg_extend] = -0.75;

	// Upper Bound of joint configuration
	q_u_bound[SJ_Hopper_ActuatorID::act_leg_extend] = 0.25;

	getFull_act_pos_z(q_l_bound, z_l_bound);
	getFull_act_pos_z(q_u_bound, z_u_bound);	

}

void HopperActuatorModel::getMassMatrix(sejong::Matrix &M_act){
	M_act.resize(NUM_TOTAL_STATES, NUM_TOTAL_STATES);
	M_act.setZero();
/*	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		M_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i) = M_motor[i];
		M_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i + 1) = - (M_motor[i] + M_spring[i]);		
		M_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i) = M_load[i];		
		M_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i + 1) = M_spring[i];				
	}*/
	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		M_act(i, NUM_STATES_PER_ACTUATOR*i) = M_motor[i];
		M_act(i, NUM_STATES_PER_ACTUATOR*i + 1) = -(M_motor[i] + M_spring[i]);		
	}

	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		M_act(NUM_ACTUATORS + i, NUM_STATES_PER_ACTUATOR*i) = M_load[i];		
		M_act(NUM_ACTUATORS + i, NUM_STATES_PER_ACTUATOR*i + 1) = M_spring[i];				
	}

}

void HopperActuatorModel::getDampingMatrix(sejong::Matrix &B_act){
	B_act.resize(NUM_TOTAL_STATES, NUM_TOTAL_STATES);
	B_act.setZero();
/*	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		B_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i) = B_motor[i];
		B_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i + 1) = - (B_motor[i] + B_spring[i]);		
		B_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i) = B_load[i];		
		B_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i + 1) = B_spring[i];				
	}*/

	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		B_act(i, NUM_STATES_PER_ACTUATOR*i) = B_motor[i];
		B_act(i, NUM_STATES_PER_ACTUATOR*i + 1) = -(B_motor[i] + B_spring[i]);		
	}
	// Assign block diagonally
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		B_act(NUM_ACTUATORS + i, NUM_STATES_PER_ACTUATOR*i) = B_load[i];		
		B_act(NUM_ACTUATORS + i, NUM_STATES_PER_ACTUATOR*i + 1) = B_spring[i];		
	}

}

void HopperActuatorModel::getStiffnessMatrix(sejong::Matrix &K_act){
	K_act.resize(NUM_TOTAL_STATES, NUM_TOTAL_STATES);
	K_act.setZero();
	// Assign block diagonally	
/*	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		K_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i) = K_motor[i];
		K_act(NUM_STATES_PER_ACTUATOR*i, NUM_STATES_PER_ACTUATOR*i + 1) = K_motor[i] - K_spring[i];		
		K_act(NUM_STATES_PER_ACTUATOR*i + 1, NUM_STATES_PER_ACTUATOR*i + 1) = K_spring[i];				
	}*/

	// Assign block diagonally	
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		K_act(i, NUM_STATES_PER_ACTUATOR*i) = K_motor[i];
		K_act(i, NUM_STATES_PER_ACTUATOR*i + 1) =  - (K_motor[i] + K_spring[i]);		
	}

	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		K_act(NUM_ACTUATORS + i, NUM_STATES_PER_ACTUATOR*i + 1) = K_spring[i];				
	}

}


// Set the zero spring force joint position
void HopperActuatorModel::set_zero_pos_q_o(sejong::Vector &q_o_in){
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		q_o[i] = q_o_in[i];
	}	
}

// -----------------------------------------------------------------------------
// Simple Relationship between actuator position and joint position
// (z - z_o) = r*(q - q_o) 
double HopperActuatorModel::get_joint_pos_q(const int &index, const double &z_act_pos){
	return (z_act_pos - z_o[index])/r_arm[index] + q_o[index];
}

double HopperActuatorModel::get_joint_vel_qdot(const int &index, const double z_act_pos, const double &z_act_vel){
	return 	getJacobian_dqdz(index, z_act_pos)*z_act_vel;	
}

double HopperActuatorModel::get_act_pos_z(const int &index, const double &q_act_pos){
	std::cout << "qo = " << q_o[index] << std::endl;
	std::cout << "q_act_pos = " << q_act_pos << std::endl;
	return z_o[index] + r_arm[index]*(q_act_pos - q_o[index]);
}

double HopperActuatorModel::getJacobian_dqdz(const int &index, const double &q_act_pos){
	return 1.0/r_arm[index];
}

// dz/dq = r 
double HopperActuatorModel::getJacobian_dzdq(const int &index, const double &z_act_pos){
	return r_arm[index];
}
// -----------------------------------------------------------------------------

void HopperActuatorModel::getFullJacobian_dzdq(const sejong::Vector &z_pos, sejong::Matrix &L){
	L.resize(NUM_ACTUATORS, NUM_ACTUATORS);
	L.setIdentity();
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		L(i,i) = getJacobian_dzdq(i, z_pos[i]); 
	}
}

void HopperActuatorModel::getFullJacobian_dqdz(const sejong::Vector &q_pos, sejong::Matrix &J){
	J.resize(NUM_ACTUATORS, NUM_ACTUATORS);
	J.setIdentity();
	for(size_t i = 0; i < NUM_ACTUATORS; i++){
		J(i,i) = getJacobian_dqdz(i, q_pos[i]); 
	}
}


void HopperActuatorModel::getFull_joint_pos_q(const sejong::Vector &z_in, sejong::Vector &q_out){
	q_out.resize(NUM_ACTUATORS);
	q_out.setZero();

	double z_val = 0.0;
	for(int i = 0; i < NUM_ACTUATORS; i++){
		z_val = z_in[i];
		q_out[i] = get_joint_pos_q(i, z_val);
	}	
}

void HopperActuatorModel::getFull_joint_vel_qdot(const sejong::Vector &z_in, const sejong::Vector &zdot_in, sejong::Vector &qdot_out){
	qdot_out.resize(NUM_ACTUATORS);
	qdot_out.setZero();
	for(int i = 0; i < NUM_ACTUATORS; i++){
		qdot_out[i] = get_joint_vel_qdot(i, z_in[i], zdot_in[i]);
	}	
}


void HopperActuatorModel::getFull_act_pos_z(const sejong::Vector &q_in, sejong::Vector &z_out){
	z_out.resize(NUM_ACTUATORS);
	z_out.setZero();

	double q_val = 0.0;
	for(int i = 0; i < NUM_ACTUATORS; i++){
		q_val = q_in[i];
		z_out[i] = get_act_pos_z(i, q_val);
	}	
}

void HopperActuatorModel::getKm_Matrix(sejong::Matrix &Km_act){
	Km_act = sejong::Matrix::Identity(NUM_ACTUATORS, NUM_ACTUATORS);
	for(int i = 0; i < NUM_ACTUATORS; i++){
		Km_act(i, i) = K_m[i];
	}	

}
