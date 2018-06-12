#include <Utils/utilities.hpp>

#include "ValkyrieRobotModel.hpp"
#include "valkyrie_definition.h"
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv){

	std::cout << "[Main]Testing Valkyrie Model" << std::endl;
	std::cout << "[Main] Constructing model ..." << std::endl;
	ValkyrieRobotModel*	robot_model;
	robot_model = ValkyrieRobotModel::GetValkyrieRobotModel();	

	sejong::Vector robot_q_init = sejong::Vector::Zero(NUM_Q);
	sejong::Vector robot_qdot_init = sejong::Vector::Zero(NUM_QDOT);

	// Set Virtual Joints
	// x_pos
	robot_q_init[0] = 0.0;
	// y_pos
	robot_q_init[1] = 0.0;
	// z_pos
	robot_q_init[2] = 1.14; //1.135; //1.131; 
	robot_q_init[NUM_QDOT] = 1.0; // Pelvis Quaternion w = 1.0


	// Initialize Joints
	robot_q_init[NUM_VIRTUAL + SJJointID::leftHipPitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftHipPitch"  )->second]->m_State.m_rValue[0] = -0.3;
	robot_q_init[NUM_VIRTUAL + SJJointID::rightHipPitch] = -0.3;  //r_joint_[r_joint_idx_map_.find("rightHipPitch" )->second]->m_State.m_rValue[0] = -0.3;
	robot_q_init[NUM_VIRTUAL + SJJointID::leftKneePitch] = 0.6;  //r_joint_[r_joint_idx_map_.find("leftKneePitch" )->second]->m_State.m_rValue[0] = 0.6;
	robot_q_init[NUM_VIRTUAL + SJJointID::rightKneePitch] = 0.6;//r_joint_[r_joint_idx_map_.find("rightKneePitch")->second]->m_State.m_rValue[0] = 0.6;
	robot_q_init[NUM_VIRTUAL + SJJointID::leftAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("leftAnklePitch")->second]->m_State.m_rValue[0] = -0.3;
	robot_q_init[NUM_VIRTUAL + SJJointID::rightAnklePitch] = -0.3; //r_joint_[r_joint_idx_map_.find("rightAnklePitch")->second]->m_State.m_rValue[0] = -0.3;

	robot_q_init[NUM_VIRTUAL + SJJointID::rightShoulderPitch] = 0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
	robot_q_init[NUM_VIRTUAL + SJJointID::rightShoulderRoll] = 1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
	robot_q_init[NUM_VIRTUAL + SJJointID::rightElbowPitch] = 0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
	robot_q_init[NUM_VIRTUAL + SJJointID::rightForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;

	robot_q_init[NUM_VIRTUAL + SJJointID::leftShoulderPitch] = -0.2; //r_joint_[r_joint_idx_map_.find("rightShoulderPitch")->second]->m_State.m_rValue[0] = 0.2;
	robot_q_init[NUM_VIRTUAL + SJJointID::leftShoulderRoll] = -1.1;  //r_joint_[r_joint_idx_map_.find("rightShoulderRoll" )->second]->m_State.m_rValue[0] = 1.1;
	robot_q_init[NUM_VIRTUAL + SJJointID::leftElbowPitch] = -0.4;//0.4;  //r_joint_[r_joint_idx_map_.find("rightElbowPitch"   )->second]->m_State.m_rValue[0] = 0.4;
	robot_q_init[NUM_VIRTUAL + SJJointID::leftForearmYaw] = 1.5;  //r_joint_[r_joint_idx_map_.find("rightForearmYaw" )->second]->m_State.m_rValue[0] = 1.5;	

	sejong::pretty_print(robot_q_init, std::cout, "Val q_init");
	sejong::pretty_print(robot_q_init, std::cout, "Val qdot_init");	

	std::cout << "[Main] Updating model ..." << std::endl;
	robot_model->UpdateModel(robot_q_init, robot_qdot_init);

	std::cout << "[Main] The gravity vector" << std::endl;
	sejong::Vector grav = sejong::Vector::Zero(NUM_QDOT);
	robot_model->getGravity(grav);
	sejong::pretty_print(grav, std::cout, "Val grav");	


	YAML::Node config;
	config  = YAML::LoadFile(THIS_PACKAGE_PATH"vec_sample.yaml");

	std::string key = "q_vec";
	std::vector<double> vec_value;
	vec_value = config[key].as<std::vector<double> >();

	std::cout << "vec_value contents" << std::endl;
	for(int i = 0; i < vec_value.size(); i++){
		std::cout << "i:" << i << ", has value " << vec_value[i] << std::endl;
	}

}