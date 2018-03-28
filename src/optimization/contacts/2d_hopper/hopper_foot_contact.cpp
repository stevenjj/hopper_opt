#include <optimization/contacts/2d_hopper/hopper_foot_contact.hpp>
#include <Utils/utilities.hpp>
#include <iostream>

// Define LeftFoot Contact ---------------------------------------------------------------
Hopper_Foot_Contact::Hopper_Foot_Contact(){
    std::cout << "[Hopper_Foot_Contact] Constructed" << std::endl;
	robot_model = HopperModel::GetRobotModel();
	contact_dim = 1;
    contact_name = "Hopper_Foot_Contact";
    contact_link_id = SJ_Hopper_LinkID::LK_foot;
    std::cout << "[Hopper_Foot_Contact] Contact Name: " << contact_name << ", Link id: " << contact_link_id << std::endl;
}
Hopper_Foot_Contact::~Hopper_Foot_Contact(){}

void Hopper_Foot_Contact::getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
    robot_model->getFullJacobian(q_state, contact_link_id, Jt);
}

void Hopper_Foot_Contact::getContactJacobianDotQdot(const sejong::Vector &q_state, 
  							  			  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){
}

void Hopper_Foot_Contact::signed_distance_to_contact(const sejong::Vector &q_state, double &distance){
	sejong::Vector pos_vec;
    robot_model->getPosition(q_state, contact_link_id, pos_vec) ;	

    // floor contact with the ground
	distance = pos_vec[0];
}
