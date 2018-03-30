#include <optimization/contacts/2d_draco/draco_toe_contact.hpp>
#include <iostream>

// Define LeftFoot Contact ---------------------------------------------------------------
Draco_Toe_Contact::Draco_Toe_Contact(){
    std::cout << "[Draco Toe Contact] Constructed" << std::endl;
	robot_model = DracoModel::GetDracoModel();
	contact_dim = 2;
    contact_name = "Draco Toe Contact";
    contact_link_id = SJLinkID::LK_FootToe;
    std::cout << "[Draco Toe Contact] Contact Name: " << contact_name << ", Link id: " << contact_link_id << std::endl;
}
Draco_Toe_Contact::~Draco_Toe_Contact(){}

void Draco_Toe_Contact::getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){
	sejong::Matrix Jtmp;
    robot_model->getFullJacobian(q_state, contact_link_id, Jtmp);
    //Jt = Jtmp;
    Jt = Jtmp.block(0, 0, 2, NUM_QDOT);    
}

void Draco_Toe_Contact::getContactJacobianDotQdot(const sejong::Vector &q_state, 
  							  			  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){
	sejong::Matrix Jdot_tmp;    
    robot_model->getFullJacobianDot(q_state, qdot_state, contact_link_id, Jdot_tmp);
    //sejong::Matrix Jdot_task = Jdot_tmp;
    sejong::Matrix Jdot_task = Jdot_tmp.block(0, 0, 2, NUM_QDOT);    

	JtDotQdot = Jdot_task*qdot_state;    
}

void Draco_Toe_Contact::signed_distance_to_contact(const sejong::Vector &q_state, double &distance){
    sejong::Vect3 pos_vec;
    robot_model->getPosition(q_state, contact_link_id, pos_vec) ;   

    // floor contact with the ground
    distance = pos_vec[1];
}

