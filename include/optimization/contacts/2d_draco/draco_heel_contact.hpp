#ifndef DRACO_CONTACT_HEEL_H
#define DRACO_CONTACT_HEEL_H

#include <optimization/contacts/contact_main.hpp>
#include "DracoModel.hpp"
#include "DracoP1Rot_Definition.h"

class Draco_Heel_Contact: public Contact{
public:
	Draco_Heel_Contact();
	~Draco_Heel_Contact();	
	DracoModel* robot_model;

	void getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getContactJacobianDotQdot(const sejong::Vector &q_state, 
  								   const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);
    void signed_distance_to_contact(const sejong::Vector &q_state, double &distance);
};

#endif