#ifndef HOPPER_FOOT_CONTACT_H
#define HOPPER_FOOT_CONTACT_H

#include <optimization/contacts/contact_main.hpp>
#include "HopperModel.hpp"
#include "Hopper_Definition.h"

class Hopper_Foot_Contact: public Contact{
public:
	Hopper_Foot_Contact();
	~Hopper_Foot_Contact();	
	HopperModel* robot_model;

	void getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt);
    void getContactJacobianDotQdot(const sejong::Vector &q_state, 
  								   const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot);
};

#endif