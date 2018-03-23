#include <iostream>
#include <Utils/utilities.hpp>
#include <optimization/contacts/2d_hopper/hopper_foot_contact.hpp>

int main(int argc, char **argv){
	std::cout << "[Main] Testing Hopper Foot Contact" << std::endl;
	Hopper_Foot_Contact foot_contact;

	sejong::Vector q_state = sejong::Vector::Zero(2);
	sejong::Matrix Jc;
	foot_contact.getContactJacobian(q_state, Jc);
	sejong::pretty_print(Jc, std::cout, "Contact Jacobian");

	std::cout << " " << std::endl;

	return 0;
}