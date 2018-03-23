#include <Utils/utilities.hpp>
#include <optimization/containers/opt_variable.hpp>
#include <optimization/containers/opt_variable_manager.hpp>
#include <optimization/containers/constraint_list.hpp>
#include <optimization/containers/contact_list.hpp>

#include <optimization/hard_constraints/constraint_main.hpp>

#include <iostream>

int main(int argc, char **argv){
	std::cout << "[Main] Testing Opt Manager Container" << std::endl;
	Opt_Variable* opt_var = new Opt_Variable();
	std::cout << "Lower Bound:" << opt_var->l_bound << std::endl;

	Opt_Variable_Manager opt_var_manager;
	opt_var_manager.append_variable(opt_var);
	opt_var_manager.append_variable(new Opt_Variable("Actuator Z", VAR_TYPE_Z, 0, 100, 0, 200));		
	opt_var_manager.append_variable(new Opt_Variable("Actuator Z", VAR_TYPE_Z, 0, 50, 0, 200));			

	sejong::Vector z_state;
	opt_var_manager.get_z_states(0, z_state);
	std::cout << "Size of Z:" << z_state.size() << " value: " << z_state[0] << std::endl;
	for(size_t i = 0; i < z_state.size(); i++){
		std::cout << " z[" << i << "] = " << z_state[i] << std::endl;		
	}

	std::cout << " " << std::endl;
	std::cout << "[Main] Testing Constraint List Container" << std::endl;
	Constraint_Function* constraint_1 = new Constraint_Function();
	Constraint_Function* constraint_2 = new Constraint_Function();

	constraint_1->constraint_name = "constraint 1";
	constraint_2->constraint_name = "constraint 2";

	Constraint_List constraint_list;
	constraint_list.append_constraint(constraint_1);
	constraint_list.append_constraint(constraint_2);

	std::cout << "We expect two constraints" << std::endl;
	for(size_t i = 0; i < constraint_list.get_size(); i++){
		std::cout << constraint_list.get_constraint(i)->constraint_name << std::endl;		
	}		

	std::cout << " " << std::endl;
	std::cout << "[Main] Testing Contact List Container" << std::endl;

	Contact* contact_1 = new Contact();
	Contact* contact_2 = new Contact();
	contact_1->contact_name = "contact 1";
	contact_2->contact_name = "contact 2";

	Contact_List contact_list;
	contact_list.append_contact(contact_1);
	contact_list.append_contact(contact_2);	

	for(size_t i = 0; i < contact_list.get_size(); i++){
		std::cout << contact_list.get_contact(i)->contact_name << std::endl;		
	}		


	std::cout << " " << std::endl;

	return 0;
}