#include <Utils/utilities.hpp>
#include <hdt/containers/opt_variable.hpp>
#include <hdt/containers/opt_variable_manager.hpp>

#include <iostream>

int main(int argc, char **argv){
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


	return 0;
}