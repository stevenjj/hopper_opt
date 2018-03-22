#ifndef OPT_VARS_H
#define OPT_VARS_H

#include <string>
#include <hdt/optimization_constants.hpp>

class Opt_Variable{
public:
	int type = VAR_TYPE_NONE;
	std::string name = "undefined_name";
	double 		value = 0.0;
	int			knotpoint = -1;
	int 		index = -1;

	double l_bound = -OPT_INFINITY;
	double u_bound = OPT_INFINITY;	

	// Constructors
	Opt_Variable();
	Opt_Variable(std::string _name, double _value);
	Opt_Variable(std::string _name, double _value, double _l_bound, double _u_bound);	
	Opt_Variable(std::string _name, int _knotpoint, double _value, double _l_bound, double _u_bound);	
	Opt_Variable(std::string _name, int _type, int _knotpoint, double _value, double _l_bound, double _u_bound);		

	// Destructors
	~Opt_Variable();	
};

#endif