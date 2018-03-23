#include <optimization/containers/opt_variable.hpp>

Opt_Variable::Opt_Variable(){}
Opt_Variable::Opt_Variable(std::string _name, double _value){
	name = _name;
	value = _value;	
}

Opt_Variable::Opt_Variable(std::string _name, double _value, double _l_bound, double _u_bound){
	name = _name;
	value = _value;	
	l_bound = _l_bound;
	u_bound = _u_bound;	
}

Opt_Variable::Opt_Variable(std::string _name, int _knotpoint, double _value, double _l_bound, double _u_bound){
	name = _name;
	value = _value;	
	knotpoint = _knotpoint;
	l_bound = _l_bound;
	u_bound = _u_bound;	

}

Opt_Variable::Opt_Variable(std::string _name, int _type, int _knotpoint, double _value, double _l_bound, double _u_bound){
	name = _name;
	type = _type;
	value = _value;	
	knotpoint = _knotpoint;
	l_bound = _l_bound;
	u_bound = _u_bound;		
}

Opt_Variable::~Opt_Variable(){}