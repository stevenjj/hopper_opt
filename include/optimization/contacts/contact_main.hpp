#ifndef CONTACT_PARENT_H
#define CONTACT_PARENT_H
#include <Utils/wrap_eigen.hpp>
#include <string>

class Contact{
public:
  Contact(){}
  virtual ~Contact(){}
  virtual void getContactJacobian(const sejong::Vector &q_state, sejong::Matrix & Jt){}
  virtual void getContactJacobianDotQdot(const sejong::Vector &q_state, 
  							  		  const sejong::Vector &qdot_state, sejong::Vector & JtDotQdot){}
  std::string contact_name = "Undefined Contact";
  int contact_dim;
  int contact_link_id;

  virtual void signed_distance_to_contact(const sejong::Vector &q_state, double &distance){}
  sejong::Vector contact_pos; // contact position
};

#endif