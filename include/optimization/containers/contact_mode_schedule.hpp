#ifndef CONTACT_MODE_SCHEDULE_CONTAINER_H
#define CONTACT_MODE_SCHEDULE_CONTAINER_H

#include <Utils/wrap_eigen.hpp>
#include <vector>
#include <string>

class Contact_Mode_Schedule{
public:
	Contact_Mode_Schedule();
	~Contact_Mode_Schedule();	

	void add_new_mode(const int &mode_start_time, const int &mode_final_time, std::vector<int> &active_contacts_indices);	
	void get_active_contacts(const int &mode, std::vector<int> &active_contacts_indices_out);
	int get_mode(const int &knotpoint);
	int get_num_modes();

private:
	int num_modes = 0;
	std::vector< std::vector<int> > mode_to_active_contacts;
	std::vector<int> mode_start_times;
	std::vector<int> mode_final_times;	

};

#endif