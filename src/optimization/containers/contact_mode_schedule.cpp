#include <optimization/containers/contact_mode_schedule.hpp>
#include <iostream>

Contact_Mode_Schedule::Contact_Mode_Schedule(){
	std::cout << "[Contact Mode Schedule] Initialized" << std::endl;
}
Contact_Mode_Schedule::~Contact_Mode_Schedule(){
	std::cout << "[Contact Mode Schedule] Destroyed" << std::endl;
}

void Contact_Mode_Schedule::add_new_mode(const int &mode_start_time, const int &mode_final_time, std::vector<int> &active_contacts_indices){
	std::cout << "[Contact_Mode_Schedule] Added new mode " << num_modes << " (start_time, final_time) = (" <<  mode_start_time << "," << mode_final_time << "). Indices of active contacts = ";
	
	if (active_contacts_indices.size() == 0){
		std::cout << "No contacts active" << std::endl;
	}else{
		for(size_t i = 0; i < active_contacts_indices.size(); i++){
			std::cout << active_contacts_indices[i] << ", ";
		}
	}
	std::cout << " " << std::endl;

	num_modes++;
	mode_start_times.push_back(mode_start_time);
	mode_final_times.push_back(mode_final_time);
	mode_to_active_contacts.push_back(active_contacts_indices);	
}

int Contact_Mode_Schedule::get_num_modes(){
	return num_modes;
}

int Contact_Mode_Schedule::get_mode(const int &knotpoint){
	for (size_t m = 0; m < num_modes; m++){
		if ( (mode_start_times[m] <= knotpoint) && (mode_final_times[m] <= knotpoint)){
			return m;
		}
	}
}

void Contact_Mode_Schedule::get_active_contacts(const int &knotpoint, std::vector<int> &active_contacts_indices_out){
	active_contacts_indices_out = mode_to_active_contacts[get_mode(knotpoint)];
}
