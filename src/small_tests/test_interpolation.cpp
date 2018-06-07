#include <iostream>
#include <Utils/utilities.hpp>
#include <Utils/minjerk_one_dim.hpp>

int main(int argc, char **argv){
	std::cout << "hello world" << std::endl;

	MinJerk_OneDimension minjerk_single;

	// Set Minimum Jerk Parameters
	sejong::Vect3 init_params; init_params.setZero(); // initial pos, vel, acc
	sejong::Vect3 final_params; final_params.setZero();// final pos, vel, acc
	final_params[0] = 1.0;
	double time_start = 0.0;
	double time_end = 1.0;

	minjerk_single.setParams(init_params, final_params, time_start, time_end);
	minjerk_single.printParameters();


}