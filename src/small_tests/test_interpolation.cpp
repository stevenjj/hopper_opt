#include <iostream>
#include <Utils/utilities.hpp>
#include <Utils/minjerk_one_dim.hpp>
#include <Utils/minjerk_one_dim.hpp>
#include <Utils/cubic_interpolate_one_dim.hpp>

int main(int argc, char **argv){
	// Set Minimum Jerk Parameters
	MinJerk_OneDimension minjerk_single;

	sejong::Vect3 init_params; init_params.setZero(); // initial pos, vel, acc
	sejong::Vect3 final_params; final_params.setZero();// final pos, vel, acc
	final_params[0] = 1.0;
	double time_start = 0.0;
	double time_end = 1.0;

	minjerk_single.setParams(init_params, final_params, time_start, time_end);
	minjerk_single.printParameters();
	printf("\n");


	//Set Cubic Interpolation Parameters
	CubicInterpolate_OneDimension cubic_single;

	sejong::Vect2 cubic_init_params; init_params.setZero(); // initial pos, vel
	sejong::Vect2 cubic_final_params; cubic_final_params.setZero();// final pos, vel	
	cubic_final_params[0] = 1.0;

	cubic_single.setParams(cubic_init_params, cubic_final_params, time_start, time_end);
	cubic_single.printParameters();

}