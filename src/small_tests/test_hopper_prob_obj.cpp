#include <optimization/optimization_problems/2d_hopper/hopper_stand_opt_problem.hpp>

#include <Utils/utilities.hpp>

int main(int argc, char **argv){
	std::cout << "[Main] Testing Hopper Jump Problem Object" << std::endl;
	Hopper_Stand_Opt hopper_opt_prob;

	std::vector<double> F_vec_test;
	hopper_opt_prob.compute_F_constraints(F_vec_test);

	return 0;
}