#ifndef SNOPT_WRAPPER_H
#define SNOPT_WRAPPER_H

#include <stdio.h>
#include <string.h>
#include <map>

#include <iostream>
#include <math.h>

#include <Optimizer/snopt/include/snoptProblem.hpp>
#include <optimization/optimization_problems/opt_problem_main.hpp>

namespace snopt_wrapper{
 
  void wbt_F(int    *Status, int *n,    double x[],
     int    *needF,  int *lenF,  double F[],
     int    *needG,  int *lenG,  double G[],
     char      *cu,  int *lencu,
     int    iu[],    int *leniu,
     double ru[],    int *lenru);

  void solve_problem_no_gradients(Optimization_Problem_Main* input_ptr_optimization_problem);
}


#endif