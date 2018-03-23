#include <optimization/snopt_wrapper.hpp>
#include <string>

namespace snopt_wrapper{

  Optimization_Problem_Main* ptr_optimization_problem;

  void wbt_F(int    *Status, int *n,    double x[],
     int    *needF,  int *lenF,  double F[],
     int    *needG,  int *lenG,  double G[],
     char      *cu,  int *lencu,
     int    iu[],    int *leniu,
     double ru[],    int *lenru){

		std::vector<double> x_vars;
		std::vector<double> F_eval;
		std::vector<double> G_eval;				
		std::vector<int> iGfun_eval;
		std::vector<int> jGvar_eval;	
		int neG_eval = 0;							

	  	// Update optimization x_vars
		for (size_t i = 0; i < *n; i++){
			x_vars.push_back(x[i]);
		}

		ptr_optimization_problem->update_opt_vars(x_vars);
		// Get F evaluations
		if ((*needF) > 0){
			ptr_optimization_problem->compute_F(F_eval);
		}

		// Populate F
		for (size_t i = 0; i < F_eval.size(); i++){
			F[i] = F_eval[i];
		}

    }    


  void solve_problem_no_gradients(Optimization_Problem_Main* input_ptr_optimization_problem){
  	std::cout << "[SNOPT Wrapper] Initializing Optimization Problem" << std::endl;
	ptr_optimization_problem = input_ptr_optimization_problem;
  	std::cout << "[SNOPT Wrapper] Problem Name: " << ptr_optimization_problem->problem_name << std::endl;


	// Prepare Variable Containers
	std::vector<double> x_vars;
	std::vector<double> x_vars_low;
	std::vector<double> x_vars_upp;	

	std::vector<double> F_eval;
	std::vector<double> F_eval_low;
	std::vector<double> F_eval_upp;

	std::vector<double> G_eval;
	std::vector<int> iGfun_eval;
	std::vector<int> jGvar_eval;
	int neG_eval = 0;

	std::vector<double> A_eval;
	std::vector<int> iAfun_eval;
	std::vector<int> jAvar_eval;
	int neA_eval = 0;


	ptr_optimization_problem->get_init_opt_vars(x_vars);
	std::cout << "[SNOPT Wrapper] Initialized Initial Value of Optimization Variables" << std::endl;
	std::cout << "[SNOPT Wrapper]                    Number of Optimization Variables: " << x_vars.size() << std::endl;	

	ptr_optimization_problem->get_opt_vars_bounds(x_vars_low, x_vars_upp);
	std::cout << "[SNOPT Wrapper] Initialized Bounds of Optimization Variables" << std::endl;
	std::cout << "[SNOPT Wrapper]  						  Num of Lower Bounds: " << x_vars_low.size() << std::endl;	
	std::cout << "[SNOPT Wrapper]  						  Num of Upper Bounds: " << x_vars_upp.size() << std::endl;		

	ptr_optimization_problem->get_F_bounds(F_eval_low, F_eval_upp);
	std::cout << "[SNOPT Wrapper] Initialized Bounds of Functions" << std::endl;
	std::cout << "[SNOPT Wrapper]  			  Num of Lower Bounds: " << F_eval_low.size() << std::endl;	
	std::cout << "[SNOPT Wrapper]  			  Num of Upper Bounds: " << F_eval_upp.size() << std::endl;		

	if (F_eval_low.size() == F_eval_upp.size()){
		std::cout << "[SNOPT_Wrapper] There are " << F_eval_low.size() << " problem functions" << std::endl; 
	}else{
		std::cout << "[SNOPT Wrapper] Error! Bounds are not equal" << std::endl;
		throw;
	}

	// Compute F initially
	ptr_optimization_problem->compute_F(F_eval);
	std::cout << "[SNOPT_Wrapper] F_eval has size " << F_eval.size() << std::endl; 


	int Cold  = 0; int Basis = 1; int Warm = 2;
	int start_condition = Basis;

	// Get Sizes
	// int n = 2;
	// int nF = 2;
	int n = x_vars.size(); // Size of optimization variables
	int nF = F_eval_low.size(); // Size of Constraints


	// Initialize Variables
	double *x      = new double[n];
	double *xlow   = new double[n];
	double *xupp   = new double[n];
	double *xmul   = new double[n];
	int    *xstate = new    int[n];

	double *F      = new double[nF];
	double *Flow   = new double[nF];
	double *Fupp   = new double[nF];
	double *Fmul   = new double[nF];
	int    *Fstate = new int[nF];

	// Get Objective Row
	int ObjRow = 0;
	ptr_optimization_problem->get_F_obj_Row(ObjRow);
	double ObjAdd = 0.0;

	int nS = 0, nInf = 0;

	int jx1, jx2, ju, ode1, ode2, Obj;
	double sInf;
	double inf = OPT_INFINITY;	


	// Populate x_vars
	for(size_t i = 0; i < x_vars.size(); i++){
		x[i] = x_vars[i];		
		xstate[i] = x[i];
		xmul[i] = 0.0;
		//std::cout << "x[" << i << "] = " << x[i] << std::endl;
	}
	// Populate F bounds
	for (size_t i = 0; i < F_eval.size(); i++){
		F[i] = F_eval[i];
		Fstate[i] =  F_eval[i];
		Fmul[i] = 0.0;
	}
	// Populate x bounds
	for(size_t i = 0; i < x_vars_low.size(); i++){
		xlow[i] = x_vars_low[i];		
	}
	for(size_t i = 0; i < x_vars_upp.size(); i++){
		xupp[i] = x_vars_upp[i];		
	}
	// Populate F bounds
	for (size_t i = 0; i < F_eval_low.size(); i++){
		Flow[i] = F_eval_low[i];
	}
	for (size_t i = 0; i < F_eval_upp.size(); i++){
		Fupp[i] = F_eval_upp[i];
	}	

	int *iAfun_test;
	int *jAvar_test;
	double *A_test;
	int    neA_test;

	int *iGfun_test;
	int *jGvar_test;
	double *G_test;
	int    neG_test;



	snoptProblemA snopt_optimization_problem;
	snopt_optimization_problem.initialize("", 1);  // no print file, summary on

	
   	snopt_optimization_problem.setPrintFile("snopt_problem.out"); 
	snopt_optimization_problem.setIntParameter("Derivative option", 0);
	snopt_optimization_problem.setIntParameter("Verify level ", 3);	
	//whole_body_trajectory_problem.setSpecsFile("small_jump.spc");
	//whole_body_trajectory_problem.setSpecsFile("com_jump.spc");

	// whole_body_trajectory_problem.computeJac(nF, n, snopt_wrapper::wbt_FG, x, xlow, xupp,
	// 	  iAfun_test, jAvar_test, A_test, neA_test,
	// 	  iGfun_test, jGvar_test, neG_test);

  	std::cout << "[SNOPT Wrapper] Solving Problem with no Gradients" << std::endl;

  	snopt_optimization_problem.solve(start_condition, nF, n, ObjAdd, ObjRow, snopt_wrapper::wbt_F,
			       xlow, xupp, Flow, Fupp,
     			  x, xstate, xmul, F, Fstate, Fmul,
     			  nS, nInf, sInf);


	// for (size_t i = 0; i < n; i++){
	// 	std::cout << "x[" << i << "] = " << x[i] << std::endl;
	// }


	// for (size_t i = 0; i < nF; i++){
	// 	std::cout << "F[" << i << "] = " << F[i] << std::endl;
	// }	




	delete []x;      delete []xlow;   delete []xupp;
	delete []xmul;   delete []xstate;

	delete []F;      delete []Flow;   delete []Fupp;
	delete []Fmul;   delete []Fstate;


  }




}