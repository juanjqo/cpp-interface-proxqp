/**
(C) Copyright 2023 DQ Robotics Developers
This file is part of DQ Robotics.
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:

- Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
        - Adapted the file DQ_QPOASESSolver.h implemented by Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
          in (https://github.com/dqrobotics/cpp-interface-qpoases) to use the proxqp solver
          (https://github.com/Simple-Robotics/proxsuite)
*/

#pragma once
#include <dqrobotics/solvers/DQ_QuadraticProgrammingSolver.h>
#include <proxsuite/helpers/optional.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <memory>


using namespace Eigen;

namespace DQ_robotics
{
class DQ_PROXQPSolver: public DQ_QuadraticProgrammingSolver
{

protected:

    std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;
    bool SOLVE_FIRST_TIME_;
    int EQUALITY_CONSTRAINT_SIZE_;
    int INEQUALITY_CONSTRAINT_SIZE_;
    double INFINITY_ = 1e30;
    VectorXd l_;

    double rho_ = 1.e-7;   // The minimal value it can take is minimum: 1.e-7. By default its value is set to 1.e-6.
    double mu_eq_ = 1.e-9; // The minimal value it can take is 1.e-9. By default its value is set to 1.e-3.
    double mu_in_ = 1.e-9;  // The minimal value it can take is 1.e-9. By default its value is set to 1.e-1.
    double mu_min_eq_ = DQ_threshold; // Minimal authorized value for mu_eq. 
    double mu_min_in_ = DQ_threshold; // Minimal authorized value for mu_in. 
    double eps_abs_ = 1.e-5; //Asbolute stopping criterion of the solver. Default value = 1.E-5.
    bool verbose_ = false;  // 	If set to true, the solver prints information at each loop. 
    bool compute_preconditioner_ = false; //If set to true, the preconditioner will be derived with the init method.
    bool compute_timings_ = false; 

    double run_time_ = 0.0;

    /**
     * @brief _initialize_problem
     * @param PROBLEM_SIZE
     * @param EQUALITY_CONSTRAINT_SIZE
     * @param INEQUALITY_CONSTRAINT_SIZE
     */
    void _initialize_problem(const int& PROBLEM_SIZE, const int& EQUALITY_CONSTRAINT_SIZE, const int& INEQUALITY_CONSTRAINT_SIZE)
    {
        EQUALITY_CONSTRAINT_SIZE_ = EQUALITY_CONSTRAINT_SIZE;
        INEQUALITY_CONSTRAINT_SIZE_ = INEQUALITY_CONSTRAINT_SIZE;
        qp_ = std::unique_ptr<proxsuite::proxqp::dense::QP<double>>(new proxsuite::proxqp::dense::QP<double>(PROBLEM_SIZE, EQUALITY_CONSTRAINT_SIZE, INEQUALITY_CONSTRAINT_SIZE));
        //qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(PROBLEM_SIZE, EQUALITY_CONSTRAINT_SIZE, INEQUALITY_CONSTRAINT_SIZE); For >= C++14
        /**
         * The solver expects constraints in the form
         *     l <= Ax <= b.
         * Therefore, we set l = [-INFINITY] to implement the constraints
         * in the form:
         *      Ax <= b.
         */
        l_ = -1*VectorXd::Ones(INEQUALITY_CONSTRAINT_SIZE)*INFINITY_;
    }


    /**
     * @brief Solves the following quadratic program
     *
     *   min(x)  0.5*x'Hx + f'x
     *   s.t.    Ax <= b
     *           Aeqx = beq.
     *
     *  PROX-QP: Yet another Quadratic Programming Solver for Robotics and beyond, A. Bambade, S. El-Kazdadi, A. Taylor and J. Carpentier.
     *  Robotics: Science and Systems. 2022.
     *
     * @param H the n x n matrix of the quadratic coeficitients of the decision variables.
     * @param f the n x 1 vector of the linear coeficients of the decision variables.
     * @param Aeq the m x n matrix of equality constraints.
     * @param beq the m x 1 value for the inequality constraints.
     * @param A the m x n matrix of inequality constraints.
     * @param b the m x 1 value for the inequality constraints.
     *
     * The following three parameter description were copy from https://simple-robotics.github.io/proxsuite/md_doc_2_PROXQP_API_2_ProxQP_api.html
     * @param compute_preconditioner a boolean parameter for executing or not the preconditioner. The preconditioner is an algorithm used
     *        (Ruiz equilibrator) for reducing the ill-conditioning of the QP problem, and hence speeding-up the solver and increasing its accuracy.
     *        It consists mostly of an heuristic involving linear scalings. Note that for very ill-conditioned QP problem,
     *        when one asks for a very accurate solution, the unscaling procedure can become less precise. By default its value is set to true.
     * @param rho the proximal step size wrt primal variable. Reducing its value speed-ups convergence wrt primal variable
     *        (but increases as well ill-conditioning of sub-problems to solve). The minimal value it can take is 1.e-7.
     *        By default its value is set to 1.e-6.
     * @param mu_eq the proximal step size wrt equality constrained multiplier. Reducing its value speed-ups convergence wrt equality constrained
     *        variable (but increases as well ill-conditioning of sub-problems to solve). The minimal value it can take is 1.e-9.
     *        By default its value is set to 1.e-3.
     * @param mu_in the proximal step size wrt inequality constrained multiplier. Reducing its value speed-ups convergence wrt inequality constrained
     *        variable (but increases as well ill-conditioning of sub-problems to solve). The minimal value it can take is 1.e-9.
     *        By default its value is set to 1.e-1.
     * @return the optimal x
     */
    VectorXd _solve_prox_quadratic_program(const MatrixXd& H, const VectorXd& f, const MatrixXd& Aeq, const VectorXd& beq,
                                           const MatrixXd& A, const VectorXd&  b, const bool& compute_preconditioner,
                                           const double& rho, const double& mu_eq, const double& mu_in)
    {
        const int PROBLEM_SIZE = H.rows();
        const int INEQUALITY_CONSTRAINT_SIZE = b.size();
        const int EQUALITY_CONSTRAINT_SIZE = beq.size();


        MatrixXd H_sym(H.rows(), H.cols());
        H_sym =MatrixXd(H.selfadjointView<Upper>());

        if (SOLVE_FIRST_TIME_ == true)
        {
            _initialize_problem(PROBLEM_SIZE, EQUALITY_CONSTRAINT_SIZE, INEQUALITY_CONSTRAINT_SIZE);
            qp_->init(H_sym,f,Aeq,beq,A,l_,b, compute_preconditioner, rho, mu_eq, mu_in); // initialize the model
        }
        else if (EQUALITY_CONSTRAINT_SIZE_ != EQUALITY_CONSTRAINT_SIZE || INEQUALITY_CONSTRAINT_SIZE_ != INEQUALITY_CONSTRAINT_SIZE)
        {
            qp_->update(H_sym,f,Aeq,beq,A,l_,b, compute_preconditioner, rho, mu_eq, mu_in); // initialize the model
        }
        qp_->init(H_sym,f,Aeq,beq,A,l_,b, compute_preconditioner, rho, mu_eq, mu_in); // initialize the model
        qp_->settings.eps_abs = eps_abs_;     // set accuracy threshold
        qp_->settings.mu_min_eq = mu_min_eq_; // set minimal authorized value for mu_eq.
        qp_->settings.mu_min_eq = mu_min_in_; // set Minimal authorized value for mu_in.
        qp_->settings.verbose = verbose_;
        qp_->settings.compute_timings = compute_timings_;
        qp_->solve();
        qp_->settings.initial_guess =  proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
        SOLVE_FIRST_TIME_ = false;
        run_time_ = qp_->results.info.run_time;
        return qp_->results.x;
    }



public:
    DQ_PROXQPSolver():SOLVE_FIRST_TIME_(true)
    {

    }
   ~DQ_PROXQPSolver()=default;

    /**
     * @brief
     *   Solves the following quadratic program
     *   min(x)  0.5*x'Hx + f'x
     *   s.t.    Ax <= b
     *           Aeqx = beq.
     * Method signature is compatible with MATLAB's 'quadprog'.
     * @param H the n x n matrix of the quadratic coeficitients of the decision variables.
     * @param f the n x 1 vector of the linear coeficients of the decision variables.
     * @param A the m x n matrix of inequality constraints.
     * @param b the m x 1 value for the inequality constraints.
     * @param Aeq the m x n matrix of equality constraints.
     * @param beq the m x 1 value for the inequality constraints.
     * @return the optimal x
     */
    VectorXd solve_quadratic_program(const MatrixXd& H, const VectorXd& f,
                                     const MatrixXd& A, const VectorXd& b,
                                     const MatrixXd& Aeq, const VectorXd& beq) override
    {
        ///---------------------------------------------------------------------------------------------------
        /// Copy from https://github.com/dqrobotics/cpp-interface-qpoases

        ///Check sizes
        //Objective function
        if(H.rows()!=H.cols())
            throw std::runtime_error("DQ_PROXQPSolver::solve_quadratic_program(): H must be symmetric. H.rows()="+std::to_string(H.rows())+" but H.cols()="+std::to_string(H.cols())+".");
        if(f.size()!=H.rows())
            throw std::runtime_error("DQ_PROXQPSolver::solve_quadratic_program(): f must be compatible with H. H.rows()=H.cols()="+std::to_string(H.rows())+" but f.size()="+std::to_string(f.size())+".");

        //Inequality constraints
        if(b.size()!=A.rows())
            throw std::runtime_error("DQ_PROXQPSolver::solve_quadratic_program(): size of b="+std::to_string(b.size())+" should be compatible with rows of A="+std::to_string(A.rows())+".");

        //Equality constraints
        if(beq.size()!=Aeq.rows())
            throw std::runtime_error("DQ_PROXQPSolver::solve_quadratic_program(): size of beq="+std::to_string(beq.size())+" should be compatible with rows of Aeq="+std::to_string(Aeq.rows())+".");

        //std::cout<<"Sizes flag: "<<INEQUALITY_CONSTRAINT_SIZE<<" , "<<EQUALITY_CONSTRAINT_SIZE<<std::endl;
        ///---------------------------------------------------------------------------------------------------
        return _solve_prox_quadratic_program(H, f, Aeq, beq, A, b, compute_preconditioner_, rho_, mu_eq_, mu_in_);
        }

    /**
     * @brief Sets the proximal step size. 
     * 
     * @param rho the proximal step size wrt primal variable. Reducing its value speed-ups convergence wrt primal variable
     *        (but increases as well ill-conditioning of sub-problems to solve). The minimal value it can take is 1.e-7.
     *        By default its value is set to 1.e-6.
    */
    void set_proximal_step_size(const double& rho)
    {
        rho_ = rho;
    }  

    /**
     * @brief Sets the mu_eq proximal step size. 
     * 
     * @param mu_eq the proximal step size wrt equality constrained multiplier. Reducing its value speed-ups convergence wrt equality constrained
     *        variable (but increases as well ill-conditioning of sub-problems to solve). The minimal value it can take is 1.e-9.
     *        By default its value is set to 1.e-3.
    */  
    void set_equality_proximal_step_size(const double& mu_eq)
    {
        mu_eq_ = mu_eq;
    }

    /**
     * @brief Sets the mu_in proximal step size. 
     * 
     * @param mu_in the proximal step size wrt inequality constrained multiplier. Reducing its value speed-ups convergence wrt inequality constrained
     *        variable (but increases as well ill-conditioning of sub-problems to solve). The minimal value it can take is 1.e-9.
     *        By default its value is set to 1.e-1
    */  
    void set_inequality_proximal_step_size(const double& mu_in)
    {
        mu_in_ = mu_in;
    }

    /**
     * @brief Sets the absolute stopping criterion
     * 
     * @param eps_abs The asbolute stopping criterion of the solver. The default value is 1e-5;
    */
    void set_absolute_stopping_criterion(const double& eps_abs)
    {
        eps_abs_ = eps_abs;
    }

    /**
     * @brief Sets the verbose mode. If set to true, the solver prints information at each loop. 
     * 
     * @param verbose The verbose flag.
    */
    void set_verbose_mode(const bool& verbose)
    {
        verbose_ = verbose;
    }

    /**
     * @brief Sets the preconditioner. If set to true, the preconditioner will be derived with the init method.  
     * 
     * @param compute_preconditioner The preconditioner flag. 
    */
    void set_preconditioner(const bool& compute_preconditioner)
    {
        compute_preconditioner_ = compute_preconditioner;
    }

    /**
     * @brief Returns the measured solver runtime (setup + solve) used to compute the solution
     * 
     * @returns The measured runtime
    */
    double get_run_time()
    {
        if (compute_timings_ == false){
            std::cerr<<"You must enable the compute_timmings option to get the runtime. "<<std::endl;
        }
        return run_time_;
    }

    /**
     * @brief Sets the compute_timmings option of the solver.
     * 
     * @param flag.
    */
    void compute_timmings(const bool& flag)
    {
        compute_timings_ = flag;
    }
};
}


