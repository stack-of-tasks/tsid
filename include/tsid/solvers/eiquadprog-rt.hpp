//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_eiquadprog_rt_hpp__
#define __invdyn_eiquadprog_rt_hpp__

#include <Eigen/Dense>

#define OPTIMIZE_STEP_1_2         // compute s(x) = ci^T * x + ci0
#define OPTIMIZE_COMPUTE_D        // use noalias
#define OPTIMIZE_UPDATE_Z         // use noalias
#define OPTIMIZE_HESSIAN_INVERSE  // use solveInPlace
#define OPTIMIZE_UNCONSTR_MINIM

//#define USE_WARM_START
//#define PROFILE_EIQUADPROG
//#define DEBUG_STREAM(msg) std::cout<<msg;
#define DEBUG_STREAM(msg)

#ifdef PROFILE_EIQUADPROG
#define START_PROFILER_EIQUADPROG_RT(x) START_PROFILER(x)
#define STOP_PROFILER_EIQUADPROG_RT(x)  STOP_PROFILER(x)
#else
#define START_PROFILER_EIQUADPROG_RT(x)
#define STOP_PROFILER_EIQUADPROG_RT(x)
#endif

#define PROFILE_EIQUADPROG_CHOWLESKY_DECOMPOSITION "EIQUADPROG_RT Chowlesky dec"
#define PROFILE_EIQUADPROG_CHOWLESKY_INVERSE      "EIQUADPROG_RT Chowlesky inv"
#define PROFILE_EIQUADPROG_ADD_EQ_CONSTR          "EIQUADPROG_RT ADD_EQ_CONSTR"
#define PROFILE_EIQUADPROG_ADD_EQ_CONSTR_1        "EIQUADPROG_RT ADD_EQ_CONSTR_1"
#define PROFILE_EIQUADPROG_ADD_EQ_CONSTR_2        "EIQUADPROG_RT ADD_EQ_CONSTR_2"
#define PROFILE_EIQUADPROG_STEP_1                 "EIQUADPROG_RT STEP_1"
#define PROFILE_EIQUADPROG_STEP_1_1               "EIQUADPROG_RT STEP_1_1"
#define PROFILE_EIQUADPROG_STEP_1_2               "EIQUADPROG_RT STEP_1_2"
#define PROFILE_EIQUADPROG_STEP_1_UNCONSTR_MINIM  "EIQUADPROG_RT STEP_1_UNCONSTR_MINIM"
#define PROFILE_EIQUADPROG_STEP_2                 "EIQUADPROG_RT STEP_2"
#define PROFILE_EIQUADPROG_STEP_2A                "EIQUADPROG_RT STEP_2A"
#define PROFILE_EIQUADPROG_STEP_2B                "EIQUADPROG_RT STEP_2B"
#define PROFILE_EIQUADPROG_STEP_2C                "EIQUADPROG_RT STEP_2C"

#define DEFAULT_MAX_ITER 1000

template<int Rows, int Cols>
struct RtMatrixX
{
//  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign, Rows, Cols> d;
  typedef Eigen::Matrix<double, Rows, Cols, Eigen::AutoAlign> d;
};

template<int Rows>
struct RtVectorX
{
//  typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::AutoAlign, Rows, 1> d;
//  typedef Eigen::Matrix<int,    Eigen::Dynamic, 1, Eigen::AutoAlign, Rows, 1> i;
  typedef Eigen::Matrix<double, Rows, 1, Eigen::AutoAlign> d;
  typedef Eigen::Matrix<int,    Rows, 1, Eigen::AutoAlign> i;
};

namespace tsid
{

  namespace solvers
  {

    /**
    * Possible states of the solver.
    */
    enum RtEiquadprog_status
    {
      RT_EIQUADPROG_OPTIMAL=0,
      RT_EIQUADPROG_INFEASIBLE=1,
      RT_EIQUADPROG_UNBOUNDED=2,
      RT_EIQUADPROG_MAX_ITER_REACHED=3,
      RT_EIQUADPROG_REDUNDANT_EQUALITIES=4
    };

    template<int nVars, int nEqCon, int nIneqCon>
    class RtEiquadprog
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      RtEiquadprog();
      virtual ~RtEiquadprog();

      int getMaxIter() const { return m_maxIter; }

      bool setMaxIter(int maxIter)
      {
        if(maxIter<0)
          return false;
        m_maxIter = maxIter;
        return true;
      }

      /**
       * @return The size of the active set, namely the number of
       * active constraints (including the equalities).
       */
      int getActiveSetSize() const { return q; }

      /**
       * @return The number of active-set iteratios.
       */
      int getIteratios() const { return iter; }

      /**
       * @return The value of the objective function.
       */
      double getObjValue() const { return f_value; }

      /**
       * @return The Lagrange multipliers
       */
      const typename RtVectorX<nIneqCon+nEqCon>::d & getLagrangeMultipliers() const
      {
        return u;
      }

      /**
       * Return the active set, namely the indeces of active constraints.
       * The first nEqCon indexes are for the equalities and are negative.
       * The last nIneqCon indexes are for the inequalities and start from 0.
       * Only the first q elements of the return vector are valid, where q
       * is the size of the active set.
       * @return The set of indexes of the active constraints.
       */
      const typename RtVectorX<nIneqCon+nEqCon>::i & getActiveSet() const
      {
        return A;
      }


      /**
       * solves the problem
       * min. x' Hess x + 2 g0' x
       * s.t. CE x + ce0 = 0
       *      CI x + ci0 >= 0
       */
      inline RtEiquadprog_status solve_quadprog(
          const typename RtMatrixX<nVars,nVars>::d & Hess,
          const typename RtVectorX<nVars>::d & g0,
          const typename RtMatrixX<nEqCon, nVars>::d & CE,
          const typename RtVectorX<nEqCon>::d & ce0,
          const typename RtMatrixX<nIneqCon, nVars>::d & CI,
          const typename RtVectorX<nIneqCon>::d & ci0,
          typename RtVectorX<nVars>::d & x);

      typename RtMatrixX<nVars,nVars>::d m_J; // J * J' = Hessian
      bool is_inverse_provided_;

    private:

      int m_maxIter;  /// max number of active-set iterations
      double f_value; /// current value of cost function

      Eigen::LLT<typename RtMatrixX<nVars,nVars>::d,Eigen::Lower> chol_;
      double solver_return_;

      /// from QR of L' N, where L is Cholewsky factor of Hessian, and N is the matrix of active constraints
      typename RtMatrixX<nVars,nVars>::d R;

      /// CI*x+ci0
      typename RtVectorX<nIneqCon>::d s;

      /// infeasibility multipliers, i.e. negative step direction in dual space
      typename RtVectorX<nIneqCon+nEqCon>::d r;

      /// Lagrange multipliers
      typename RtVectorX<nIneqCon+nEqCon>::d u;

      /// step direction in primal space
      typename RtVectorX<nVars>::d z;

      /// J' np
      typename RtVectorX<nVars>::d d;

      /// current constraint normal
      typename RtVectorX<nVars>::d np;

      /// active set (indeces of active constraints)
      /// the first nEqCon indeces are for the equalities and are negative
      /// the last nIneqCon indeces are for the inequalities are start from 0
      typename RtVectorX<nIneqCon+nEqCon>::i A;

      /// initialized as K \ A
      /// iai(i)=-1 iff inequality constraint i is in the active set
      /// iai(i)=i otherwise
      typename RtVectorX<nIneqCon>::i iai;

      /// initialized as [1, ..., 1, .]
      /// if iaexcl(i)!=1 inequality constraint i cannot be added to the active set
      /// if adding ineq constraint i fails => iaexcl(i)=0
      /// iaexcl(i)=0 iff ineq constraint i is linearly dependent to other active constraints
      /// iaexcl(i)=1 otherwise
      typename RtVectorX<nIneqCon>::i iaexcl;

      typename RtVectorX<nVars>::d x_old;           // old value of x
      typename RtVectorX<nIneqCon+nEqCon>::d u_old; // old value of u
      typename RtVectorX<nIneqCon+nEqCon>::i A_old; // old value of A

#ifdef OPTIMIZE_ADD_CONSTRAINT
      typename RtVectorX<nVars>::d T1;           // tmp vector
#endif
	  
      /// size of the active set A (containing the indices of the active constraints)
      int q;

      /// number of active-set iterations
      int iter;

      template<typename Scalar>
      inline Scalar distance(Scalar a, Scalar b)
      {
        Scalar a1, b1, t;
        a1 = std::abs(a);
        b1 = std::abs(b);
        if (a1 > b1)
        {
          t = (b1 / a1);
          return a1 * std::sqrt(1.0 + t * t);
        }
        else
          if (b1 > a1)
          {
            t = (a1 / b1);
            return b1 * std::sqrt(1.0 + t * t);
          }
        return a1 * std::sqrt(2.0);
      }

      inline void compute_d(typename RtVectorX<nVars>::d & d,
                            const typename RtMatrixX<nVars,nVars>::d & J,
                            const typename RtVectorX<nVars>::d & np)
      {
#ifdef OPTIMIZE_COMPUTE_D
        d.noalias() = J.adjoint() * np;
#else
        d = J.adjoint() * np;
#endif
      }

      inline void update_z(typename RtVectorX<nVars>::d & z,
                           const typename RtMatrixX<nVars,nVars>::d & J,
                           const typename RtVectorX<nVars>::d & d,
                           int iq)
      {
#ifdef OPTIMIZE_UPDATE_Z
        z.noalias() = J.rightCols(nVars-iq) * d.tail(nVars-iq);
#else
        z = J.rightCols(J.cols()-iq) * d.tail(J.cols()-iq);
#endif
      }

      inline void update_r(const typename RtMatrixX<nVars,nVars>::d & R,
                           typename RtVectorX<nIneqCon+nEqCon>::d& r,
                           const typename RtVectorX<nVars>::d& d,
                           int iq)
      {
        r.head(iq)= d.head(iq);
        R.topLeftCorner(iq,iq).template triangularView<Eigen::Upper>().solveInPlace(r.head(iq));
      }

      inline bool add_constraint(
          typename RtMatrixX<nVars,nVars>::d & R,
          typename RtMatrixX<nVars,nVars>::d & J,
          typename RtVectorX<nVars>::d & d,
          int& iq, double& R_norm);

      inline void delete_constraint(
          typename RtMatrixX<nVars,nVars>::d& R,
          typename RtMatrixX<nVars,nVars>::d& J,
          typename RtVectorX<nIneqCon+nEqCon>::i & A,
          typename RtVectorX<nIneqCon+nEqCon>::d & u,
          int& iq, int l);
    };

  } /* namespace solvers */
} /* namespace tsid */

#endif /* __invdyn_eiquadprog_rt_hpp__ */
