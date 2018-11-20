#ifndef __tsid_python_util_container_hpp__
#define __tsid_python_util_container_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/solvers/fwd.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-bound.hpp"

using namespace std;
namespace tsid
{
  namespace python
  {         
    typedef solvers::ConstraintLevel ConstraintLevel;
    typedef solvers::HQPData HQPData;

    class ConstraintLevels
    {
    public:
        ConstraintLevels(){}
        ~ConstraintLevels(){}

        inline void print (){
            stringstream ss;
            for(ConstraintLevel::const_iterator iit=m_std_const.begin(); iit!=m_std_const.end(); iit++)
            {
                const math::ConstraintBase* c = iit->second;
                ss<<" - "<<c->name()<<": w="<<iit->first<<", ";
                if(c->isEquality())
                    ss<<"equality, ";
                else if(c->isInequality())
                    ss<<"inequality, ";
                else
                    ss<<"bound, ";
                ss<<c->rows()<<"x"<<c->cols()<<endl;
            }
            cout << ss.str() << endl;
        }
        inline ConstraintLevel& get (){
            return m_std_const;
        }
    
        inline void append_eq (double num, math::ConstraintEquality* i){
           m_std_const.push_back(solvers::make_pair<double, math::ConstraintBase*>(num, i));
        }
        inline void append_ineq (double num, math::ConstraintInequality* i){
           m_std_const.push_back(solvers::make_pair<double, math::ConstraintBase*>(num, i));
        }
        inline void append_bound (double num, math::ConstraintBound* i){
           m_std_const.push_back(solvers::make_pair<double, math::ConstraintBase*>(num, i));
        }
    private:
        ConstraintLevel m_std_const;
    };

    class HQPDatas
    {
    public:
        HQPDatas(){}
        ~HQPDatas(){}

        inline void resize (size_t i) {
           m_std_hqp.resize(i);
        }

        inline void print () const {
            stringstream ss;
            unsigned int priority = 0;
            for(HQPData::const_iterator it=m_std_hqp.begin(); it!=m_std_hqp.end(); it++)
            {
                ss<<"Level "<< priority<<endl;
                for(ConstraintLevel::const_iterator iit=it->begin(); iit!=it->end(); iit++)
                {
                const math::ConstraintBase* c = iit->second;
                ss<<" - "<<c->name()<<": w="<<iit->first<<", ";
                if(c->isEquality())
                    ss<<"equality, ";
                else if(c->isInequality())
                    ss<<"inequality, ";
                else
                    ss<<"bound, ";
                ss<<c->rows()<<"x"<<c->cols()<<endl;
                }
                priority++;
            }
             cout << ss.str() << endl;
        }
        // inline void append (ConstraintLevel cons){
        //     m_std_hqp.push_back(cons);
        // }
        inline void append_helper (ConstraintLevels* cons){
            m_std_hqp.push_back(cons->get());
        }

        inline HQPData get (){
            return m_std_hqp;
        }
        inline bool set (HQPData data){
            m_std_hqp = data;
        }
    
    private:
        HQPData m_std_hqp;
    };
  }
}


#endif // ifndef __tsid_python_util_container_hpp__