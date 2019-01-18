/*
 * Copyright 2018, Gepetto team, LAAS-CNRS
 *
 * This file is part of sot-talos-balance.
 * sot-talos-balance is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-talos-balance is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-talos-balance.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "sot/talos_balance/simple-zmp-estimator.hh"

#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

#include "sot/talos_balance/utils/commands-helper.hh"
#include "sot/talos_balance/utils/stop-watch.hh"

namespace dynamicgraph
{
  namespace sot
  {
    namespace talos_balance
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;

//Size to be aligned                                    "-------------------------------------------------------"
#define PROFILE_SIMPLEZMPESTIMATOR_ZMP_COMPUTATION      "SimpleZmpEstimator: zmp computation                    "
#define PROFILE_SIMPLEZMPESTIMATOR_COPLEFT_COMPUTATION  "SimpleZmpEstimator: copLeft computation                "
#define PROFILE_SIMPLEZMPESTIMATOR_COPRIGHT_COMPUTATION "SimpleZmpEstimator: copRight computation               "

#define INPUT_SIGNALS m_wrenchLeftSIN << m_wrenchRightSIN << m_poseLeftSIN << m_poseRightSIN

#define OUTPUT_SIGNALS m_copLeftSOUT << m_copRightSOUT << m_zmpSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef SimpleZmpEstimator EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(SimpleZmpEstimator,
                                         "SimpleZmpEstimator");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      SimpleZmpEstimator::SimpleZmpEstimator(const std::string& name)
                      : Entity(name)
                      , CONSTRUCT_SIGNAL_IN(wrenchLeft, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(wrenchRight, dynamicgraph::Vector)
                      , CONSTRUCT_SIGNAL_IN(poseLeft, MatrixHomogeneous)
                      , CONSTRUCT_SIGNAL_IN(poseRight, MatrixHomogeneous)
                      , CONSTRUCT_SIGNAL_OUT(copLeft, dynamicgraph::Vector, m_wrenchLeftSIN << m_poseLeftSIN)
                      , CONSTRUCT_SIGNAL_OUT(copRight, dynamicgraph::Vector, m_wrenchRightSIN << m_poseRightSIN)
                      , CONSTRUCT_SIGNAL_OUT(zmp, dynamicgraph::Vector, m_wrenchLeftSIN << m_wrenchRightSIN << m_copLeftSOUT << m_copRightSOUT)
                      , m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init", makeCommandVoid0(*this, &SimpleZmpEstimator::init, docCommandVoid0("Initialize the entity.")));
      }

      void SimpleZmpEstimator::init()
      {
        if(!m_wrenchLeftSIN.isPlugged())
          return SEND_MSG("Init failed: signal wrenchLeft is not plugged", MSG_TYPE_ERROR);
        if(!m_poseLeftSIN.isPlugged())
          return SEND_MSG("Init failed: signal poseLeft is not plugged", MSG_TYPE_ERROR);
        if(!m_wrenchRightSIN.isPlugged())
          return SEND_MSG("Init failed: signal wrenchRight is not plugged", MSG_TYPE_ERROR);
        if(!m_poseRightSIN.isPlugged())
          return SEND_MSG("Init failed: signal poseRight is not plugged", MSG_TYPE_ERROR);

        m_initSucceeded = true;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      dynamicgraph::Vector computeCoP(const dg::Vector & wrench, const MatrixHomogeneous & pose)
      {
          const double h = pose(2,3);

          const double fx = wrench[0];
          const double fy = wrench[1];
          const double fz = wrench[2];
          const double tx = wrench[3];
          const double ty = wrench[4];

          const double px = (- ty - fx*h)/fz;
          const double py = (  tx - fy*h)/fz;
          const double pz = - h;

          dg::Vector copLocal(3);
          copLocal[0] = px;
          copLocal[1] = py;
          copLocal[2] = pz;

          dg::Vector cop = pose.linear()*copLocal + pose.translation();

          return cop;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(copLeft, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal copLeft before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_SIMPLEZMPESTIMATOR_COPLEFT_COMPUTATION);

        const dynamicgraph::Vector & wrenchLeft = m_wrenchLeftSIN(iter);
        const MatrixHomogeneous & poseLeft = m_poseLeftSIN(iter);

        assert(wrenchLeft.size()==6 && "Unexpected size of signal wrenchLeft");

        s = computeCoP(wrenchLeft,poseLeft);

        getProfiler().stop(PROFILE_SIMPLEZMPESTIMATOR_COPLEFT_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(copRight, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal copRight before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_SIMPLEZMPESTIMATOR_COPRIGHT_COMPUTATION);

        const dynamicgraph::Vector & wrenchRight = m_wrenchRightSIN(iter);
        const MatrixHomogeneous & poseRight = m_poseRightSIN(iter);

        assert(wrenchRight.size()==6 && "Unexpected size of signal wrenchRight");

        s = computeCoP(wrenchRight,poseRight);

        getProfiler().stop(PROFILE_SIMPLEZMPESTIMATOR_COPRIGHT_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(zmp, dynamicgraph::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal zmp before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_SIMPLEZMPESTIMATOR_ZMP_COMPUTATION);

        const dynamicgraph::Vector & wrenchLeft = m_wrenchLeftSIN(iter);
        const dynamicgraph::Vector & copLeft = m_copLeftSOUT(iter);

        const dynamicgraph::Vector & wrenchRight = m_wrenchRightSIN(iter);
        const dynamicgraph::Vector & copRight = m_copRightSOUT(iter);

        assert(wrenchLeft.size()==6 && "Unexpected size of signal wrenchLeft");
        assert(wrenchRight.size()==6 && "Unexpected size of signal wrenchRight");

        const double fzLeft = wrenchLeft[2];
        const double fzRight = wrenchRight[2];

        s = ( copLeft*fzLeft + copRight*fzRight ) / ( fzLeft + fzRight );

        getProfiler().stop(PROFILE_SIMPLEZMPESTIMATOR_ZMP_COMPUTATION);

        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void SimpleZmpEstimator::display(std::ostream& os) const
      {
        os << "SimpleZmpEstimator " << getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }
    } // namespace talos_balance
  } // namespace sot
} // namespace dynamicgraph
