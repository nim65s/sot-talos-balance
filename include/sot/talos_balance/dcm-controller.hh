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

#ifndef __sot_talos_balance_dcm_controller_H__
#define __sot_talos_balance_dcm_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (dcm_controller_EXPORTS)
#    define DCMCONTROLLER_EXPORT __declspec(dllexport)
#  else
#    define DCMCONTROLLER_EXPORT __declspec(dllimport)
#  endif
#else
#  define DCMCONTROLLER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include "utils/signal-helper.hh"
#include "utils/logger.hh"
#include <map>
#include "boost/assign.hpp"

namespace dynamicgraph {
  namespace sot {
    namespace talos_balance {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class DCMCONTROLLER_EXPORT DcmController
	                         : public ::dynamicgraph::Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        DcmController( const std::string & name );

        void init(const double & dt);

        void resetDcmIntegralError();

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(Kp, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(Ki, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(decayFactor, double);
        DECLARE_SIGNAL_IN(omega, double);
        DECLARE_SIGNAL_IN(mass, double);
        DECLARE_SIGNAL_IN(com, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(dcm, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(dcmDes, dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(zmpDes, dynamicgraph::Vector);

        DECLARE_SIGNAL_OUT(zmpRef, dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(wrenchRef, dynamicgraph::Vector);

        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[DcmController-"+name+"] "+msg, t, file, line);
        }

      protected:
        bool m_initSucceeded;    /// true if the entity has been successfully initialized
        dynamicgraph::Vector m_dcmIntegralError; // internal state
        double m_dt;

      }; // class DcmController

    }    // namespace talos_balance
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_talos_balance_dcm_controller_H__