/*
 * Copyright 2019
 * LAAS-CNRS
 * F. Bailly
 * T. Flayol
 * F. Risbourg
 */

#ifndef __sot_talos_balance_ft_wrist_calibration_H__
#define __sot_talos_balance_ft_wrist_calibration_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (__sot_talos_balance_ft_wrist_calibration_H__)
#    define SOTFTWRISTCALIBRATION_EXPORT __declspec(dllexport)
#  else
#    define SOTFTWRISTCALIBRATION_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTFTWRISTCALIBRATION_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <dynamic-graph/signal-helper.h>
#include <dynamic-graph/real-time-logger.h>
#include <sot/core/matrix-geometry.hh>
#include <sot/core/robot-utils.hh>
#include <map>
#include "boost/assign.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>


#include <sot/talos_balance/robot/robot-wrapper.hh>

namespace dynamicgraph {
  namespace sot {
    namespace talos_balance {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTFTWRISTCALIBRATION_EXPORT FtWristCalibration
                     :public::dynamicgraph::Entity
      {
        //typedef FtCalibration EntityClassName;
        DYNAMIC_GRAPH_ENTITY_DECL();

      public:
        /* --- CONSTRUCTOR ---- */
        FtWristCalibration( const std::string & name);
        /// Initialize
        void init(const std::string & robotRef);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(rightWristForceIn,  dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(leftWristForceIn,   dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(q, dynamicgraph::Vector);
        DECLARE_SIGNAL_INNER(rightWeight, dynamicgraph::Vector);
        DECLARE_SIGNAL_INNER(leftWeight, dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(rightWristForceOut, dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(leftWristForceOut, dynamicgraph::Vector);

        /* --- COMMANDS --- */

        /// Commands for setting the hand weight
        void setRightHandWeight(const Vector &rightW);
        void setLeftHandWeight(const  Vector  &leftW);
        
        /// Command to calibrate the wrist sensors when the robot is in half sitting with the hands aligned
        void calibrateWristSensor();

        /**
         * @brief  Set to true if you want to remove the weight from the force
         * 
         * @param[in] removeWeight  Boolean used to remove the weight
         */
        void setRemoveWeight(const bool &removeWeight);

        void displayRobotUtil();

        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        
        /* --- TYPEDEFS ---- */
        typedef Eigen::Matrix<double, 6, 1>    Vector6d;

      protected:
        /// Robot Util instance to get the sensor frame
        RobotUtilShrPtr m_robot_util;
        /// Pinocchio robot model
        pinocchio::Model m_model;
        /// Pinocchio robot data
        pinocchio::Data *m_data;
        /// Id of the force sensor frame
        pinocchio::FrameIndex m_rightSensorId;
        /// Id of the joint of the end-effector
        pinocchio::FrameIndex m_leftSensorId;
        /// Number of iteration for right calibration (-2 = not calibrated, -1 = caibration done)
        int m_rightCalibrationIter = -2;
        /// Number of iteration for right calibration (-2 = not calibrated, -1 = caibration done)
        int m_leftCalibrationIter =  -2;
        /// Offset or bias to be removed from Right FT sensor
        Vector6d m_right_FT_offset;
        /// Offset or bias to be removed from Left FT sensor
        Vector6d m_left_FT_offset;
        /// Variable used durring average computation of the offset
        Vector6d m_right_FT_offset_calibration_sum;
        /// Variable used durring average computation of the offset
        Vector6d m_left_FT_offset_calibration_sum;
        /// true if the entity has been successfully initialized
        bool m_initSucceeded;
        /// weight of the right hand
        Vector6d m_rightHandWeight;
        /// weight of the left hand
        Vector6d m_leftHandWeight;
        /// If true, the weight of the end effector is removed from the force
        bool m_removeWeight;


      }; // class FtWristCalibration

    }    // namespace talos_balance
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_talos_balance_ft_wrist_calibration_H__
