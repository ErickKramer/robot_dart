#ifndef ARM_SIMULATION_HPP
#define ARM_SIMULATION_HPP
// General C++ lib
#include <iostream>
#include <algorithm>
#include <cstdlib>

// robot_dart lib 
#include <robot_dart/control/pid_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>
// Load graphics 
#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif

// dart lib 
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>



#include <Eigen/Core>

namespace arm_dart{
    class SchunkArm{
    public:
        SchunkArm(std::string urdf_path, 
            std::vector<std::pair<std::string, std::string>> packages, double time_step = 0.001, 
            std::string name="arm robot");
        
        ~SchunkArm();

        void run_sim(double max_duration = 5.0);

        Eigen::VectorXd compute_pose(Eigen::Isometry3d link_transform);

        double movement_duration(std::vector<Eigen::VectorXd> velocities, double time_step);

        double total_movement(Eigen::VectorXd init_conf, Eigen::VectorXd end_conf);

        void display_run_results(robot_dart::RobotDARTSimu simu, double time_step, std::vector<double>& ctrl);

        int num_ctrl_dofs;

        
    };
} //namespace arm_dart

#endif