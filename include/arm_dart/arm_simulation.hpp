#ifndef ARM_SIMULATION_HPP
#define ARM_SIMULATION_HPP

// General C++ lib
#include <iostream>
#include <algorithm>
#include <cstdlib>

// robot_dart lib
#include <robot_dart/robot.hpp>
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

// Eigen lib
#include <Eigen/Core>

namespace arm_dart{
    class SchunkArm{
    public:
        using robot_t = std::shared_ptr<robot_dart::Robot>;
        SchunkArm(std::string urdf_path,
            std::vector<std::pair<std::string, std::string>> packages, std::string name,
            double time_step){
                std::cout << "Testing class " << std::endl;
                std::cout << "Name " << name << std::endl;
                std::cout << "Time step "<< time_step << std::endl;
            
            // Load robot
            _arm_robot = std::make_shared<robot_dart::Robot>(urdf_path, packages,name);

            // Create Simulation 
            robot_dart::RobotDARTSimu simu(time_step);
            // Pin arm to the world
            _arm_robot->fix_to_world();
            _arm_robot->set_position_enforced(true);

            // Load graphics
            #ifdef GRAPHIC
                // Set graphics properties (world, resolution, shadows, real_time_node)
                simu.set_graphics(
                    std::make_shared<robot_dart::graphics::Graphics>(simu.world(), 1920,1080,true,true));

                // Set Camera position
                std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics())->
                    look_at({0.,3.,0.}, {0., 0., 0.5});

            #endif

            // Get DOFs of the robot
            double num_dofs = _arm_robot->skeleton()->getNumDofs();

            // Get DOFs that can be controlled (identify mimic joints)
            double mimic_joints = 0;
            for (size_t i=0; i<num_dofs; i++){
                auto joint = _arm_robot->skeleton()->getDof(i)->getJoint();
                if (joint->getActuatorType() == dart::dynamics::Joint::MIMIC){
                    mimic_joints++;
                }
            }

            _num_ctrl_dofs = static_cast<int>(num_dofs - mimic_joints);
            std::cout << "Robot " << name << " has " << _num_ctrl_dofs << " DOFS " << std::endl;
        }
        
        void set_controller(std::string pid_file_path){
            // Initial arm configuration
            std::vector<double> ctrl(_num_ctrl_dofs, 0.0);

            // Add PID Controller
            _arm_robot->add_controller(std::make_shared<robot_dart::control::PIDControl>(ctrl));
            
            Eigen::VectorXd Kp = Eigen::VectorXd::Ones(_num_ctrl_dofs); 
            Eigen::VectorXd Ki = Eigen::VectorXd::Ones(_num_ctrl_dofs); 
            Eigen::VectorXd Kd = Eigen::VectorXd::Ones(_num_ctrl_dofs); 
            double i_min;
            double i_max;
            std::ifstream pid_file(pid_file_path);
            std::string _line;
            
            while(std::getline(pid_file, _line)){
                std::istringstream line(_line);
                std::vector<std::string> params(std::istream_iterator<std::string>{line},
                            std::istream_iterator<std::string>());
                if (params[0] == "P"){
                    for (size_t i = 1; i < params.size(); i++){
                        Kp[i-1] = atof(params[i].c_str());
                    }
                    std::cout << "Kp " << Kp.transpose() << std::endl;
                }else if (params[0] == "I"){
                    for (size_t i = 1; i < params.size(); i++){
                        Ki[i-1] = atof(params[i].c_str());
                    }
                    std::cout << "Ki " << Ki.transpose() << std::endl;
                }else if (params[0] == "D"){
                    for (size_t i = 1; i < params.size(); i++){
                        Kd[i-1] = atof(params[i].c_str());
                    }
                    std::cout << "Kd " << Kd.transpose() << std::endl;
                }else if (params[0] == "i_limits"){
                    i_min = atof(params[1].c_str());
                    i_max = atof(params[2].c_str());
                    std::cout << "i limits " << i_min << " " << i_max << std::endl;
                }

            }
            pid_file.close();

            // Set PD gains
            std::static_pointer_cast<robot_dart::control::PIDControl>(_arm_robot->controllers()[0])
            ->set_pid(Kp, Ki, Kd, i_min, i_max);


        }
        /*
        void run_sim(double max_duration = 5.0);

        Eigen::VectorXd compute_pose(Eigen::Isometry3d link_transform);

        double movement_duration(std::vector<Eigen::VectorXd> velocities, double time_step);

        double total_movement(Eigen::VectorXd init_conf, Eigen::VectorXd end_conf);

        void display_run_results(robot_dart::RobotDARTSimu simu, double time_step, std::vector<double>& ctrl);
    */
    protected:
        robot_t _arm_robot;
        int _num_ctrl_dofs;


    };
} //namespace arm_dart

#endif
