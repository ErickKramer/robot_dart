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
            double time_step) : _simu(std::make_shared<robot_dart::RobotDARTSimu>())
            {
                std::cout << "Testing class " << std::endl;
                std::cout << "Name " << name << std::endl;
                std::cout << "Time step "<< time_step << std::endl;
            
            // Load robot
            _arm_robot = std::make_shared<robot_dart::Robot>(urdf_path, packages,name);

            // Create Simulation 
            // robot_dart::RobotDARTSimu simu(time_step);
            // _simu->set_step(time_step);

            // Pin arm to the world
            _arm_robot->fix_to_world();
            _arm_robot->set_position_enforced(true);

            // Load graphics
            // #ifdef GRAPHIC
            //     // Set graphics properties (world, resolution, shadows, real_time_node)
            //     _simu->set_graphics(
            //         std::make_shared<robot_dart::graphics::Graphics>(_simu->world(), 1920,1080,true,true));

            //     // Set Camera position
            //     std::static_pointer_cast<robot_dart::graphics::Graphics>(_simu->graphics())->
            //         look_at({0.,3.,0.}, {0., 0., 0.5});

            // #endif

            // Get DOFs of the robot
            _num_dofs = _arm_robot->skeleton()->getNumDofs();

            // Get DOFs that can be controlled (identify mimic joints)
            double mimic_joints = 0;
            for (int i=0; i<_num_dofs; i++){
                auto joint = _arm_robot->skeleton()->getDof(i)->getJoint();
                if (joint->getActuatorType() == dart::dynamics::Joint::MIMIC){
                    mimic_joints++;
                }
            }

            _num_ctrl_dofs = static_cast<int>(_num_dofs - mimic_joints);
            std::cout << "Robot " << name << " has " << _num_ctrl_dofs << " DOFS " << std::endl;

            // Adds collision detection
            // _simu.world()->getConstraintSolver()->
            //     setCollisionDetector(dart::collision::FCLCollisionDetector::create());
        }
        //==============================================================================
        void init_simu(double time_step){
            _simu->set_step(time_step);
            std::cout << "Time step set to " << _simu->step() << std::endl;
            // Load graphics
            #ifdef GRAPHIC
                // Set graphics properties (world, resolution, shadows, real_time_node)
                _simu->set_graphics(
                    std::make_shared<robot_dart::graphics::Graphics>(_simu->world(), 1920,1080,true,true));

                // Set Camera position
                std::static_pointer_cast<robot_dart::graphics::Graphics>(_simu->graphics())->
                    look_at({0.,3.,0.}, {0., 0., 0.5});

            #endif

        }
        //==============================================================================
        void init_controller(std::string pid_file_path){
            // Creates a PID controller extracting the paramters from a text file and 
            // adds it to the arm_robot 

            std::cout << "Initializing controller " << std::endl;
            // Initial arm configuration
            std::vector<double> ctrl(_num_ctrl_dofs, 0.0);

            // Add PID Controller
            _arm_robot->add_controller(std::make_shared<robot_dart::control::PIDControl>(ctrl));
            
            Eigen::VectorXd Kp = Eigen::VectorXd::Ones(_num_ctrl_dofs); 
            Eigen::VectorXd Ki = Eigen::VectorXd::Ones(_num_ctrl_dofs); 
            Eigen::VectorXd Kd = Eigen::VectorXd::Ones(_num_ctrl_dofs); 
            double i_min = 0;
            double i_max = 0;

            // Extract PID params from file
            std::ifstream pid_file(pid_file_path);
            std::string _line;
            std::cout << "Reading file " << pid_file_path << std::endl;
            
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

            // Set PID gains
            std::static_pointer_cast<robot_dart::control::PIDControl>(_arm_robot->controllers()[0])
            ->set_pid(Kp, Ki, Kd, i_min, i_max);

            std::cout << "PID Controller set" << std::endl; 
        }
        //==============================================================================
        void set_acceleration_limits(double acc_limit){
            Eigen::VectorXd acc_upper_limit = Eigen::VectorXd::Ones(_num_dofs) * acc_limit;
            Eigen::VectorXd acc_lower_limit = Eigen::VectorXd::Ones(_num_dofs) * - acc_limit;
            _arm_robot->skeleton()->setAccelerationUpperLimits(acc_upper_limit);
            _arm_robot->skeleton()->setAccelerationLowerLimits(acc_lower_limit);
        }
        //==============================================================================
        /*    
        void run_sim(double max_duration = 5.0);
Eigen::VectorXd compute_pose(Eigen::Isometry3d link_transform);
double movement_duration(std::vector<Eigen::VectorXd> velocities, double time_step);
        
        double total_movement(Eigen::VectorXd init_conf, Eigen::VectorXd end_conf);

        void display_run_results(robot_dart::RobotDARTSimu simu, double time_step, std::vector<double>& ctrl);
    */
    protected:
        robot_t _arm_robot;
        int _num_dofs;
        int _num_ctrl_dofs;
        robot_dart::RobotDARTSimuPtr _simu;



    };
} //namespace arm_dart

#endif
