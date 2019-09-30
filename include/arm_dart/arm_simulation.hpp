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
#include <robot_dart/utils.hpp>

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

    Eigen::VectorXd compute_pose(Eigen::Isometry3d link_transform){
        //--------------------------------------------------------------------------
        // Computes end_effector pose 
        //--------------------------------------------------------------------------
        // Computes rotation matrix
        Eigen::Matrix3d rot_matrix = link_transform.rotation();
        // Computes Homogeneous transformation matrix
        // Eigen::Matrix4d transformation_matrix = link_transform.matrix();
        // Computes Quaternions
        Eigen::Quaterniond quat_end(rot_matrix);
        // Computes Euler angles
        Eigen::Vector3d euler = rot_matrix.eulerAngles(2,1,0);
        // Computes translation vector
        Eigen::VectorXd translation = link_transform.translation();
        // Computes pose vector
        Eigen::VectorXd pose(link_transform.translation().size() + euler.size());
        pose << translation, euler;

        return robot_dart::Utils::round_small(pose);
    }

    struct JointStateDesc : public robot_dart::descriptor::BaseDescriptor{
        //--------------------------------------------------------------------------
        // Descriptor used to record the joints states
        //--------------------------------------------------------------------------
        JointStateDesc(robot_dart::RobotDARTSimu& simu, size_t desc_dump = 1) :
            robot_dart::descriptor::BaseDescriptor(simu, desc_dump) {}

        void operator()(){
            // Stores the joint positions
            if (_simu.robots().size() > 0){
                // Add current joints configuration to the joints_states vector
                joints_states.push_back(_simu.robots()[0]->skeleton()->getPositions());

            }
        }

        std::vector<Eigen::VectorXd> joints_states;
    };

    struct PoseStateDesc : public robot_dart::descriptor::BaseDescriptor{
        //--------------------------------------------------------------------------
        // Descriptor used to record end effector states
        //--------------------------------------------------------------------------
        PoseStateDesc(robot_dart::RobotDARTSimu& simu, size_t desc_dump = 1) :
            robot_dart::descriptor::BaseDescriptor(simu, desc_dump) {}

        void operator()(){
            // Stores the end effector poses
            if(_simu.robots().size() > 0){
                // Get numJoints
                double numJoints = _simu.robots()[0]->skeleton()->getNumBodyNodes();

                // Add current end effector pose to the end_effector states vector
                pose_states.push_back(compute_pose(_simu.robots()[0]->skeleton()->
                    getBodyNode(numJoints - 1)->getWorldTransform()));
            }
        }

        std::vector<Eigen::VectorXd> pose_states;
    };

    struct JointVelDesc : public robot_dart::descriptor::BaseDescriptor{
        //--------------------------------------------------------------------------
        // Descriptor used to record joint velocities 
        //--------------------------------------------------------------------------
        JointVelDesc(robot_dart::RobotDARTSimu& simu, size_t desc_dump = 1) :
            robot_dart::descriptor::BaseDescriptor(simu, desc_dump) {}

        void operator()(){
            // Stores the joint positions
            if (_simu.robots().size() > 0){
                // Add current joints configuration to the joints_velocities vector
                joints_velocities.push_back(_simu.robots()[0]->skeleton()->getVelocities());

            }
        }

        std::vector<Eigen::VectorXd> joints_velocities;
    };

    class SchunkArm{
    public:
        using robot_t = std::shared_ptr<robot_dart::Robot>;
        SchunkArm(std::string urdf_path,
            std::vector<std::pair<std::string, std::string>> packages, std::string name) : _simu(std::make_shared<robot_dart::RobotDARTSimu>())
        {
            //--------------------------------------------------------------------------
            // Initialize robot
            //--------------------------------------------------------------------------
            _arm_robot = std::make_shared<robot_dart::Robot>(urdf_path, packages,name);

            // Pin arm to the world
            _arm_robot->fix_to_world();
            _arm_robot->set_position_enforced(true);

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

        }

        //==============================================================================
        void init_simu(double time_step){
            //--------------------------------------------------------------------------
            // Initialize Simulation
            //--------------------------------------------------------------------------
            _simu->set_step(time_step);

            // Load graphics
            #ifdef GRAPHIC
                // Set graphics properties (world, resolution, shadows, real_time_node)
                _simu->set_graphics(
                    std::make_shared<robot_dart::graphics::Graphics>(_simu->world(), 1920,1080,true,true));

                // Set Camera position
                std::static_pointer_cast<robot_dart::graphics::Graphics>(_simu->graphics())->
                    look_at({0.,3.,0.}, {0., 0., 0.5});

            #endif

            // Adds collision detection
            _simu->world()->getConstraintSolver()->
                setCollisionDetector(dart::collision::FCLCollisionDetector::create());

            // Add robot to the simulation
            _simu->add_robot(_arm_robot);
        }

        //==============================================================================
        void set_descriptors(std::vector<std::string> descriptors){
            //--------------------------------------------------------------------------
            // Add descriptors to the simulation
            //--------------------------------------------------------------------------
            for (const auto &descriptor: descriptors){
                if (descriptor == "joint_states"){
                    // std::cout << "Adding joint states desc " << std::endl;
                    _simu->add_descriptor(std::make_shared<JointStateDesc>(*_simu));
                }else if (descriptor == "pose_states"){
                    // std::cout << "Adding pose states desc " << std::endl;
                    _simu->add_descriptor(std::make_shared<PoseStateDesc>(*_simu));
                }else if (descriptor == "velocity_states"){
                    // std::cout << "Adding velocity states desc " << std::endl;
                    _simu->add_descriptor(std::make_shared<JointVelDesc>(*_simu));
                }
            }
        }
        
        //==============================================================================
        void reset_descriptors(std::vector<std::string> descriptors){
            //--------------------------------------------------------------------------
            // Add descriptors to the simulation
            //--------------------------------------------------------------------------
            for (const auto &descriptor: descriptors){
                if (descriptor == "joint_states"){
                    std::static_pointer_cast<JointStateDesc>(_simu->descriptor(0))->joints_states.clear();
                }else if (descriptor == "pose_states"){
                    std::static_pointer_cast<PoseStateDesc>(_simu->descriptor(1))->pose_states.clear();
                }else if (descriptor == "velocity_states"){
                    std::static_pointer_cast<JointVelDesc>(_simu->descriptor(2))->joints_velocities.clear();
                }
            }
        }

        //==============================================================================
        void run_simu(double simulation_time){
            //--------------------------------------------------------------------------
            // Run simulation
            //--------------------------------------------------------------------------
            _simu->run(simulation_time);
            
            //--------------------------------------------------------------------------
            // Display simulation results
            //--------------------------------------------------------------------------
            std::cout << "Number of joints_states recorded " <<
                std::static_pointer_cast<JointStateDesc>(_simu->descriptor(0))->joints_states.size() << std::endl;
            
            std::vector<Eigen::VectorXd> poses = 
                std::static_pointer_cast<PoseStateDesc>(_simu->descriptor(1))->pose_states;
            std::cout << "Number of pose_states recorded " << poses.size() << std::endl;
    
            Eigen::VectorXd initial_configuration = robot_dart::Utils::round_small(
                std::static_pointer_cast<JointStateDesc>(_simu->descriptor(0))->joints_states.front()); 
            std::cout << "Initial joints configuration \n" << initial_configuration.transpose() << std::endl;

            Eigen::VectorXd end_configuration = robot_dart::Utils::round_small(
                std::static_pointer_cast<JointStateDesc>(_simu->descriptor(0))->joints_states.back()); 
            std::cout << "Final joints configuration \n" << end_configuration.transpose() << std::endl;
   
            Eigen::VectorXd target_positions = Eigen::VectorXd::Map(_ctrl.data(), _ctrl.size());

            // In the case that a joint is mimic (e.g. gripper fingers)
            if (target_positions.size() < end_configuration.size()){
                // Resize the target_positions vector used to control the robot to match the joints configuration vector
                target_positions.conservativeResize(end_configuration.size());
                // Copy the target position value of the last joint controlled to the mimic joint position
                target_positions[end_configuration.size()-1] = target_positions[end_configuration.size()-2];
            } 

            std::cout << "Joint angles requested " << target_positions.transpose() << std::endl;
    
            std::cout << "Error for the arm configuration \n" << (target_positions - end_configuration).transpose() << std::endl;

            std::cout << "Pose of the end effector \n " << poses.back().transpose() << std::endl;

            std::cout << "Total arm movement " << total_movement(initial_configuration, end_configuration) << std::endl;

            // Collect recorded velocities
            std::vector<Eigen::VectorXd> velocities = 
                std::static_pointer_cast<JointVelDesc>(_simu->descriptor(2))->joints_velocities;
            // backup(velocities, "text", "velocities.txt");

            // Computes movement duration
            double duration = movement_duration(velocities, _simu->step());
            std::cout << "Arm stoped at " << duration << " seconds " << std::endl;
            
        }

        //==============================================================================
        double movement_duration(std::vector<Eigen::VectorXd> velocities, double timestep){
            //--------------------------------------------------------------------------
            // Computes duration of the arm movement by finding when the arm stopped moving,
            // i.e. the joint velocities are almost zero
            //--------------------------------------------------------------------------
            double index = 0;
            double threshold = 1e-3;

            for (const auto &element : velocities){
                if (element.isZero(threshold)){
                    std::cout << "Index of joints velocities lower than threshold " << threshold <<
                        " is " << index << std::endl;
                    return timestep * index;
                }
                index++;
            }

            return velocities.size()*timestep;

        }

        //==============================================================================
        double total_movement(Eigen::VectorXd init_conf, Eigen::VectorXd end_conf){
            //--------------------------------------------------------------------------
            // Computes the amount of radians the arm moved from the initial configuration to
            // the final configuration
            //--------------------------------------------------------------------------
            double total = 0;
            for (int i = 0; i < init_conf.size()-1; i++){
                total += abs(end_conf[i] - init_conf[i]);
            }
            return total;
        }

        //==============================================================================
        void init_controller(std::string pid_file_path){
            //--------------------------------------------------------------------------
            // Initialize PID Controller
            //--------------------------------------------------------------------------
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
            _ctrl = ctrl;
        }

        //==============================================================================
        void set_acceleration_limits(double acc_limit){
            //--------------------------------------------------------------------------
            // Set acceleration limits for the joints
            //--------------------------------------------------------------------------
            Eigen::VectorXd acc_upper_limit = Eigen::VectorXd::Ones(_num_dofs) * acc_limit;
            Eigen::VectorXd acc_lower_limit = Eigen::VectorXd::Ones(_num_dofs) * - acc_limit;
            _arm_robot->skeleton()->setAccelerationUpperLimits(acc_upper_limit);
            _arm_robot->skeleton()->setAccelerationLowerLimits(acc_lower_limit);
        }

        //==============================================================================
        void display_robot_info(){
            std::cout << "Arm: " << _arm_robot->name() << std::endl;
            std::cout << "Number of DoF: " << _num_dofs << std::endl;
            std::cout << "Number of controllable DoF: " << _num_ctrl_dofs << std::endl;
            std::cout << "Acceleration lower " << _arm_robot -> skeleton() -> getAccelerationLowerLimits().transpose() << std::endl;
            std::cout << "Acceleration higher " << _arm_robot -> skeleton() -> getAccelerationUpperLimits().transpose() << std::endl;
            std::cout << "Velocity lower " << _arm_robot -> skeleton() -> getVelocityLowerLimits().transpose() << std::endl;
            std::cout << "Velocity higher " << _arm_robot -> skeleton() -> getVelocityUpperLimits().transpose() << std::endl;
            std::cout<<"-----------------------------------"<<std::endl;
        }
        
        //==============================================================================
        void set_goal_configuration(std::vector<double> conf){
            //--------------------------------------------------------------------------
            // Set the goal configuration for the arm
            //--------------------------------------------------------------------------
            _arm_robot->controllers()[0]->set_parameters(conf);
            _ctrl = conf;
        }

        //==============================================================================
        int get_control_dofs(){
            return _num_ctrl_dofs;
        }
        
        //==============================================================================
        void reset_robot(){
            _arm_robot.reset();
        }

    protected:
        robot_t _arm_robot;
        int _num_dofs;
        int _num_ctrl_dofs;
        robot_dart::RobotDARTSimuPtr _simu;
        std::vector<double> _ctrl;



    };
} //namespace arm_dart

#endif
