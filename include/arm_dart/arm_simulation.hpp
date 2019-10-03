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

    //==============================================================================
    bool position_achieved(std::vector<Eigen::VectorXd> joints_positions){
        //--------------------------------------------------------------------------
        // Check whether the joints positions have stabilised up to a desired limit 
        // and with a certain threshold
        //--------------------------------------------------------------------------
        double threshold = 1e-3;
        size_t checking_limit = 500.;

        // Iterate backwards and compare a pair of joints positions
        for (size_t i = 0; i < checking_limit; i++){
            if (!joints_positions.rbegin()[i].isApprox(joints_positions.rbegin()[i+1], threshold))
                return false;
        }

        return true;
    }

    //==============================================================================
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
                if (joints_states.size() > 4){
                    if (position_achieved(joints_states)){
                        _simu.stop_sim();
                    }
                }

            }
        }

        std::vector<Eigen::VectorXd> joints_states;
    };

    //==============================================================================
    struct PoseStateDesc : public robot_dart::descriptor::BaseDescriptor{
        //--------------------------------------------------------------------------
        // Descriptor used to record end effector states
        //--------------------------------------------------------------------------
        PoseStateDesc(robot_dart::RobotDARTSimu& simu, size_t desc_dump = 1) :
            robot_dart::descriptor::BaseDescriptor(simu, desc_dump) {}

        void operator()(){
            // Stores the end effector poses
            if(_simu.robots().size() > 0){

                // Add current end effector pose to the end_effector states vector
                pose_states.push_back(compute_pose(_simu.robots()[0]->skeleton()->
                    getBodyNode("pg70_palm_link")->getWorldTransform()));
            }
        }

        std::vector<Eigen::VectorXd> pose_states;
    };

    //==============================================================================
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

    //==============================================================================
    class SchunkArm{
    public:
        using robot_t = std::shared_ptr<robot_dart::Robot>;
        SchunkArm(std::string urdf_path,
            std::vector<std::pair<std::string, std::string>> packages, std::string name) : _simu(std::make_shared<robot_dart::RobotDARTSimu>())
        {
            //--------------------------------------------------------------------------
            // Initialize robot
            //--------------------------------------------------------------------------
            auto robot = std::make_shared<robot_dart::Robot>(urdf_path, packages,name); 
            _arm_robot = robot->clone(); 

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
        void reset_configuration(){
            //--------------------------------------------------------------------------
            // Set arm to its initial configuration
            //--------------------------------------------------------------------------
            _simu->robots()[0]->skeleton()->setPositions(Eigen::VectorXd::Zero(_num_dofs));
        }

        //==============================================================================
        void run_simu(double simulation_time){
            //--------------------------------------------------------------------------
            // Run simulation
            //--------------------------------------------------------------------------
            _simu->run(simulation_time);
            
            //--------------------------------------------------------------------------
            // Record simulation data
            //--------------------------------------------------------------------------
            // Record total movement
            Eigen::VectorXd end_configuration = robot_dart::Utils::round_small(
                std::static_pointer_cast<JointStateDesc>(_simu->descriptor(0))->joints_states.back()); 
            _total_joints_motion = total_movement(Eigen::VectorXd::Zero(end_configuration.size()), end_configuration);

            // Record total torque
            _total_torque = std::static_pointer_cast<robot_dart::control::PIDControl>(_arm_robot->controllers()[0])
                ->get_total_torque(); 

            // Record end_effector pose 
            std::vector<Eigen::VectorXd> poses = 
                std::static_pointer_cast<PoseStateDesc>(_simu->descriptor(1))->pose_states;
            _end_effector_pose = poses.back();

            // Record simulation duration
            _total_steps = poses.size();
        }

        //==============================================================================
        void backup(const std::vector<Eigen::VectorXd> v, std::string file_type, std::string file_name) {
            if (file_type == "text"){
                std::ofstream out_file(file_name);
                for (const auto &element : v){
                    out_file << element.transpose() << "\n";
                }
            }

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
            std::cout << "Acceleration lower limit " << _arm_robot->skeleton()->getAccelerationLowerLimits().transpose() << std::endl;
            std::cout << "Acceleration upper limit " << _arm_robot->skeleton()->getAccelerationUpperLimits().transpose() << std::endl;
            std::cout << "Velocity lower limit " << _arm_robot->skeleton()->getVelocityLowerLimits().transpose() << std::endl;
            std::cout << "Velocity upper limit " << _arm_robot->skeleton()->getVelocityUpperLimits().transpose() << std::endl;
            std::cout << "Position lower limit " << get_positions_lower_limits().transpose()<< std::endl;
            std::cout << "Position upper limit " << get_positions_upper_limits().transpose()<< std::endl;
            // std::cout << "End effector name " << _arm_robot->skeleton()->getBodyNode("pg70_finger_left_link")->getName() << std::endl;
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

        //==============================================================================
        Eigen::VectorXd get_positions_upper_limits(){
            return _arm_robot->skeleton()->getPositionUpperLimits();
        }
        
        //==============================================================================
        Eigen::VectorXd get_positions_lower_limits(){
            return _arm_robot->skeleton()->getPositionLowerLimits();
        }

        //==============================================================================
        double get_total_joints_motion(){
            return _total_joints_motion;
        }
        
        //==============================================================================
        double get_total_torque(){
            return _total_torque;
        }
        
        //==============================================================================
        Eigen::VectorXd get_final_pose(){
            return _end_effector_pose;
        }

        //==============================================================================
        double get_total_steps(){
            return _total_steps;
        }

    protected:
        robot_t _arm_robot;
        int _num_dofs;
        int _num_ctrl_dofs;
        robot_dart::RobotDARTSimuPtr _simu;
        std::vector<double> _ctrl;
        double _total_joints_motion;
        double _total_torque;
        Eigen::VectorXd _end_effector_pose;
        double _total_steps;



    };
} //namespace arm_dart

#endif
