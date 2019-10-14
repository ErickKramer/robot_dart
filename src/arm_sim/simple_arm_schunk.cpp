#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/control/pid_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include "robot_dart/utils.hpp"

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>

#include <fstream>
#include <iterator>
#include <chrono>
#include <unistd.h>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif

Eigen::VectorXd compute_pose(Eigen::Isometry3d link_transform);
// double movement_duration(std::vector<VectorXd> velocities, double timestep);



struct JointStateDesc : public robot_dart::descriptor::BaseDescriptor{
    // Descriptor used to record the joints states
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
    // Descriptor used to record the end_effector pose states
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
    // Descriptor used to record the joints velocities
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


Eigen::VectorXd compute_pose(Eigen::Isometry3d link_transform){
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

double movement_duration(std::vector<Eigen::VectorXd> velocities, double timestep){
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

double total_movement(Eigen::VectorXd init_conf, Eigen::VectorXd end_conf){
    /*
    Computes the amount of radians the arm moved from the initial configuration to 
    the final configuration
     */
    double total = 0;
    for (int i = 0; i < init_conf.size()-1; i++){
        total += abs(end_conf[i] - init_conf[i]);
    }
    return total;
}

void display_run_results(robot_dart::RobotDARTSimu simu, double timestep, std::vector<double>& ctrl, double joint_to_tune){
    /*  Displays information relevant of the ran simulation
        Args:
        - simu:
        - timestep: resolution of the simulation
    */
    std::cout << "Number of joints_states recorded " <<
        std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.size() << std::endl;
    std::vector<Eigen::VectorXd> poses = 
        std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states;
    std::cout << "Number of pose_states recorded " << poses.size() << std::endl;
    
    Eigen::VectorXd initial_configuration = robot_dart::Utils::round_small(
        std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.front()); 
    std::cout << "Initial joints configuration \n" << initial_configuration.transpose() << std::endl;

    Eigen::VectorXd end_configuration = robot_dart::Utils::round_small(
        std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.back()); 
    std::cout << "Final joints configuration \n" << end_configuration.transpose() << std::endl;
   
    Eigen::VectorXd target_positions = Eigen::VectorXd::Map(ctrl.data(), ctrl.size());

    // In the case that a joint is mimic (e.g. gripper fingers)
    if (target_positions.size() < end_configuration.size()){
        // Resize the target_positions vector used to control the robot to match the joints configuration vector
        target_positions.conservativeResize(end_configuration.size());
        // Copy the target position value of the last joint controlled to the mimic joint position
        target_positions[end_configuration.size()-1] = target_positions[end_configuration.size()-2];
    } 

    std::cout << "Joint angles requested " << target_positions.transpose() << std::endl;
    
    std::cout << "Error for the arm configuration \n" << (target_positions - end_configuration)[joint_to_tune] << std::endl;

    std::cout << "Pose of the end effector \n " << poses.back().transpose() << std::endl;

    std::cout << "Total arm movement " << total_movement(initial_configuration, end_configuration) << std::endl;

    // Collect recorded velocities
    std::vector<Eigen::VectorXd> velocities = 
        std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities;
    // backup(velocities, "text", "velocities.txt");

    // Computes movement duration
    double duration = movement_duration(velocities, timestep);
    std::cout << "Arm stoped at " << duration << " seconds " << std::endl;
}

void backup(const std::vector<Eigen::VectorXd> v, std::string file_type, std::string file_name){
    /*
    Stores an Eigen vector into a file.
    Args:
        - v: Vector desired to store
        - file_type: Type of file desired
        - file_name: Name desired for the file
    */

    if (file_type == "text"){
        std::ofstream outFile(file_name);
        for (const auto &element : v){
            outFile << element.transpose() << "\n";
        }
    }

}


int main(){

    // std::string system = "Home";
    std::string system = "FKIE";

    // ---------------- URDF ---------------------
    // Specify meshes packages
    std::vector<std::pair<std::string, std::string>> packages = {{"lwa4d",
        std::string(RESPATH) + "/models/meshes/lwa4d"}};
    // std::vector<std::pair<std::string, std::string>> packages = {{"iiwa14",
    //     std::string(RESPATH) + "/models/meshes/iiwa14"}};

    // Load URDF
    // auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/arm_schunk_with_collisions.urdf", packages, "schunk lwa4d");
    auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/schunk_with_pg70.urdf", packages, "schunk lwa4d with PG70 gripper");
    // auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/schunk_with_fetch.urdf", packages, "schunk lwa4d with PG70 gripper");
    // auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/iiwa14.urdf", packages, "iiwa14 arm");

    // ---------------- Simulator ---------------------
    // Create robot_dart simulator
    std::srand(std::time(NULL));
    double timestep = 0.010;
    double simulation_time = 10.;
    // Setting timestep
    robot_dart::RobotDARTSimu simu(timestep);

    // Pin the arm to the world
    arm_robot->fix_to_world();
    arm_robot->set_position_enforced(true);

    #ifdef GRAPHIC
        // Specify the graphics for the simulator
        // Pass the world, resolution (widht, height)
        if (system == "Home"){
            // Resolution screen of personal Asus
            simu.set_graphics(
                std::make_shared<robot_dart::graphics::Graphics>(simu.world(), 1920,1080, true, true));

        }
        else if(system == "FKIE"){
            std::cout<<"Running 1920x1200 resolution " << std::endl;
            // Resolution screen at work 
            simu.set_graphics(
                std::make_shared<robot_dart::graphics::Graphics>(simu.world(), 1920,1200, true, true));
        }
        else{
            // Generic resolution
            simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world()));
        }

        // Set camera position looking at the center
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics())->
            look_at({3., 1., 0.}, {0., 0., 0.5});
    #endif

    // Get DOFs of the robot
    double num_dofs = arm_robot->skeleton()->getNumDofs();

    // Get DOFs that are controllable (not mimic)
    double mimic_joints = 0;
    for (size_t i = 0; i < num_dofs ; i++){
        auto joint = arm_robot->skeleton()->getDof(i)->getJoint();
        if (joint->getActuatorType() == dart::dynamics::Joint::MIMIC){
            mimic_joints++;
        }
    }
    int num_ctrl_dofs = static_cast<int>(num_dofs - mimic_joints);    

    // Set of desired initial configuration
    // std::vector<double> ctrl(num_dofs, 0.0);
    std::vector<double> ctrl(num_ctrl_dofs, 0.0);

    // Set values for the gripper (Close position)
    // ctrl[1] = M_PI_2;
    // ctrl[8] = -0.029;

    // Add a PD-controller to the arm
    // arm_robot->add_controller(std::make_shared<robot_dart::control::PDControl>(ctrl));
    
    // Add a PID Controller for the arm  
    arm_robot->add_controller(std::make_shared<robot_dart::control::PIDControl>(ctrl));
    
    // PID_params
    // Computes Kp Vector
    /*
    Joints: 
    - 0: Rolling : Set
    - 1: Flexing : Set
    - 2: Rolling : Set 
    - 3: Flexing : Set
    - 4: Rolling : Set 
    - 5: Flexing : Set 
    - 6: Rolling : Set 
    - 7: Left finger : Set 
    - 8: Right finger : Not Set  (Mimic)
    */

    // double joint_to_tune = 3;
    double joint_to_tune;

    // Load PID Params  
    Eigen::VectorXd Kp = Eigen::VectorXd::Ones(num_ctrl_dofs); 
    Eigen::VectorXd Ki = Eigen::VectorXd::Ones(num_ctrl_dofs); 
    Eigen::VectorXd Kd = Eigen::VectorXd::Ones(num_ctrl_dofs); 
    double i_min;
    double i_max;
    std::ifstream pid_file("res/pid_params.txt");
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
        }else if (params[0] == "joint_to_tune"){
            joint_to_tune = atof(params[1].c_str());
        }

    }
    pid_file.close();

    // Set PD gains
    // std::static_pointer_cast<robot_dart::control::PDControl>(arm_robot->controllers()[0])
    //   ->set_pd(Kp, Kd);

    // Set PD gains
    std::static_pointer_cast<robot_dart::control::PIDControl>(arm_robot->controllers()[0])
      ->set_pid(Kp, Ki, Kd, i_min, i_max);
    
    // Set Accelaration limits
    Eigen::VectorXd acc_upper_limit = Eigen::VectorXd::Ones(num_dofs)*0.01;
    Eigen::VectorXd acc_lower_limit = Eigen::VectorXd::Ones(num_dofs)*-0.01;
    arm_robot->skeleton()->setAccelerationUpperLimits(acc_upper_limit);
    arm_robot->skeleton()->setAccelerationLowerLimits(acc_lower_limit);

    // Specify collision detection
    simu.world()->getConstraintSolver()->
        setCollisionDetector(dart::collision::FCLCollisionDetector::create());

    // Add robot to the simulator
    simu.add_robot(arm_robot);

    // Add descriptors for the simulator
    simu.add_descriptor(std::make_shared<JointStateDesc>(simu));
    simu.add_descriptor(std::make_shared<PoseStateDesc>(simu));
    simu.add_descriptor(std::make_shared<JointVelDesc>(simu));

    // Display arm information
    std::cout << "Arm: " << arm_robot->name() << std::endl;
    std::cout << "Number of DoF: " << num_dofs << std::endl;
    std::cout << "Acceleration lower " << arm_robot -> skeleton() -> getAccelerationLowerLimits().transpose() << std::endl;
    std::cout << "Acceleration higher " << arm_robot -> skeleton() -> getAccelerationUpperLimits().transpose() << std::endl;
    std::cout << "Velocity lower " << arm_robot -> skeleton() -> getVelocityLowerLimits().transpose() << std::endl;
    std::cout << "Velocity higher " << arm_robot -> skeleton() -> getVelocityUpperLimits().transpose() << std::endl;
    std::cout << "Position upper lim " << arm_robot -> skeleton() -> getPositionUpperLimits().transpose() << std::endl;
    std::cout << "Position lower lim " << arm_robot -> skeleton() -> getPositionLowerLimits().transpose() << std::endl;
    std::cout<<"-----------------------------------"<<std::endl;

    // std::cout << "Initial joint angles requested" << std::endl;
    // for (auto c : ctrl) std::cout << c << " ";
    // std::cout << std::endl;
    
    // auto start = std::chrono::steady_clock::now();
    
    // Run simulation
    // simu.run(simulation_time/4);

    // auto end = std::chrono::steady_clock::now();

    // display_run_results(simu, timestep, ctrl);
    
    // std::cout << "Simulation ran for " << 
    //     std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() 
    //         << " ms " <<std::endl;

    // std::cout << "Simulation ran for " << 
    //     std::chrono::duration_cast<std::chrono::seconds>(end - start).count() 
    //         << " sec " <<std::endl;

    // Reset States vector
    // std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();
    // std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states.clear();
    // std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities.clear();

    std::cout<<"-----------------------------------"<<std::endl;


    if (joint_to_tune == 7){
        std::cout<<"Opening gripper " << std::endl;
        ctrl[joint_to_tune] = 0.033;
    }
    else{
        std::cout<<"Moving joint " << joint_to_tune << " " << M_PI_2 << " radians"<<std::endl;
        ctrl[joint_to_tune] = M_PI_2;
    }

    // ctrl[0] = M_PI_2;
    // ctrl[1] = -M_PI_2;
    // ctrl[2] = M_PI_2;
    // ctrl[3] = -M_PI_2;
    // ctrl[4] = M_PI_2;
    // ctrl[5] = -M_PI_2;
    // ctrl[6] = M_PI_2;
    // ctrl[7] = 0.033;
    // ctrl[joint_to_tune] = 0.033;

    // Set controller
    arm_robot->controllers()[0]->set_parameters(ctrl);

    // Run simulation
    simu.run(simulation_time);
    display_run_results(simu, timestep, ctrl, joint_to_tune);

    // Create a velocities log file to analyze the top velocity of the movement
    std::vector<Eigen::VectorXd> velocities = std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities;
    backup(velocities, "text", "velocities_to_test.txt");

    // Reset States vector
    std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();
    std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states.clear();
    std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities.clear();

    std::cout<<"-----------------------------------"<<std::endl;

    // // ctrl[joint_to_tune] = 0;
    // std::vector<double> initial_conf(num_ctrl_dofs, 0.0);
    // ctrl = initial_conf;

    // // Set controller
    // arm_robot->controllers()[0]->set_parameters(ctrl);

    // // Run simulation
    // simu.run(simulation_time);
    // display_run_results(simu, timestep, ctrl);

    // // Create a velocities log file to analyze the top velocity of the movement
    // // velocities = std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities;
    // // backup(velocities, "text", "velocities_to_initial.txt");

    // // Reset States vector
    // std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();
    // std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states.clear();
    // std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities.clear();

    // std::cout<<"-----------------------------------"<<std::endl;

    // std::cout<<"Moving second joint -pi/2 radians"<<std::endl;

    // Specify joint angle
    // ctrl[1] = -M_PI_2;
    // ctrl[3] = -M_PI_2;
    // ctrl[5] = -M_PI_2;
    // std::cout << "Joint angles requested" << std::endl;
    // for (auto c : ctrl) std::cout << c << " ";
    // std::cout << std::endl;

    // // Set controller
    // arm_robot->controllers()[0]->set_parameters(ctrl);

    // // Run simulation
    // simu.run(simulation_time);
    // display_run_results(simu, timestep);

    // // Reset States vector
    // std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();
    // std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states.clear();
    // std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities.clear();

    arm_robot.reset();
    return 0;

}

// NOTE: REFACTOR
// void display_link_info(Eigen::Isometry3d link_transform){

//     /*
//         Displays the following information from the transform of a link:
//         - Homogeneous Transformation matrix
//         - Rotation matrix
//         - Translation matrix
//         - Quaternion
//         - Euler angles
//         - Pose
//     */

//     Computes rotation matrix
//     Eigen::Matrix3d rot_matrix = link_transform.rotation();
//     // Computes Homogeneous transformation matrix
//     Eigen::Matrix4d transformation_matrix = link_transform.matrix();
//     // Computes Quaternions
//     Eigen::Quaterniond quat_end(rot_matrix);
//     // Computes Euler angles
//     Eigen::Vector3d euler = rot_matrix.eulerAngles(2,1,0);
//     // Computes translation vector
//     Eigen::VectorXd translation = link_transform.translation();
//     // Computes pose vector
//     Eigen::VectorXd pose(link_transform.translation().size() + euler.size());
//     pose << translation, euler;
//     Computes an axis angle based on the quaternions
//     https://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html
//     Eigen::AngleAxisd angle_axis(quat_end);


//     std::cout << "Homogeneous transformation matrix" << std::endl;
//     std::cout << transformation_matrix << std::endl;

//     std::cout << "\n Rotation matrix" << std::endl;
//     std::cout << rot_matrix << std::endl;

//     std::cout << "\n Translation vector" << std::endl;
//     std::cout << translation << std::endl;

//     std::cout << "\n Quaternion" << std::endl;
//     std::cout << "w: " << quat_end.w() << " x: " << quat_end.x() <<
//                  " y: " << quat_end.y() << " z: " << quat_end.z() << std::endl;

//     std::cout << "\n Euler angles" << std::endl;
//     std::cout << "Yaw: " << euler[0] << " Pitch: " << euler [1] << " Roll: " <<
//                  euler[2] << std::endl;

//     std::cout << "\n Pose" << std::endl;
//     std::cout << compute_pose(link_transform).transpose() << std::endl;

//     std::cout << "\n Axis angle" << std::endl;
//     std::cout << angle_axis.axis().transpose() << std::endl;
//     std::cout<< angle_axis.angle() << std::endl;
// }
