#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include <fstream>
#include <iterator>
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

    return pose; 
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

    return velocities.size();

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

void display_run_results(robot_dart::RobotDARTSimu simu, double timestep){
    /*  Displays information relevant of the ran simulation 
        Args:
        - simu:
        - timestep: resolution of the simulation
    */
    // std::cout << "Pose" << compute_pose(frame_transform).transpose() << std::endl;
    std::cout << "Number of joints_states recorded " << 
        std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.size() << std::endl;
    std::cout << "Final joints configuration " << std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->
        joints_states.back().transpose() << std::endl;

    // Collectes recorded poses 
    std::vector<Eigen::VectorXd> poses = std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states;
    std::cout << "Number of pose_states recorded " << 
        poses.size() << std::endl;
    std::cout << "Pose of the end effector " << poses.back().transpose() << std::endl;

    // Collecte recorded velocities
    std::vector<Eigen::VectorXd> velocities = std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities;
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

    // ---------------- URDF ---------------------
    // Specify meshes packages
    std::vector<std::pair<std::string, std::string>> packages = {{"lwa4d", 
        std::string(RESPATH) + "/models/meshes/lwa4d"}};
    // std::vector<std::pair<std::string, std::string>> packages = {{"iiwa14", 
    //     std::string(RESPATH) + "/models/meshes/iiwa14"}};
    
    // Load URDF 
    // auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/arm_schunk_with_collisions.urdf", packages, "schunk lwa4d");
    auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/schunk_with_pg70.urdf", packages, "schunk lwa4d with PG70 gripper");
    // auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/iiwa14.urdf", packages, "iiwa14 arm");
    
    // ---------------- Simulator ---------------------
    // Create robot_dart simulator
    std::srand(std::time(NULL));
    double timestep = 0.0001;
    double simulation_time = 10.;
    // Setting timestep  
    robot_dart::RobotDARTSimu simu(timestep);
    
    // Pin the arm to the world 
    arm_robot->fix_to_world();
    arm_robot->set_position_enforced(true);

    #ifdef GRAPHIC
        // Specify the graphics for the simulator
        // Pass the world, resolution (widht, height)
        simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world(), 1920,1080, true, false));
        
        // Set camera position looking at the center 
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics())->
            look_at({0., 5., 1}, {0., 0., 0.});
    #endif

    // Get DOFs of the robot
    double numDOFs = arm_robot->skeleton()->getNumDofs();

    // Set of desired initial configuration
    std::vector<double> ctrl(numDOFs, 0.0);
    // std::vector<double> ctrl = {0., 0., 0., 0., 0., 0., 0.};
    std::cout << "Initial joint angles requested" << std::endl;
    for (auto c : ctrl) std::cout << c << " ";
    std::cout << std::endl;

    // Add a PD-controller to the arm 
    arm_robot->add_controller(std::make_shared<robot_dart::control::PDControl>(ctrl));

    // Set PD gains
    std::static_pointer_cast<robot_dart::control::PDControl>(arm_robot->controllers()[0])
    //   ->set_pd(300., 50.);
      ->set_pd(30., 10.);
    
    // Set Accelaration limits
    Eigen::VectorXd acc_upper_limit = Eigen::VectorXd::Ones(numDOFs)*0.01;
    Eigen::VectorXd acc_lower_limit = Eigen::VectorXd::Ones(numDOFs)*-0.01;
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
    std::cout << "Number of DoF: " << numDOFs << std::endl;
    std::cout << "Acceleration lower " << arm_robot -> skeleton() -> getAccelerationLowerLimits().transpose() << std::endl;
    std::cout << "Acceleration higher " << arm_robot -> skeleton() -> getAccelerationUpperLimits().transpose() << std::endl;
    std::cout << "Velocity lower " << arm_robot -> skeleton() -> getVelocityLowerLimits().transpose() << std::endl;
    std::cout << "Velocity higher " << arm_robot -> skeleton() -> getVelocityUpperLimits().transpose() << std::endl;
    std::cout<<"-----------------------------------"<<std::endl;
    
    // Run simulation 
    simu.run(simulation_time);
    display_run_results(simu, timestep);

    // Reset States vector
    std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();
    std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states.clear();
    std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities.clear();
    
    std::cout<<"-----------------------------------"<<std::endl;

    std::cout<<"Moving second joint pi/2 radians"<<std::endl;

    // Specify joint angle
    ctrl[1] = M_PI_2;
    ctrl[8] = 1;
    // ctrl[3] = M_PI_2;
    // ctrl[4] = M_PI_2;
    // ctrl[5] = M_PI_2;
    std::cout << "Joint angles requested" << std::endl;
    for (auto c : ctrl) std::cout << c << " ";
    std::cout << std::endl;

    // Set controller 
    arm_robot->controllers()[0]->set_parameters(ctrl);

    // Run simulation
    simu.run(simulation_time);
    display_run_results(simu, timestep);
    
    // Create a velocities log file to analyze the top velocity of the movement
    std::vector<Eigen::VectorXd> velocities = std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities;
    backup(velocities, "text", "velocities.txt");
    
    // Reset States vector
    std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();
    std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states.clear();
    std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities.clear();
    
    // std::cout<<"-----------------------------------"<<std::endl;

    // std::cout<<"Moving second joint -pi/2 radians"<<std::endl;

    // // Specify joint angle
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