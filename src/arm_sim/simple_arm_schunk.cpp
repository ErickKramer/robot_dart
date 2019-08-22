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
    // Descriptor used to log the joints states
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
    // Descriptor used to log the end_effector pose states 
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
    // Descriptor used to log the joints states
    JointVelDesc(robot_dart::RobotDARTSimu& simu, size_t desc_dump = 1) : 
        robot_dart::descriptor::BaseDescriptor(simu, desc_dump) {}

    void operator()(){
        // Stores the joint positions
        if (_simu.robots().size() > 0){
            // Add current joints configuration to the joints_states vector
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

void backup(const std::vector<Eigen::VectorXd> v, std::string file_type, std::string file_name){


    if (file_type == "text"){
        std::ofstream outFile(file_name);
        for (const auto &element : v){
            outFile << element.transpose() << "\n";
        }
    }

}


int main(){
    // Create robot_dart simulator
    std::srand(std::time(NULL));

    double timestep = 0.001;
    double simulation_time = 3.;
    
    // Setting timestep of 0.001 seconds 
    robot_dart::RobotDARTSimu simu(timestep);
    
    // Specify meshes packages
    std::vector<std::pair<std::string, std::string>> packages = {{"schunk", std::string(RESPATH) + "/models/meshes/lwa4d"}};

    // Create a robot from URDF specifying where the stl files are located
    // auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/arm_schunk_without_collisions.urdf", packages);
    auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/arm_schunk_with_collisions.urdf", packages, "schunk lwa4d");
    
    // Pin the arm to the workd 
    arm_robot->fix_to_world();
    arm_robot->set_position_enforced(true);


    // Set of desired positions of the joints in radians
    std::vector<double> ctrl = {0., M_PI_2, 0., 0., 0., 0., 0.};

    // Add a PD-controller to the arm 
    arm_robot->add_controller(std::make_shared<robot_dart::control::PDControl>(ctrl));
    
    // Set PD gains
    std::static_pointer_cast<robot_dart::control::PDControl>(arm_robot->controllers()[0])
      ->set_pd(10., 1.);
   
   
    // Specify collision detection
    simu.world()->getConstraintSolver()->
        setCollisionDetector(dart::collision::FCLCollisionDetector::create());

    #ifdef GRAPHIC
        // Specify the graphics for the simulator
        // Pass the world, resolution (widht, height)
        
        simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world(), 1920,1080));
        // Set camera position (0,3,1) looking at the center (0,0,0)
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics())->
            look_at({0., 5., 1}, {0., 0., 0.});
    #endif 

    // Draw a floor with a square size of 10 meters and 0.2 meters of height  
    // simu.add_floor(10., 0.2);

    // Add robot to the simulator
    simu.add_robot(arm_robot);

    // Add descriptors for the simulator
    simu.add_descriptor(std::make_shared<JointStateDesc>(simu));
    simu.add_descriptor(std::make_shared<PoseStateDesc>(simu));
    simu.add_descriptor(std::make_shared<JointVelDesc>(simu));
    
    std::cout << "Arm: " << arm_robot->name() << std::endl;
    std::cout << "Number of DoF: " << arm_robot->skeleton()->getNumDofs() << std::endl;
    std::cout<<"-----------------------------------"<<std::endl;
    
    // Run the simulator for 2 seconds 
    simu.run(simulation_time);
    
    //Transform docs->https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html
    Eigen::Isometry3d frame_transform;
    frame_transform = arm_robot->skeleton()->getBodyNode("right_arm_ee_link")->getWorldTransform();

    std::cout<<"Moving second joint pi/2 radians"<<std::endl;

    std::cout << "Pose" << compute_pose(frame_transform).transpose() << std::endl;

    std::cout << "Number of joints_states recorded " << 
        std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.size() << std::endl;

    std::cout << "Number of pose_states recorded " << 
        std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states.size() << std::endl;

    std::vector<Eigen::VectorXd> poses = std::static_pointer_cast<PoseStateDesc>(simu.descriptor(1))->pose_states;
    
    std::cout << "Last pose recorded " << poses.back().transpose() << std::endl;

    //Reset joints_states vector 
    std::cout << "Joint configuration " << std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->
        joints_states.back().transpose() << std::endl;
    std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();

    // Backup the stored poses in a text file
    // backup(poses, "text", "poses.txt");

    std::vector<Eigen::VectorXd> velocities = std::static_pointer_cast<JointVelDesc>(simu.descriptor(2))->joints_velocities;
    backup(velocities, "text", "velocities.txt");

    double duration = movement_duration(velocities, timestep);
    std::cout << "Arm stoped at " << duration << " seconds" << std::endl;

    // Eigen::VectorXd last_velocity = velocities.back(); 
    // std::cout << last_velocity.isZero(1e-3) << std::endl;
    
    // std::cout << "Velocities " << arm_robot->skeleton()->getVelocities() << std::endl;
     
    std::cout<<"-----------------------------------"<<std::endl;

    ctrl = {0., -M_PI_2, 0., 0., 0., 0., 0.};
    arm_robot->controllers()[0]->set_parameters(ctrl);
    simu.run(simulation_time);
    
    frame_transform = arm_robot->skeleton()->getBodyNode("right_arm_ee_link")->getWorldTransform();
    
    std::cout<<"Moving second joint -pi/2 radians"<<std::endl;
    
    std::cout << "Pose" << compute_pose(frame_transform).transpose() << std::endl;
    
    std::cout << "Number of joints_states recorded " << 
        std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.size() << std::endl;

    std::cout << "Joint configuration " << std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->
        joints_states.back().transpose() << std::endl;
    //Reset joints_states vector 
    std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();
    std::cout<<"-----------------------------------"<<std::endl;
    
    // ctrl = {0., 0, 0., 0., 0., 0., 0.};
    // arm_robot->controllers()[0]->set_parameters(ctrl);
    // simu.run(simulation_time);
   
    // frame_transform = arm_robot->skeleton()->getBodyNode("right_arm_ee_link")->getWorldTransform();
    
    // std::cout<<"Moving second joint 0 radians"<<std::endl;
    // display_link_info(frame_transform);
    // std::cout << "Expected number of joints_states = " << simulation_time/timestep << std::endl;
    // std::cout << "Number of joints_states recorded " << 
    //     std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.size() << std::endl;
    // std::cout << "Joint configuration " << std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->
    //     joints_states.back().transpose() << std::endl;
    // //Reset joints_states vector 
    // std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();

    // std::cout<<"-----------------------------------"<<std::endl;
    
    // ctrl = {M_PI_2, M_PI_2, 0., 0., 0., 0., 0.};
    // arm_robot->controllers()[0]->set_parameters(ctrl);
    // simu.run(simulation_time);
   
    // frame_transform = arm_robot->skeleton()->getBodyNode("right_arm_ee_link")->getWorldTransform();
    
    // std::cout<<"Moving first joint pi/2 and second joint pi/2 radians"<<std::endl;
    // display_link_info(frame_transform);
    // std::cout << "Expected number of joints_states = " << simulation_time/timestep << std::endl;
    // std::cout << "Number of joints_states recorded " << 
    //     std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.size() << std::endl;
    // std::cout << "Joint configuration " << std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->
    //     joints_states.back().transpose() << std::endl;
    // //Reset joints_states vector 
    // std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();

    // std::cout<<"-----------------------------------"<<std::endl;
    
    // ctrl = {M_PI_2, -M_PI_2, 0., 0., 0., 0., 0.};
    // arm_robot->controllers()[0]->set_parameters(ctrl);
    // simu.run(simulation_time);
   
    // frame_transform = arm_robot->skeleton()->getBodyNode("right_arm_ee_link")->getWorldTransform();
    
    // std::cout<<"Moving first joint pi/2 and second joint -pi/2 radians"<<std::endl;
    // display_link_info(frame_transform);
    // std::cout << "Expected number of joints_states = " << simulation_time/timestep << std::endl;
    // std::cout << "Number of joints_states recorded " << 
    //     std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.size() << std::endl;
    // std::cout << "Joint configuration " << std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->
    //     joints_states.back().transpose() << std::endl;
    // //Reset joints_states vector 
    // std::static_pointer_cast<JointStateDesc>(simu.descriptor(0))->joints_states.clear();

    arm_robot.reset();
   
    return 0; 

}