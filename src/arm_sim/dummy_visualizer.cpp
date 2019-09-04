#include <iostream>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif 

#include <robot_dart/robot.hpp>

int main(){
    std::srand(std::time(NULL));

    double timestep = 0.001;
    double sim_duration = 10.;
    // Creates simulation
    robot_dart::RobotDARTSimu simu(timestep);

    // Indicated the path where the meshes files are located 
    // std::vector<std::pair<std::string, std::string>> packages = {{"fetch", 
    //     std::string(RESPATH) + "/models/meshes/fetch"}};
    
    // std::vector<std::pair<std::string, std::string>> packages = {{"iiwa14", 
    //     std::string(RESPATH) + "/models/meshes/iiwa14"}};

    std::vector<std::pair<std::string, std::string>> packages = {{"lwa4d", 
        std::string(RESPATH) + "/models/meshes/lwa4d"}};
    
    // std::vector<std::pair<std::string, std::string>> packages = {{"telerob", 
    //     std::string(RESPATH) + "/models/meshes/telerob"}};

    #ifdef GRAPHIC
        // Generates the world
        simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world(), 1920,1080, true, false));

        // Speciy the location of the cameras
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics())->
            look_at({0., 5., 1.}, {0., 0., 0.});
        
        // Pass instruction message to the viewer 
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics())->
            add_instruction("Visualization of URDFs \n");
        
        // Display instructions
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics())->
            display_instructions();
    #endif


    // Read the robot from the urdf
    // auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/fetch_arm.urdf", packages, "fetch arm");
    // auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/iiwa14.urdf", packages, "iiwa14 arm");
    // auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/arm_schunk_with_collisions.urdf", packages, "schunk lwa4d arm");
    // auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/telerob.urdf", packages, "telerob arm");
    auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/schunk_with_pg70.urdf", packages, "schunk lwa4d arm");
    // auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/schunk_with_fetch.urdf", packages, "schunk lwa4d arm");

    // simu.add_floor(10., 0.2);

    // Load collision moduled
    simu.world()->getConstraintSolver()->
        setCollisionDetector(dart::collision::FCLCollisionDetector::create());
    
    // Pin robot to the world
    dummy_robot->fix_to_world();
    dummy_robot->set_position_enforced(true);
    
    // Display the name of the robot
    std::cout << "Simulating the robot " << dummy_robot->name() << std::endl;

    // Display the number of DoF
    std::cout << "Number of DoF " << dummy_robot->skeleton()->getNumDofs() << std::endl;

    std::cout << "Limits of the robot " << std::endl;
    std::cout << "------------------------------------- " << std::endl;
    std::cout << "Position lower limits " << dummy_robot->skeleton()->getPositionLowerLimits().transpose() << std::endl;
    std::cout << "Position higher limits " << dummy_robot->skeleton()->getPositionUpperLimits().transpose() << std::endl;
    std::cout << "Velocity lower limits " << dummy_robot->skeleton()->getVelocityLowerLimits().transpose() << std::endl;
    std::cout << "Velocity higher limits " << dummy_robot->skeleton()->getVelocityUpperLimits().transpose() << std::endl;
    std::cout << "------------------------------------- " << std::endl;

    simu.add_robot(dummy_robot);
    simu.run(sim_duration);

    // Vector to move the position of the arm in the world
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

    // Translate the base robot 
    tf.translation() = Eigen::Vector3d(0.5, 0.5, 0.0);
    
    // Rotate the base of the robot
    tf.rotate(Eigen::AngleAxisd(-M_PI_4, Eigen::Vector3d::UnitY()));

    // Move the base of the arm
    dummy_robot->set_base_pose(tf);

    // Eigen::VectorXd robot_positions = dummy_robot->skeleton()->getPositions();
    // Eigen::VectorXd robot_velocities = dummy_robot->skeleton()->getVelocities();

    // std::cout << "Robot positions " << robot_positions.transpose() << std::endl;
    // std::cout << "Robot velocities " << robot_velocities.transpose() << std::endl;

    // Get the mass matrix of the arm
    // Eigen::MatrixXd mass_matrix = dummy_robot->skeleton()->getMassMatrix();
    // std::cout << "Mass Matrix \n" << mass_matrix << std::endl;
    // std::cout << "Mass of the arm " << dummy_robot->skeleton()->getMass() << std::endl;


    // Runs simulator
    simu.run(sim_duration);
    return 0;
}