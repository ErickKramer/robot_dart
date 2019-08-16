#include <iostream>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif 

int main(){
    std::srand(std::time(NULL));

    // Creates simulation
    robot_dart::RobotDARTSimu simu(0.001);

    // Indicated the path where the meshes files are located 
    // std::vector<std::pair<std::string, std::string>> packages = {{"fetch", 
    //     std::string(RESPATH) + "/models/meshes/fetch"}};
    
    // std::vector<std::pair<std::string, std::string>> packages = {{"iiwa14", 
    //     std::string(RESPATH) + "/models/meshes/iiwa14"}};

    std::vector<std::pair<std::string, std::string>> packages = {{"schunk", 
        std::string(RESPATH) + "/models/meshes/lwa4d"}};

    #ifdef GRAPHIC
        // Generates the world
        simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world(), 1920,1080));

        // Speciy the location of the cameras
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics()) -> 
            look_at({0., 3., 1.}, {0., 0., 0.});
    #endif


    // Read the robot from the urdf
    // auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/fetch_arm.urdf", packages, "dummy_robot");
    // auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/iiwa14.urdf", packages, "dummy_robot");
    auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/arm_schunk_with_collisions.urdf", packages, "dummy_robot");

    // simu.add_floor(10., 0.2);

    // Load collision moduled
    simu.world() -> getConstraintSolver() ->
        setCollisionDetector(dart::collision::FCLCollisionDetector::create());
    
    // Pin robot to the world
    dummy_robot -> fix_to_world();
    dummy_robot -> set_position_enforced(true);

    double numDofs = dummy_robot -> skeleton() -> getNumDofs();

    // std::vector<double> ctrl()
    
    // Display the name of the robot
    std::cout << "Simulating the robot " << dummy_robot -> name() << std::endl;

    // Display the number of DoF
    std::cout << "Number of DoF " << dummy_robot -> skeleton() -> getNumDofs() << std::endl;

    // Add robot to the simulator
    simu.add_robot(dummy_robot);

    // Set the position of a single joint
    // dummy_robot -> skeleton() -> getDof(1) -> setPosition(M_PI_4);

    simu.run(5.);

    // Vector to move the position of the arm in the world
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();

    // Translate the base robot 
    tf.translation() = Eigen::Vector3d(0, 0.0, 0.0);
    
    // Rotate the base of the robot
    tf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY()));

    // dummy_robot -> skeleton() -> setTransformFromParentBodyNode(tf);
    dummy_robot -> skeleton() -> getRootBodyNode() -> getParentJoint() ->
        setTransformFromParentBodyNode(tf);


    // Runs simulator for 5 seconds
    simu.run(5.);
    return 0;
}