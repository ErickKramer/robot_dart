#include <iostream>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif 

int main(){
    std::srand(std::time(NULL));

    // Creates simulation
    robot_dart::RobotDARTSimu simu(0.001);

    std::vector<std::pair<std::string, std::string>> packages = {{"fetch", 
        std::string(RESPATH) + "/models/meshes/fetch"}};

    #ifdef GRAPHIC
        // Generates the world
        simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world(), 1920,1080));

        // Speciy the location of the cameras
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics()) -> 
            look_at({0., 3., 1.}, {0., 0., 0.});
    #endif


    // Read the robot from the urdf
    auto dummy_robot = std::make_shared<robot_dart::Robot>("res/models/fetch.urdf", packages);

    simu.add_floor(10., 0.2);
    simu.world() -> getConstraintSolver() ->
        setCollisionDetector(dart::collision::FCLCollisionDetector::create());
    // Pin robot to the world
    dummy_robot -> fix_to_world();
    dummy_robot -> set_position_enforced(true);

    // Add robot to the simulator
    simu.add_robot(dummy_robot);



    // Runs simulator for 5 seconds
    simu.run(5.);
    return 0;
}