#include <iostream>
// #include <algorithm>
// #include <cstdlib>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>


#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif 
/*
    Descriptors:
        - Functors that are used to log information/data. 

        - In this case we log, state trajectories. 

        - The descriptor is called every desc_dump states of the simulation. 

        - 

 */
struct StateDesc : public robot_dart::descriptor::BaseDescriptor{
    StateDesc(robot_dart::RobotDARTSimu& simu, size_t desc_dump = 1) : 
        robot_dart::descriptor::BaseDescriptor(simu, desc_dump) {}

    void operator()(){
        if (_simu.robots().size()> 0){
            states.push_back(_simu.robots()[0]->skeleton()->getPositions());
        }
    }

    std::vector<Eigen::VectorXd> states;

};


int main(){
    // Create robot_dart simulator
    std::srand(std::time(NULL));
    
    // Setting timestep of 0.001 seconds 
    robot_dart::RobotDARTSimu simu(0.001);
    
    // Specify meshes packages
    std::vector<std::pair<std::string, std::string>> packages = {{"schunk", std::string(RESPATH) + "/models/meshes/lwa4d"}};

    // Create a robot from URDF and give a name to it
    auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/arm_schunk_without_collisions.urdf", packages);
    
    auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/arm_schunk_with_collisions.urdf", packages);
    
    // Pin the arm to the workd 
    arm_robot -> fix_to_world();
    arm_robot -> set_position_enforced(true);


    // Set of desired positions of the joints in radians
    std::vector<double> ctrl = {0., M_PI/2, 0., 0., 0., 0., 0.};

    // Add a PD-controller to the arm 
    arm_robot -> add_controller(std::make_shared<robot_dart::control::PDControl>(ctrl));
    
    // Set PD gains
    std::static_pointer_cast<robot_dart::control::PDControl>(arm_robot -> controllers()[0])
        -> set_pd(10., 1.);
   
   
    #ifdef GRAPHIC
        simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world()));
        // Set camera position (0,3,1) looking at the center (0,0,0)
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics()) -> 
            look_at({0., 3., 1}, {0., 0., 0.});
    #endif 

    // Draw a floor with a square size of 10 meters and 0.2 meters of height  
    // simu.add_floor(10., 0.2);

    // Simulate the world 
    // Add the arm to the simulator
    simu.add_robot(arm_robot);
    simu.add_descriptor(std::make_shared<StateDesc>(simu));

    Eigen::Vector3d size(0., 0., 0.);

    std::cout<< "Pose of the end effector?" << std::endl;
    std::cout<<(arm_robot -> body_trans("right_arm_ee_link") * size).transpose() << std::endl;

    // Run the simulator for 2 seconds 
    simu.run(2.);

    ctrl = {0., -M_PI/2, 0., 0., 0., 0., 0.};
    arm_robot -> controllers()[0] -> set_parameters(ctrl);
    simu.run(2.);
    
    std::cout<<(arm_robot -> body_trans("right_arm_ee_link") * size).transpose() << std::endl;
    
    ctrl = {0., 0, 0., 0., 0., 0., 0.};
    arm_robot -> controllers()[0] -> set_parameters(ctrl);
    simu.run(2.);
    
    std::cout<<(arm_robot -> body_trans("right_arm_ee_link") * size).transpose() << std::endl;
    
    std::cout<< std::static_pointer_cast<StateDesc>(simu.descriptor(0)) -> states.size() << std::endl;
    
    std::cout << (arm_robot -> name()) << std::endl;

    arm_robot.reset();

    return 0; 

}