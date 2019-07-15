#include <iostream>
#include <robot_dart/robot_dart_simu.hpp>

#include <robot_dart/control/pd_control.hpp>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif 

std::shared_ptr<robot_dart::Robot> random_box(size_t num = 0){
    // random pose 
    Eigen::Vector6d pose = Eigen::Vector6d::Random();
    
    // Assert a pose above the ground 
    pose(5) += 1.5;

    // Random size 
    Eigen::Vector3d size = Eigen::Vector3d::Random().array() * 
        Eigen::Vector3d(0.1, 0.2, 0.1).array() + 0.3;
    
    return robot_dart::Robot::create_box(size, pose, "free", 1., 
        dart::Color::Blue(1.0), "box_" + std::to_string(num));

}


int main(){
    // Create robot_dart simulator
    std::srand(std::time(NULL));
    // Setting timestep of 0.001 seconds 
    robot_dart::RobotDARTSimu simu(0.001);
    #ifdef GRAPHIC
        simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world()));
        // Set camera position (0,3,1) looking at the center (0,0,0)
        std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics()) -> 
            look_at({0., 3., 1.}, {0., 0., 0.});
    #endif 

        // Draw a floor with a square size of 10 meters and 0.2 meters of height  
        simu.add_floor(10., 0.2);

    // Add some random boxes/spheres 
    for (size_t i = 0; i < 3; i++){
        simu.add_robot(random_box(i));
    }
    // Add one robot with a controller 
    auto arm_robot = std::make_shared<robot_dart::Robot>("res/models/arm.urdf");

    // Pin the arm to the workd 
    arm_robot -> fix_to_world();
    arm_robot -> set_position_enforced(true);

    
    // Set of desired positions
    std::vector<double> ctrl = {0.0, 0.0, 0.0, 1.0};


    // Add a PD-controller to the arm 
    arm_robot -> add_controller(std::make_shared<robot_dart::control::PDControl>(ctrl));

    // Simulate the world 
    // Add the arm to the simulator
    simu.add_robot(arm_robot);

    // Run the simulator for 5 seconds 
    simu.run(5.);

    return 0; 

}