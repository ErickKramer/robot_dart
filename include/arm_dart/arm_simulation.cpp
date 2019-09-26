#include "arm_simulation.hpp"

namespace arm_dart {
    SchunkArm::SchunkArm(std::string urdf_path, 
        std::vector<std::pair<std::string, std::string>> packages, double time_step = 0.001, 
        std::string name="arm robot")
    {   
        // // Load robot 
        // auto arm_robot = std::make_shared<robot_dart::Robot>(urdf_path, packages,)

        // // Create Simulator 
        // robot_dart::RobotDARTSimu simu(time_step);
        // // Pin arm to the world 
        // arm_robot->fix_to_world();
        // arm_robot->set_position_enforced(true);

        // // Load graphics 
        // #ifdef GRAPHIC
        //     // Set graphics properties (world, resolution, shadows, real_time_node)
        //     simu.set_graphics(
        //         std::make_shared<robot_dart::graphics::Graphics>(simu.world(), 1920,1080,true,true));
            
        //     // Set Camera position
        //     std::static_pointer_cast<robot_dart::graphics::Graphics>(simu.graphics())->
        //         look_at({0.,3.,0.}, {0., 0., 0.5});
            
        // #endif

        // // Get DOFs of the robot
        // double num_dofs = arm_robot->skeleton()->getNumDofs();

        // // Get DOFs that can be controlled (identify mimic joints)
        // double mimic_joints = 0;
        // for (size_t i=0; i<num_dofs; i++){
        //     auto joint = arm_robot->skeleton()->getDof(i)->getJoint();
        //     if (joint->getActuatorType() == dart::dynamics::Joint::MIMIC){
        //         mimic_joints++;
        //     }
        // }

        // num_ctrl_dofs = static_cast<int>(num_dofs - mimic_joints);
    
    }

    SchunkArm::~SchunkArm(){

    }
}