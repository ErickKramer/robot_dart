#include <iostream>
#include <cstdlib>
#include <arm_dart/arm_simulation.hpp>
#include <boost/filesystem.hpp>
#define GetCurrentDir getcwd

int main()
{
    // Get Working directory
    char cCurrentPath[FILENAME_MAX];
    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))){
        return errno;
    }
    // cCurrentPath[sizeof(cCurrentPath)-1] = '\0';
    // std::cout<<"Current working directory is " << cCurrentPath << std::endl;
    boost::filesystem::path cur_path = cCurrentPath;

    // Print info
    // std::cout << "parent path  " << cur_path.parent_path() << std::endl;

    // Defining URDF
    std::vector<std::pair<std::string, std::string>> packages = {{"lwa4d",
        cur_path.parent_path().string() + "/robot_dart/res/models/meshes/lwa4d"}};
    std::string urdf_path = cur_path.parent_path().string() 
        + "/robot_dart/res/models/schunk_with_pg70.urdf";
    std::string name = "schunk arm";
    double time_step = 0.001;

    std::string end_effector_name = "end_virtual_link";

    // Instance of SchunkArm class
    arm_dart::SchunkArm arm_simu(urdf_path, packages,name, end_effector_name);

    // Initialize simulation
    arm_simu.init_simu(time_step);

    // Initialize PID Controller
    std::string pid_file_path = cur_path.parent_path().string() 
        + "/robot_dart/res/pid_params.txt"; 
    arm_simu.init_controller(pid_file_path);

    // Set Acceleration limits
    arm_simu.set_acceleration_limits(0.01);

    // Set descriptors to analyze the simulation
    std::vector<std::string> descriptors = {"joint_states", "pose_states","velocity_states"};
    arm_simu.set_descriptors(descriptors);

    // Display robot information
    arm_simu.display_robot_info();

    // Run simulation
    double simulation_time = 10.;
    // arm_simu.run_simu(simulation_time/4);

    // Reset states descriptors
    // arm_simu.reset_descriptors(descriptors);

    std::vector<double> conf(arm_simu.get_control_dofs(), 0.0);
    conf[1] = M_PI_4;
    conf[7] = 0.033;

    arm_simu.set_goal_configuration(conf);
    arm_simu.run_simu(simulation_time);

    //--------------------------------------------------------------------------
    // Display simulation results
    //--------------------------------------------------------------------------
    std::cout << "Pose of the end effector \n " << arm_simu.get_final_pose().transpose() << std::endl;

    std::cout << "Total arm movement " << arm_simu.get_total_joints_motion()<< std::endl;
    
    std::cout << "Total torque " << arm_simu.get_total_torque()<< std::endl;
    
    std::cout << "Number of time steps required to reach target configuration " 
        << arm_simu.get_total_steps() << std::endl;

    arm_simu.reset_descriptors(descriptors);
    arm_simu.reset_configuration();

    conf[1] = -M_PI_4;
    conf[7] = 0.;

    arm_simu.set_goal_configuration(conf);
    arm_simu.run_simu(simulation_time);
    std::cout << "Pose of the end effector \n " << arm_simu.get_final_pose().transpose() << std::endl;

    std::cout << "Total arm movement " << arm_simu.get_total_joints_motion()<< std::endl;
    
    std::cout << "Total torque " << arm_simu.get_total_torque()<< std::endl;
    
    std::cout << "Number of time steps required to reach target configuration " 
        << arm_simu.get_total_steps() << std::endl;
    arm_simu.reset_descriptors(descriptors);

    arm_simu.reset_robot();
    return 0;
}
