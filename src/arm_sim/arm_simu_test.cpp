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
    std::cout<<"Current working directory is " << cCurrentPath << std::endl;
    boost::filesystem::path cur_path = cCurrentPath;

    // Print info
    std::cout << "parent path  " << cur_path.parent_path() << std::endl;

    // -----------------------------------------------------------
    // Simulation Definition
    // -----------------------------------------------------------
    // Defining URDF
    std::vector<std::pair<std::string, std::string>> packages = {{"lwa4d",
        cur_path.parent_path().string() + "/robot_dart/res/models/meshes/lwa4d"}};
    std::string urdf_path = cur_path.parent_path().string() 
        + "/robot_dart/res/models/schunk_with_pg70.urdf";
    std::string name = "schunk arm";
    double time_step = 0.001;

    arm_dart::SchunkArm simu(urdf_path, packages,name,time_step);

    std::string pid_file_path = cur_path.parent_path().string() 
        + "/robot_dart/res/pid_params.txt"; 
    
    simu.set_controller(pid_file_path);
    return 0;
}
