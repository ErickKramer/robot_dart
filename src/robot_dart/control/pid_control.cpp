#include "pid_control.hpp"
#include "robot_dart/robot.hpp"
#include "robot_dart/utils.hpp"

namespace robot_dart {
    namespace control {
        PIDControl::PIDControl() : RobotControl() {}
        PIDControl::PIDControl(const std::vector<double>& ctrl, bool full_control) : RobotControl(ctrl, full_control) {}

        void PIDControl::configure()
        {
            if (_ctrl.size() == _control_dof)
                _active = true;

            if (_Kp.size() == 0)
                set_pid(10., 0., 0.1, -1, 1);
        }

        Eigen::VectorXd PIDControl::calculate(double)
        {
            ROBOT_DART_ASSERT(_control_dof == _ctrl.size(), "PIDControl: Controller parameters size is not the same as DOFs of the robot", Eigen::VectorXd::Zero(_control_dof));
            auto robot = _robot.lock();
            Eigen::VectorXd target_positions = Eigen::VectorXd::Map(_ctrl.data(), _ctrl.size());

            Eigen::VectorXd current_positions = get_positions();
            auto time_step = robot->skeleton()->getTimeStep();
            // Eigen::VectorXd dq = get_velocities();

            // Calculate error
            Eigen::VectorXd error = target_positions.array() - current_positions.array();

            // Compute proportional term
            Eigen::VectorXd Pout = _Kp.array() * error.array();

            // Compute Integral term
            _integral = _integral.array() + error.array() * time_step; 
            Eigen::VectorXd Iout = _Ki.array() * _integral.array();

            // Constraint the I term 
            for (unsigned i = 0; i < Iout.size(); i++){
                if (Iout[i] > _i_max)
                    Iout[i] = _i_max;
                else if(Iout[i] < _i_min)
                    Iout[i] = _i_min;
            }
            //std::cout << "integral " << Iout.transpose() << std::endl;

            // Compute Derivative term 
            Eigen::VectorXd derivative = (error.array() - _pre_error.array()) / time_step;
            Eigen::VectorXd Dout = _Kd.array() * derivative.array();

            // Update previous error
            _pre_error = error;

            // Compute torque Commands 
            Eigen::VectorXd commands = Pout + Iout + Dout;

            // std::cout << "Target Positions " << std::endl;
            // std::cout << target_positions.transpose() << std::endl;
            // std::cout << "Current Positions " << std::endl;
            // std::cout << current_positions.transpose() << std::endl;
            // std::cout << "Current Vel " << std::endl;
            // std::cout << dq.transpose() << std::endl;
            // std::cout << "Commands " << std::endl;
            // std::cout << commands.transpose() << std::endl;
            // std::cout << "----" << std::endl;
            return commands;
        }

        void PIDControl::set_pid(double Kp, double Ki, double Kd, double i_min, double i_max)
        {
            ROBOT_DART_WARNING(_control_dof != 1, "PIDControl: Setting all the gains to Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd);
            _Kp = Eigen::VectorXd::Constant(_control_dof, Kp);
            _Ki = Eigen::VectorXd::Constant(_control_dof, Ki);
            _Kd = Eigen::VectorXd::Constant(_control_dof, Kd);
            _pre_error = Eigen::VectorXd::Zero(_control_dof);
            _integral = Eigen::VectorXd::Zero(_control_dof);
            _i_min = i_min;
            _i_max = i_max;
        }

        void PIDControl::set_pid(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Ki, const Eigen::VectorXd& Kd, double i_min, double i_max)
        {
            ROBOT_DART_ASSERT(static_cast<size_t>(Kp.size()) == _control_dof, "PIDControl: The Kp size is not the same as the DOFs!", );
            ROBOT_DART_ASSERT(static_cast<size_t>(Ki.size()) == _control_dof, "PIDControl: The Kp size is not the same as the DOFs!", );
            ROBOT_DART_ASSERT(static_cast<size_t>(Kd.size()) == _control_dof, "PIDControl: The Kd size is not the same as the DOFs!", );
            _Kp = Kp;
            _Ki = Ki;
            _Kd = Kd;
            _pre_error = Eigen::VectorXd::Zero(_control_dof);
            _integral = Eigen::VectorXd::Zero(_control_dof);
            _i_min = i_min;
            _i_max = i_max;
        }

        std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> PIDControl::pid() const
        {
            return std::make_tuple(_Kp, _Ki, _Kd);
        }

        std::shared_ptr<RobotControl> PIDControl::clone() const
        {
            return std::make_shared<PIDControl>(*this);
        }
    } // namespace control
} // namespace robot_dart
