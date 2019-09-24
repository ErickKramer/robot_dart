
#ifndef ROBOT_DART_CONTROL_PID_CONTROL
#define ROBOT_DART_CONTROL_PID_CONTROL

#include <Eigen/Core>
#include <utility>

#include <robot_dart/control/robot_control.hpp>

namespace robot_dart {
    namespace control {

        class PIDControl : public RobotControl {
        public:
            PIDControl();
            PIDControl(const std::vector<double>& ctrl, bool full_control = false);

            void configure() override;
            Eigen::VectorXd calculate(double) override;

            void set_pid(double p, double i, double d, double i_min, double i_max);
            void set_pid(const Eigen::VectorXd& p, const Eigen::VectorXd& i, const Eigen::VectorXd& d, double i_min, double i_max);

            std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> pid() const;

            std::shared_ptr<RobotControl> clone() const override;
        protected:
            Eigen::VectorXd _Kp;
            Eigen::VectorXd _Ki;
            Eigen::VectorXd _Kd;
            Eigen::VectorXd _pre_error;
            Eigen::VectorXd _integral;
            double _i_min;
            double _i_max;
        };
    } // namespace control
} // namespace robot_dart

#endif
