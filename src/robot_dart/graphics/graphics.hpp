#ifndef ROBOT_DART_GRAPHICS_GRAPHICS_HPP
#define ROBOT_DART_GRAPHICS_GRAPHICS_HPP

#include <GLFW/glfw3.h>

#include <robot_dart/graphics/base_graphics.hpp>

// #include <dart/gui/osg/osg.hpp>

namespace robot_dart {
    namespace graphics {
        class Graphics : public BaseGraphics {
        public:
            Graphics() {}

            Graphics(dart::simulation::WorldPtr world)
            {
                _enabled = true;
                _world = world;
                // _osg_world_node = new dart::gui::osg::WorldNode(world);
                set_render_period(world->getTimeStep());
                // _osg_viewer.addWorldNode(_osg_world_node);
                if (!glfwInit())
                    _enabled = false;
                window = glfwCreateWindow(640, 480, "robot_dart Window", NULL, NULL);

                if (!window) {
                    glfwTerminate();
                    _enabled = false;
                }

                glfwMakeContextCurrent(window);
            }

            ~Graphics() { glfwTerminate(); }

            bool done() const override
            {
                // return _osg_viewer.done();
                return glfwWindowShouldClose(window);
            }

            void refresh() override
            {
                static int i = 0;

                if (!_enabled)
                    return;

                // if (!_osg_viewer.isRealized()) {
                //     _osg_viewer.setUpViewInWindow(0, 0, 640, 480);
                //     // if (Params::graphics::fullscreen())
                //     //     _osg_viewer.setUpViewOnSingleScreen();
                //     _osg_viewer.realize();
                // }

                // // process next frame
                // if (i % _render_period == 0)
                //     _osg_viewer.frame();
                if (i % _render_period == 0) {
                    /* Render here */
                    glClear(GL_COLOR_BUFFER_BIT);

                    /* Swap front and back buffers */
                    glfwSwapBuffers(window);

                    /* Poll for and process events */
                    glfwPollEvents();
                }
                i++;
            }

            void set_render_period(double dt) override
            {
                // we want to display at around 60Hz of simulated time
                _render_period = std::floor(0.015 / dt);
                if (_render_period < 1)
                    _render_period = 1;
            }

            void set_enable(bool enable) override
            {
                _enabled = enable;
            }

            void look_at(const Eigen::Vector3d& camera_pos,
                const Eigen::Vector3d& look_at = Eigen::Vector3d(0, 0, 0),
                const Eigen::Vector3d& up = Eigen::Vector3d(0, 0, 1))
            {
                _camera_pos = camera_pos;
                _look_at = look_at;
                _camera_up = up;

                // // set camera position
                // _osg_viewer.getCameraManipulator()->setHomePosition(
                //     osg::Vec3d(_camera_pos(0), _camera_pos(1), _camera_pos(2)), osg::Vec3d(_look_at(0), _look_at(1), _look_at(2)), osg::Vec3d(_camera_up(0), _camera_up(1), _camera_up(2)));
                // _osg_viewer.home();
            }

        protected:
            Eigen::Vector3d _look_at;
            Eigen::Vector3d _camera_pos;
            Eigen::Vector3d _camera_up;
            // osg::ref_ptr<dart::gui::osg::WorldNode> _osg_world_node;
            // dart::gui::osg::Viewer _osg_viewer;
            dart::simulation::WorldPtr _world;
            int _render_period;
            bool _enabled;

            GLFWwindow* window;
        };
    } // namespace graphics
} // namespace robot_dart

#endif
