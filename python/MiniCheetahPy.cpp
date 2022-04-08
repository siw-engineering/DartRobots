#include "DartRobots/MiniCheetah.hpp"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <spdlog/spdlog.h>

namespace py = pybind11;
using namespace DartRobots;

PYBIND11_MODULE(MiniCheetahPy, m)
{
    py::class_<MiniCheetahConfig>(m, "Config")
        .def(py::init<>())
        .def_readwrite("spawn_pos", &MiniCheetahConfig::spawnPos)
        .def_readwrite("spawn_orientation", &MiniCheetahConfig::spawnOrientation)
        .def_readwrite("spawn_joint_pos", &MiniCheetahConfig::spawnJointPos)
        .def(py::pickle(
            [](const MiniCheetahConfig &config) { // __getstate__
                return py::make_tuple(config.spawnPos, config.spawnOrientation, config.spawnJointPos);
            },
            [](py::tuple t) { // __setstate__
                if (t.size() != 3)
                {
                    spdlog::warn("Invalid state given for Config, should be tuple of 3 items, returning default");
                    return MiniCheetahConfig();
                }
                return MiniCheetahConfig{.spawnPos = t[0].cast<Eigen::Vector3d>(),
                                         .spawnOrientation = t[1].cast<Eigen::Vector3d>(),
                                         .spawnJointPos = t[2].cast<Eigen::Matrix<double, 12, 1>>()};
            }));

    py::class_<MiniCheetah>(m, "MiniCheetah")
        .def(py::init<>())
        .def(py::init<MiniCheetahConfig>())
        .def("step", &MiniCheetah::Step, "Steps the simulator N times")
        .def("render", &MiniCheetah::Render, "Draws the current frame")
        .def("save_state", &MiniCheetah::SaveState, "Saves the current robot state")
        .def("load_state", &MiniCheetah::LoadState, "Loads the robot state to the specified checkpoint id")
        .def("set_joint_commands", &MiniCheetah::SetJointCommands, "Sets the commands for each joint")

        .def("set_joint_coulomb_friction", &MiniCheetah::SetJointCoulombFriction,
             "Sets the coulomb friction for each joint (Nm), produces counter torque equal to this value")
        .def("set_joint_viscous_friction", &MiniCheetah::SetJointViscousFriction,
             "Sets the viscous friction for each joint (Nm/(rad/s)), produces counter torque proportional to "
             "velocity*value")
        .def("set_foot_friction", &MiniCheetah::SetFootFriction, "Sets the friction coefficient of each foot")

        .def("get_joint_coulomb_friction", &MiniCheetah::GetJointCoulombFriction,
             "Sets the coulomb friction for each joint (Nm)")
        .def("get_joint_viscous_friction", &MiniCheetah::GetJointViscousFriction,
             "Sets the viscous friction for each joint (Nm/(rad/s))")
        .def("get_foot_friction", &MiniCheetah::GetFootFriction, "Gets the friction coefficient of each foot")

        .def("get_foot_positions", &MiniCheetah::GetFootPositions,
             "Gets the position of foot centre, in world frame and coordinates")

        .def("get_foot_contact_states", &MiniCheetah::GetFootContactStates,
             "Gets foot contact states, true for contact, false otherwise")
        .def("get_foot_contact_forces", &MiniCheetah::GetFootContactForces,
             "Gets foot contact force vectors, 0 if not in contact")
        .def("get_foot_contact_normals", &MiniCheetah::GetFootContactNormals,
             "Gets foot contact contact normals, 0 if not in contact, unit vector representing direction otherwise")

        .def("get_joint_positions", &MiniCheetah::GetJointPositions, "Gets joint positions of robot")
        .def("get_joint_velocities", &MiniCheetah::GetJointVelocities, "Gets joint velocities of robot")

        .def(
            "get_orientation",
            [](const MiniCheetah &r) { return Eigen::Matrix<double, 4, 1>{r.GetOrientation().coeffs()}; },
            "Gets the orientation of the robot in quaternion in world frame, (x,y,z,w)")
        .def("get_world_lin_vel", &MiniCheetah::GetWorldLinVel,
             "Gets the linear velocity (m/s) of the robot in world frame")
        .def("get_world_ang_vel", &MiniCheetah::GetWorldAngVel,
             "Gets the angular velocity (rad/s) of the robot in world frame")
        .def("get_world_lin_acc", &MiniCheetah::GetWorldLinAcc,
             "Gets the linear acceleration (m/s^2) of the robot in world frame")
        .def("add_ball", &MiniCheetah::AddBall,
             "Adds a ball relative to robot geometric centre, returns name of ball stored in sim",
             py::arg("translation"), py::arg("color"), py::arg("radius"), py::arg("name"))
        .def("set_ball_translation", &MiniCheetah::SetBallTranslation, "Sets translation of a ball using its name",
             py::arg("name"), py::arg("translation"), py::arg("frame"))
        .def("delete_ball", &MiniCheetah::DeleteBall, "Deletes a ball using its name", py::arg("name"));
}
