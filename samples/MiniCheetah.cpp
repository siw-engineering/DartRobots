#include "DartRobots/MiniCheetah.hpp"
#include <chrono>
#include <iostream>
using namespace std::chrono;

int main()
{
    auto robot = DartRobots::MiniCheetah();
    auto startTime = steady_clock::now();
    auto markerName = robot.AddBall(Eigen::Vector3d(0.19, 0.11, -0.29), Eigen::Vector3d(0.9, 0.0, 0.0), 0.05);
    auto markerName2 = robot.AddBall(Eigen::Vector3d(0.19, -0.11, -0.29), Eigen::Vector3d(0.9, 0.0, 0.0), 0.05);
    std::cout << "Add ball with marker name: " << markerName << std::endl;
    std::cout << "Add ball with marker name: " << markerName2 << std::endl;
    robot.SaveState(0);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 200; j++)
        {
            robot.SetJointCommands(Eigen::Matrix<double, 12, 1>::Zero());
            robot.Step(1);
            auto footPos = robot.GetFootPositions();
            robot.SetBallTranslation(markerName, footPos.block<3, 1>(0, 0));
            robot.Render();
        }
        robot.LoadState(0);
        robot.Reset();
        robot.DeleteBall(markerName2);
    }
    auto endTime = steady_clock::now();
    auto timeDiffMs = duration_cast<milliseconds>(endTime - startTime).count();
    std::cout << "Time taken: " << timeDiffMs << std::endl;
}