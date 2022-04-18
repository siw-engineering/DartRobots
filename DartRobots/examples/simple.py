import time
import numpy as np
from DartRobots.DartRobotsPy import World, MiniCheetah, MiniCheetahConfig, \
    get_mini_cheetah_urdf, get_ground_urdf, set_log_level

if __name__ == '__main__':
    set_log_level(0)
    config = MiniCheetahConfig()
    config.urdf_path = get_mini_cheetah_urdf()
    world = World()
    robot = MiniCheetah(config)
    world.set_terrain_urdf(get_ground_urdf())
    world.set_robot(robot)
    robot.save_state(0)
    robot.set_joint_commands(np.full((12,), 0.0))
    zero_cmd = np.full((12,), 0.0)
    start_time = time.time()
    for _ in range(10):
        for i in range(150):
            robot.set_joint_commands(zero_cmd)
            world.step(1)
            world.render()
        robot.load_state(0)
    end_time = time.time()

    print(f"Time taken: {end_time - start_time}s")
