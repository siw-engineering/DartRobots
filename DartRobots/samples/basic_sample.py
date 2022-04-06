import time
import numpy as np
import DartRobots.MiniCheetahPy as B


if __name__ == '__main__':
    robot = B.MiniCheetah()

    robot.save_state(0)
    robot.set_joint_commands(np.full((12,),0.0))
    zero_cmd = np.full((12,), 0.0)
    start_time = time.time()
    for _ in range(10):
        for i in range(150):
            robot.set_joint_commands(zero_cmd)
            robot.step(1)
            robot.render()
        robot.load_state(0)
    end_time = time.time()

    print(f"Time taken: {end_time - start_time}s")
