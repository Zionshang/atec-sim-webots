from simulation import Simulation
from action import Action
import cv2
import numpy as np


def main():
    sim = Simulation(def_name="Grq20V2d3X5")
    sim.start()
    print("Waiting for simulation to be ready...")
    sim.wait_until_ready()
    print("Simulation is ready.")

    action = Action(sim)
    print("Running action 'set_position'")
    result = action.set_position({"delta_x": 3, "delta_y": 0, "delta_yaw": 0})
    print(result["status"])
    print("Running action 'set_velocity'")
    result = action.set_velocity({"vx": 0, "vy": -0.5, "yaw_rate": 0.0, "duration": 2})
    print(result["status"])

    # 简单展示相机画面
    for _ in range(100):
        img_bgr = sim.get_camera_frame()
        cv2.imshow("wrist_camera", img_bgr)
        if cv2.waitKey(1) == 27:  # ESC
            break

    sim.join()


if __name__ == "__main__":
    main()
