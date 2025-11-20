from simulation import Simulation
from action import Action


def main():
    sim = Simulation(def_name="Grq20V2d3X5")
    sim.start()

    init_state = sim.state_queue.get()
    dt = init_state["time_step_ms"] / 1000.0

    action = Action(sim, dt=dt)
    action.sim.update_state(init_state["position"], init_state["rotation"])

    print("Running action 'set_position'")
    result = action.set_position({"delta_x": 3, "delta_y": 0, "delta_yaw": 0})
    print(result["status"])
    print("Running action 'set_velocity'")
    result = action.set_velocity({"vx": 0, "vy": -0.5, "yaw_rate": 0.0, "duration": 2})
    print(result["status"])

    sim.join()


if __name__ == "__main__":
    main()
