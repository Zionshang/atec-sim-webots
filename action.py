import math
import time

from simulation import Simulation


class Action:
    """对外接口：提供 set_velocity / set_position（相对位移）并返回简单状态。"""

    def __init__(self, simulation: Simulation, default_speed: float = 0.5):
        self.sim = simulation
        self.default_speed = default_speed

    def set_velocity(self, params: dict):
        """
        设置本体系速度并按 duration 持续施加（默认为0则单步）。
        params:
          - vx, vy: 本体系线速度
          - yaw_rate: 本体系角速度
          - duration: 持续时间秒
        """
        vx = float(params.get("vx", 0.0))
        vy = float(params.get("vy", 0.0))
        yaw_rate = float(params.get("yaw_rate", 0.0))
        duration = float(params.get("duration", 0.0))

        steps = max(1, math.ceil(duration / self.sim.dt))
        for _ in range(steps):
            self.sim.set_velocity(vx_body=vx, vy_body=vy, wz=yaw_rate)
            time.sleep(self.sim.dt)
        return {"status": "ok"}

    def set_position(self, params: dict):
        """
        基于当前姿态设定相对位移/转角（本体系），用默认速度迭代积分直至抵达。
        params:
          - delta_x, delta_y: 本体系增量（米）
          - delta_yaw: 增量旋转（弧度）
        """
        if self.sim.dt <= 0:
            return {"status": "ok"}

        dx_body = float(params.get("delta_x", 0.0))
        dy_body = float(params.get("delta_y", 0.0))
        dyaw = float(params.get("delta_yaw", 0.0))

        # 将本体系增量转换为世界系目标增量
        c0, s0 = math.cos(self.sim.yaw), math.sin(self.sim.yaw)
        dx = c0 * dx_body - s0 * dy_body
        dy = s0 * dx_body + c0 * dy_body

        dist = math.hypot(dx, dy)
        v_mag = self.default_speed if dist > 0 else 0.0
        steps = 1 if v_mag == 0 else max(1, math.ceil(dist / (v_mag * self.sim.dt)))
        duration = steps * self.sim.dt
        dir_x = dx / dist if dist > 1e-9 else 0.0
        dir_y = dy / dist if dist > 1e-9 else 0.0
        vx_world = dir_x * v_mag
        vy_world = dir_y * v_mag
        wz = dyaw / duration if duration > 0 else 0.0

        for _ in range(steps):
            c, s = math.cos(self.sim.yaw), math.sin(self.sim.yaw)
            vx_body = c * vx_world + s * vy_world
            vy_body = -s * vx_world + c * vy_world
            self.sim.set_velocity(vx_body, vy_body, wz)
            time.sleep(self.sim.dt)
        return {"status": "ok"}
