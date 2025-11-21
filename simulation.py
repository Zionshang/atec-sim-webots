import sys
import os
import time
import math
from threading import Event, Lock, Thread

sys.path.append(os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python"))
from controller import Supervisor


class Simulation(Thread):
    """
    仅负责 Webots 仿真步进的子线程。
    通过共享变量+锁接收目标位姿 (x, y, z, yaw)。
    """

    def __init__(self, def_name: str):
        super().__init__(daemon=True)
        self.def_name = def_name
        self.lock = Lock()
        self.ready_event = Event()

        # webots Supervisor 相关
        self.supervisor = Supervisor()
        self.time_step_ms = int(self.supervisor.getBasicTimeStep())
        self.dt = self.time_step_ms / 1000.0
        robot = self.supervisor.getFromDef(self.def_name)
        if robot is None:
            raise RuntimeError(f"无法找到 DEF 名称为 '{self.def_name}' 的 Robot 节点。")
        self.trans_field = robot.getField("translation")
        self.rot_field = robot.getField("rotation")

        self.x = 0.0  # 世界系下的位置x
        self.y = 0.0  # 世界系下的位置y
        self.z = 0.0  # 世界系下的位置z
        self.yaw = 0.0  # 世界系下的航向角（弧度）

        # 初始化起始状态
        pos = self.trans_field.getSFVec3f()
        rot = self.rot_field.getSFRotation()
        self.x, self.y, self.z = pos
        ax, ay, az, theta = rot
        half = theta * 0.5
        s = math.sin(half)
        qw = math.cos(half)
        qx = ax * s
        qy = ay * s
        qz = az * s
        self.yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        # 初始化目标位姿
        self.target_pose = [self.x, self.y, self.z, self.yaw]

    def set_velocity(self, vx_body: float, vy_body: float, wz: float):
        """
        通过supervisor设置局部系下的机器人速度
        """
        if self.dt <= 0:
            return
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vx_world = c * vx_body - s * vy_body
        vy_world = s * vx_body + c * vy_body
        self.x += vx_world * self.dt
        self.y += vy_world * self.dt
        self.yaw += wz * self.dt
        with self.lock:
            self.target_pose = (self.x, self.y, self.z, self.yaw)

    def run(self):
        while True:
            with self.lock:
                pose = self.target_pose

            start = time.perf_counter()

            # 设置机器人位姿
            x, y, z, yaw = pose
            self.trans_field.setSFVec3f([x, y, z])
            self.rot_field.setSFRotation([0.0, 0.0, 1.0, yaw])

            # 执行仿真步进
            status = self.supervisor.step(self.time_step_ms)
            if status == -1:
                break
            if status == 0:
                self.ready_event.set()

            # 保持仿真步进周期
            elapsed = time.perf_counter() - start
            remaining = self.dt - elapsed
            if remaining > 0:
                time.sleep(remaining)
        print("Exiting simulation.")

    def wait_until_ready(self, timeout: float | None = None) -> bool:
        return self.ready_event.wait(timeout=timeout)
