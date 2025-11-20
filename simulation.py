import sys
import os
import time
import math
import multiprocessing as mp
from queue import Empty

sys.path.append(os.path.join(os.environ["WEBOTS_HOME"], "lib", "controller", "python"))
from controller import Supervisor


class Simulation(mp.Process):
    """
    仅负责 Webots 仿真步进的子进程。
    通过队列接收目标位姿 (x, y, z, yaw)，并把当前状态（status、position、rotation、time_step_ms）回传。
    """

    def __init__(self, def_name: str):
        super().__init__(daemon=True)
        self.def_name = def_name
        self.stop_event = mp.Event()
        self.command_queue: mp.Queue[tuple[float, float, float, float]] = mp.Queue()
        self.state_queue: mp.Queue[dict] = mp.Queue(maxsize=1)
        # 控制侧缓存（在父进程中使用）
        self.dt = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.vx_body = 0.0
        self.vy_body = 0.0
        self.wz = 0.0

    def set_target_pose(self, x: float, y: float, z: float, yaw: float):
        """发送目标位姿给仿真进程"""
        self.command_queue.put((x, y, z, yaw))

    def get_latest_state(self, timeout=None):
        """在父进程侧获取最新状态（若队列为空则返回 None）。"""
        latest = None
        try:
            while True:
                latest = self.state_queue.get(timeout=timeout)
                timeout = 0  # 后续非阻塞
        except Empty:
            pass
        return latest

    # ===== 父进程侧控制辅助方法 =====
    def configure_dt(self, dt: float):
        self.dt = dt

    def update_state(self, position, rotation):
        """更新内部缓存的位姿（供父进程设置初始值或外部同步）。"""
        self.x, self.y, self.z = position
        ax, ay, az, theta = rotation
        half = theta * 0.5
        s = math.sin(half)
        qw = math.cos(half)
        qx = ax * s
        qy = ay * s
        qz = az * s
        self.yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    def set_velocity(self, vx_body: float, vy_body: float, wz: float):
        """在父进程中按指定速度积分一次，并发送目标位姿。"""
        if self.dt <= 0:
            return
        self.vx_body = vx_body
        self.vy_body = vy_body
        self.wz = wz
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vx_world = c * self.vx_body - s * self.vy_body
        vy_world = s * self.vx_body + c * self.vy_body
        self.x += vx_world * self.dt
        self.y += vy_world * self.dt
        self.yaw += self.wz * self.dt
        self.set_target_pose(self.x, self.y, self.z, self.yaw)

    def _emit_state(self, status, trans_field, rot_field, time_step_ms: int):
        """状态回传"""
        payload = {
            "status": status,
            "position": trans_field.getSFVec3f(),
            "rotation": rot_field.getSFRotation(),
            "time_step_ms": time_step_ms,
        }
        try:
            if self.state_queue.full():
                self.state_queue.get_nowait()
        except Exception:
            pass
        self.state_queue.put(payload)

    def run(self):
        supervisor = Supervisor()
        time_step_ms = int(supervisor.getBasicTimeStep())

        robot = supervisor.getFromDef(self.def_name)
        if robot is None:
            raise RuntimeError(f"无法找到 DEF 名称为 '{self.def_name}' 的 Robot 节点。")

        trans_field = robot.getField("translation")
        rot_field = robot.getField("rotation")

        # 初始状态回传：附带 time_step_ms 方便主进程使用
        self._emit_state("init", trans_field, rot_field, time_step_ms)

        period = time_step_ms / 1000.0
        latest_pose = None

        while not self.stop_event.is_set():
            # 消费最新的目标位姿（仅取最后一个）
            try:
                while True:
                    latest_pose = self.command_queue.get_nowait()
            except Empty:
                pass

            start = time.perf_counter()

            if latest_pose is not None:
                x, y, z, yaw = latest_pose
                trans_field.setSFVec3f([x, y, z])
                rot_field.setSFRotation([0.0, 0.0, 1.0, yaw])

            status = supervisor.step(time_step_ms)

            # 回传当前状态
            self._emit_state(status, trans_field, rot_field, time_step_ms)

            if status == -1:
                break

            elapsed = time.perf_counter() - start
            remaining = period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def stop(self):
        self.stop_event.set()
