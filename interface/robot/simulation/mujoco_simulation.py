import os
import time
import socket
import struct
import threading
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer
from colorama import init, Fore, Style

# Initialize colorama for colored terminal output
init(autoreset=True)

MODEL_NAME = "lite3"
XML_PATH = "../../../Lite3_description/lite3_mjcf/mjcf/Lite3_stair.xml"
LOCAL_PORT = 20001
CTRL_IP = "127.0.0.1"
CTRL_PORT = 30010
USE_VIEWER = True
DT = 0.001
RENDER_INTERVAL = 10

URDF_INIT = {
    "lite3": np.array([0, -1.35453, 2.54948] * 4, dtype=np.float32)
}

class MuJoCoSimulation:
    def __init__(self, model_key: str = MODEL_NAME, 
                 xml_relpath: str = XML_PATH,
                 local_port: int = LOCAL_PORT, 
                 ctrl_ip: str = CTRL_IP, 
                 ctrl_port: int = CTRL_PORT):
        
        # UDP communication
        self.local_port = local_port
        self.ctrl_addr = (ctrl_ip, ctrl_port)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("0.0.0.0", local_port))
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Load MJCF
        xml_full = str(Path(__file__).resolve().parent / xml_relpath)
        print("xml_full", xml_full)
        if not os.path.isfile(xml_full):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_full}")

        self.model = mujoco.MjModel.from_xml_path(xml_full)
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)

        # Robot DOF list
        self.actuator_ids = [a for a in range(self.model.nu)]  # 0..11
        self.dof_num = len(self.actuator_ids)

        # Initialize standing pose
        self._set_initial_pose(model_key)

        # Buffers
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)
        self.timestamp = 0.0
        self.last_print_time = 0  # Track last print time

        print(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")

        # Visualization
        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._set_viewer_camera()

    def _set_viewer_camera(self):
        self.viewer.cam.distance = 3.8
        self.viewer.cam.azimuth = 135
        self.viewer.cam.elevation = -18
        self.viewer.cam.lookat[:] = np.array([0.4, 0.0, 0.45])

    def _set_initial_pose(self, key: str):
        """Set joint positions to match PyBullet initial angles."""
        qpos0 = self.data.qpos.copy()
        qpos0[7:7+self.dof_num] = URDF_INIT[key]
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

    def print_debug_info(self):
        """Consolidated function to print debug information with colors and aligned formatting."""
        # Format arrays with 2 decimal places and fixed width
        def format_array(arr):
            return "[" + ", ".join(f"{x:6.2f}" for x in arr) + "]"

        # Get current joint states for printing
        q = self.data.qpos[7:7+self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6+self.dof_num].reshape(-1, 1)
        tau = self.input_tq.flatten()
        q_world = self.data.qpos[3:7]
        rpy = self.quaternion_to_euler(q_world)
        angvel_b = self.data.qvel[3:6]
        mat = np.zeros(9, dtype=np.float64)
        mujoco.mju_quat2Mat(mat, q_world.astype(np.float64))
        R = mat.reshape(3, 3)
        body_acc = self.data.sensordata[16:19]

        print(f"{Fore.CYAN}=== [Debug Info] ==={Style.RESET_ALL}")
        print(f"{Fore.GREEN}[IMU] RPY        :{Style.RESET_ALL} {format_array(rpy.flatten())}")
        print(f"{Fore.GREEN}[IMU] Omega      :{Style.RESET_ALL} {format_array(angvel_b.flatten())}")
        print(f"{Fore.GREEN}[IMU] Acc_body   :{Style.RESET_ALL} {format_array(body_acc.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Position  :{Style.RESET_ALL} {format_array(q.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Velocity  :{Style.RESET_ALL} {format_array(dq.flatten())}")
        print(f"{Fore.YELLOW}[Joint] Torque    :{Style.RESET_ALL} {format_array(tau.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Target Pos:{Style.RESET_ALL} {format_array(self.pos_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Actual Pos:{Style.RESET_ALL} {format_array(q.T.flatten())}")     
        print(f"{Fore.MAGENTA}[Joint Cmd] Target Vel:{Style.RESET_ALL} {format_array(self.vel_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Actual Vel:{Style.RESET_ALL} {format_array(dq.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Kp Term   :{Style.RESET_ALL} {format_array(self.kp_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Kd Term   :{Style.RESET_ALL} {format_array(self.kd_cmd.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] FF Tau    :{Style.RESET_ALL} {format_array(self.tau_ff.T.flatten())}")
        print(f"{Fore.MAGENTA}[Joint Cmd] Final Torq:{Style.RESET_ALL} {format_array(self.input_tq.T.flatten())}")
        print(f"{Fore.CYAN}==================={Style.RESET_ALL}")

    def start(self):
        # Start UDP receiver thread
        threading.Thread(target=self._udp_receiver, daemon=True).start()
        print(f"[INFO] UDP receiver on 0.0.0.0:{self.local_port}")

        # Main simulation loop
        step = 0
        last_time = time.time()
        while True:
            if time.time() - last_time >= DT:
                last_time = time.time()
                
                step += 1
                # 控制律

                self._apply_joint_torque()
                # 模拟一步
                mujoco.mj_step(self.model, self.data)

                self.timestamp = step * DT

                # 采样 & 发送观测
                self._send_robot_state(step)
                # 可视化
                if self.viewer and step % RENDER_INTERVAL == 0:
                    self.viewer.sync()
                    
                # Print at 0.5 Hz (every 2 seconds)
                current_time = time.perf_counter()
                if current_time - self.last_print_time >= 2.0:
                    self.print_debug_info()
                    self.last_print_time = current_time
                    

    def _udp_receiver(self):
        """
        12f kp | 12f pos | 12f kd | 12f vel | 12f tau = 240 bytes
        """
        fmt = f'{self.dof_num}f' * 5
        expected = struct.calcsize(fmt)
        while True:
            data, addr = self.recv_sock.recvfrom(expected)
            if len(data) < expected:
                print(f"[WARN] UDP packet size {len(data)} != {expected}")
                continue
            unpacked = struct.unpack(fmt, data)
            self.kp_cmd = np.asarray(unpacked[0:self.dof_num], dtype=np.float32).reshape(self.dof_num, 1)
            self.pos_cmd = np.asarray(unpacked[self.dof_num:self.dof_num * 2], dtype=np.float32).reshape(self.dof_num,
                                                                                                         1)
            self.kd_cmd = np.asarray(unpacked[self.dof_num * 2:self.dof_num * 3], dtype=np.float32).reshape(
                self.dof_num, 1)
            self.vel_cmd = np.asarray(unpacked[self.dof_num * 3:self.dof_num * 4], dtype=np.float32).reshape(
                self.dof_num, 1)
            self.tau_ff = np.asarray(unpacked[self.dof_num * 4:], dtype=np.float32).reshape(self.dof_num, 1)

    def _apply_joint_torque(self):
        # Current joint states
        q = self.data.qpos[7:7+self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6+self.dof_num].reshape(-1, 1)

        # τ = kp*(q_d - q) + kd*(dq_d - dq) + τ_ff
        self.input_tq = (
            self.kp_cmd * (self.pos_cmd - q) +
            self.kd_cmd * (self.vel_cmd - dq) +
            self.tau_ff
        )
        # Write to control buffer
        self.data.ctrl[:] = self.input_tq.flatten()

    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _send_robot_state(self, step: int):
        # IMU
        q_world = self.data.qpos[3:7]
        rpy = self.quaternion_to_euler(q_world)
        angvel_b = self.data.qvel[3:6]
        body_acc = self.data.sensordata[16:19]

        # Joints
        q = self.data.qpos[7:7+self.dof_num]
        dq = self.data.qvel[6:6+self.dof_num]
        tau = self.input_tq.flatten()

        # Pack and send
        payload = np.concatenate((
            np.array([self.timestamp], dtype=np.float64),
            np.asarray(rpy, dtype=np.float32),
            np.asarray(body_acc, dtype=np.float32),
            np.asarray(angvel_b, dtype=np.float32),
            q.astype(np.float32),
            dq.astype(np.float32),
            tau.astype(np.float32)
        ))
        fmt = "1d" + f"{len(payload)-1}f"
        try:
            self.send_sock.sendto(struct.pack(fmt, *payload), 
                                  self.ctrl_addr)
        except socket.error as ex:
            print(f"[UDP send] {ex}")


if __name__ == "__main__":
    sim = MuJoCoSimulation()
    sim.start()
