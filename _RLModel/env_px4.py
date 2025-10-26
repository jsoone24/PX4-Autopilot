# File: env_px4.py
import os, time, subprocess, asyncio
from collections import deque
import numpy as np
import gymnasium as gym

# 환경 설정
SIM_SPEED = int(os.getenv("SIM_SPEED", 10))   # Gazebo 배속
CTRL_RATE = 10                                # RL 제어 주기 (Hz)
HIST_K     = 5                               # action history 길이
BIAS_MAX   = np.deg2rad(10.0)                 # ±10 °/s

class Px4BiasEnv(gym.Env):
    metadata = {"render_modes": []}
    def __init__(self, instance_id: int):
        super().__init__()
        self.instance = instance_id
        self.port     = 14540 + 10 * instance_id
        self.act_hist = deque([np.zeros(3)] * HIST_K, maxlen=HIST_K)

        # 관측·행동 스페이스 정의
        # pos(3), vel(3), att(3), ang_vel(3), err(3), history(3*HIST_K)
        obs_dim = 3 + 3 + 3 + 3 + 3 + 3 * HIST_K
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = gym.spaces.Box(
            low=-BIAS_MAX, high=BIAS_MAX, shape=(3,), dtype=np.float32)

        # PX4 SITL 연결
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self._connect())
        # TODO: Gazebo groundtruth topic 구독 로직 추가

    async def _connect(self):
        from mavsdk import System
        self.drone = System()
        await self.drone.connect(system_address=f"udp://:{self.port}")

    async def _arm_takeoff_120(self):
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        async for pos in self.drone.telemetry.position():
            if pos.relative_altitude_m >= 118.0:
                break

    def _inject_bias(self, bias: np.ndarray):
        # 터미널 명령으로 IMU bias 주입
        x, y, z = bias
        cmd = f"gz topic -p /gazebo/default/gyro_bias -m 'x: {x} y: {y} z: {z}'"
        subprocess.run(cmd, shell=True, check=True)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # 이륙 후 120m 호버
        self.loop.run_until_complete(self._arm_takeoff_120())
        self.act_hist.clear()
        self.target = np.array([500.0, 500.0, 120.0], dtype=np.float32)
        return self._get_obs(), {}

    def _get_groundtruth(self):
        # TODO: Gazebo groundtruth topic에서 pos, vel, att, ang_vel 반환
        # 예시로 0벡터 반환
        return np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)

    def _get_obs(self):
        p, v, att, ang_vel = self._get_groundtruth()
        err = p - self.target
        hist = np.concatenate(self.act_hist)
        return np.concatenate([p, v, att, ang_vel, err, hist]).astype(np.float32)

    def step(self, action):
        a = np.clip(action, -BIAS_MAX, BIAS_MAX)
        self._inject_bias(a)
        self.act_hist.appendleft(a)

        # sim-time DT_PER_STEP = 1/CTRL_RATE, real-time sleep에 sim-speed 반영
        time.sleep(1.0 / (CTRL_RATE * SIM_SPEED))

        obs = self._get_obs()
        dist = np.linalg.norm(obs[0:3] - self.target)
        reward = -0.01 * dist
        done = bool(dist < 5.0 or obs[2] < 1.0)
        if obs[2] < 1.0:
            reward -= 50.0
        return obs, reward, done, False, {}

    def close(self):
        pass
