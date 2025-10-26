# File: train_sac.py
import os, time, subprocess
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv
import torch
from env_px4 import Px4BiasEnv

# 학습 설정
N_ENV      = 8
SIM_SPEED  = 10
TIMESTEPS  = 2_000_000
MODEL_PATH = "model_ckpt/bias_sac_px4"

# 1) 다중 SITL 인스턴스 실행
subprocess.Popen(["./launch_sitl.sh", str(N_ENV), str(SIM_SPEED)])
time.sleep(8)

# 2) 벡터 환경 생성
def make_env(i):
    return lambda: Px4BiasEnv(i)
env = SubprocVecEnv([make_env(i) for i in range(N_ENV)])

# 3) SAC 모델 초기화
policy_kwargs = dict(
    net_arch={"pi": [256, 256, 128], "qf": [400, 300]},
    activation_fn=torch.nn.ReLU
)
model = SAC(
    "MlpPolicy", env,
    policy_kwargs=policy_kwargs,
    buffer_size=1_000_000,
    batch_size=256,
    learning_starts=5_000,
    gamma=0.99,
    tau=0.005,
    device="cuda",
    verbose=1
)

# 4) 학습 실행
model.learn(total_timesteps=TIMESTEPS)
model.save(MODEL_PATH)

# 5) TorchScript로 정책 저장
example_obs = torch.randn(1, env.observation_space.shape[0]).to(model.device)
scripted = torch.jit.trace(model.policy, example_obs)
scripted.save(f"{MODEL_PATH}_policy.pt")
