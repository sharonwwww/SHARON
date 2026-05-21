"""
第一步：先跑这个脚本，把输出截图/粘贴给我。
它会打印所有关节名称、类型、以及当前角度，
这样我们就能知道哪些关节控制什么，方向是否正确。
"""
import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# 用你原来的初始朝向
start_orn = p.getQuaternionFromEuler([math.pi / 2, 0, math.pi / 2])
robot_id = p.loadURDF("laikago/laikago_toes.urdf", [0, 0, 0.5], start_orn)

# 禁用默认速度控制干扰
for j in range(p.getNumJoints(robot_id)):
    p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, force=0)

print("=" * 60)
print(f"总关节数: {p.getNumJoints(robot_id)}")
print("=" * 60)
for j in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, j)
    joint_name = info[1].decode()
    joint_type = {0: "REVOLUTE", 1: "PRISMATIC", 4: "FIXED"}.get(info[2], str(info[2]))
    lower_lim  = info[8]
    upper_lim  = info[9]
    axis       = info[13]   # 旋转轴方向
    print(f"  [{j:2d}] {joint_name:<25} type={joint_type:<10} axis={axis}  limits=[{lower_lim:.2f}, {upper_lim:.2f}]")

print("=" * 60)
print("窗口将保持打开，可以在 GUI 里观察机器人初始姿态")
print("按 Ctrl+C 退出")

# 让仿真静止，方便观察初始状态
try:
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
except KeyboardInterrupt:
    pass

p.disconnect()