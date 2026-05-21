import pybullet as p
import pybullet_data
import time
import math

LEGS = {
    'FR': (0,  1,  2),
    'FL': (4,  5,  6),
    'RR': (8,  9,  10),
    'RL': (12, 13, 14),
}

TROT_PHASE = {
    'FR': 0.0,
    'RL': 0.0,
    'FL': math.pi,
    'RR': math.pi,
}

STAND_HIP   =  0.032
STAND_UPPER = -1.284
STAND_LOWER =  1.811


def set_leg(robot_id, leg, hip, upper, lower, force=200.0):
    hid, uid, lid = LEGS[leg]
    for jid, ang in [(hid, hip), (uid, upper), (lid, lower)]:
        p.setJointMotorControl2(
            robot_id, jid,
            p.POSITION_CONTROL,
            targetPosition=ang,
            force=force,
            maxVelocity=5.0,
            positionGain=0.5,
            velocityGain=0.1,
        )


def trot(robot_id, t,
         step_angle=0.08,
         step_h_angle=0.08,
         freq=0.5,
         force=200.0):
    for leg, phase_off in TROT_PHASE.items():
        phi = (2 * math.pi * freq * t + phase_off) % (2 * math.pi)

        # 支撑相占比加大：摆动相只占 40%，支撑相占 60%
        swing_ratio = 0.4
        swing_end   = 2 * math.pi * swing_ratio

        if phi < swing_end:                     # 摆动相（抬腿）
            prog  = phi / swing_end
            upper = STAND_UPPER - step_angle * math.sin(math.pi * prog)
            lower = STAND_LOWER - step_h_angle * math.sin(math.pi * prog)
        else:                                   # 支撑相（蹬地）
            prog  = (phi - swing_end) / (2 * math.pi - swing_end)
            upper = STAND_UPPER + step_angle * 0.5 * math.sin(math.pi * prog)
            lower = STAND_LOWER

        set_leg(robot_id, leg, STAND_HIP, upper, lower, force=force)


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    p.resetDebugVisualizerCamera(
        cameraDistance=1.2, cameraYaw=45,
        cameraPitch=-25, cameraTargetPosition=[0, 0, 0.3])

    start_orn = p.getQuaternionFromEuler([math.pi / 2, 0, math.pi / 2])
    robot_id  = p.loadURDF("laikago/laikago_toes.urdf", [0, 0, 0.8], start_orn)

    for j in range(p.getNumJoints(robot_id)):
        p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, force=0)

    dt = 1.0 / 240.0

    # 站立稳定 3 秒
    print("站立初始化...")
    for _ in range(int(3.0 / dt)):
        for leg in LEGS:
            set_leg(robot_id, leg, STAND_HIP, STAND_UPPER, STAND_LOWER, force=200.0)
        p.stepSimulation()
        time.sleep(dt)

    print("Trot 开始，Ctrl+C 退出")
    t = 0.0
    try:
        while True:
            trot(robot_id, t,
                 step_angle=0.08,
                 step_h_angle=0.08,
                 freq=0.5,
                 force=200.0)
            p.stepSimulation()
            time.sleep(dt)
            t += dt
    except KeyboardInterrupt:
        print("结束")

    p.disconnect()


if __name__ == '__main__':
    main()