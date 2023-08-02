import time
import mujoco
import mujoco.viewer
import numpy as np

import digit_controller_pybind as dc

AGILITY_NAMES = [
    "left-hip-roll",
    "left-hip-yaw",
    "left-hip-pitch",
    "left-knee",
    "left-toe-A",
    "left-toe-B",
    "right-hip-roll",
    "right-hip-yaw",
    "right-hip-pitch",
    "right-knee",
    "right-toe-A",
    "right-toe-B",
    "left-shoulder-roll",
    "left-shoulder-pitch",
    "left-shoulder-yaw",
    "left-elbow",
    "right-shoulder-roll",
    "right-shoulder-pitch",
    "right-shoulder-yaw",
    "right-elbow",
    "left-shin",
    "left-tarsus",
    "left-toe-pitch",
    "left-toe-roll",
    "left-heel-spring",
    "right-shin",
    "right-tarsus",
    "right-toe-pitch",
    "right-toe-roll",
    "right-heel-spring",
]

NUM_MOTORS = 20

LIMITS = dc.Limits()
LIMITS.torque_limit = [
    126.682458,
    79.176536,
    216.927898,
    231.316950,
    41.975942,
    41.975942,
    126.682458,
    79.176536,
    216.927898,
    231.316950,
    41.975942,
    41.975942,
    126.682458,
    126.682458,
    79.176536,
    126.682458,
    126.682458,
    126.682458,
    79.176536,
    126.682458,
]
LIMITS.damping_limit = [
    66.849046,
    26.112909,
    38.050020,
    38.050020,
    28.553161,
    28.553161,
    66.849046,
    26.112909,
    38.050020,
    38.050020,
    28.553161,
    28.553161,
    66.849046,
    66.849046,
    26.112909,
    66.849046,
    66.849046,
    66.849046,
    26.112909,
    66.849046,
]
LIMITS.velocity_limit = [
    4.581489,
    7.330383,
    8.508480,
    8.508480,
    11.519173,
    11.519173,
    4.581489,
    7.330383,
    8.508480,
    8.508480,
    11.519173,
    11.519173,
    4.581489,
    4.581489,
    7.330383,
    4.581489,
    4.581489,
    4.581489,
    7.330383,
    4.581489,
]

# initialize the controller
gc = dc.Digit_Controller()
init_ctrl_mode = 2  # 0: standing_analytic, 1: standing_numeric, 2: walking
flag_torque_only = 1  # 0: use AR method, 1: feedforward torque only
gc.Initialize_(init_ctrl_mode, flag_torque_only)
gc.Set_Initial_Standing_Gains_()
gc.Set_Initial_Walking_Gains_()

# test controller inside mujoco
m = mujoco.MjModel.from_xml_path("digit.xml")
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def controller(m: mujoco.MjModel, d: mujoco.MjData):
    command = dc.Command()
    observation = dc.Observation()

    observation.time = d.time
    observation.error = 0

    # Get base data
    baseId = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")

    for j in range(3):
        observation.base.translation[j] = d.qpos[j]

    observation.base.orientation.w = d.qpos[3]
    observation.base.orientation.x = d.qpos[4]
    observation.base.orientation.y = d.qpos[5]
    observation.base.orientation.z = d.qpos[6]

    rotMat = np.zeros((9,), dtype=np.double)
    mujoco.mju_quat2Mat(rotMat, d.qpos + 3)
    linVel = np.zeros((3,), dtype=np.double)
    # angVel = np.zeros((3,), dtype=np.double)
    mujoco.mju_mulMatTVec(linVel, rotMat, d.qvel, 3, 3)

    for j in range(3):      
        observation.base.linear_velocity[j] = linVel[j]

    for j in range(3):
        observation.base.angular_velocity[j] = d.qvel[j + 3]

    observation.imu.orientation.w = 0
    observation.imu.orientation.x = 0
    observation.imu.orientation.y = 0
    observation.imu.orientation.z = 0

    for j in range(3):
        observation.imu.angular_velocity[j] = 0

    for j in range(3):
        observation.imu.linear_acceleration[j] = 0

    for j in range(3):
        observation.imu.magnetic_field[j] = 0

    for j in range(12):
        observation.motor.position[j] = 0
        observation.motor.velocity[j] = 0
        observation.motor.torque[j] = 0

        joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, AGILITY_NAMES[j])
        act_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, AGILITY_NAMES[j])
        if joint_id == -1:
            print("Couldn't find motor", AGILITY_NAMES[j])
        else:
            observation.motor.position[j] = d.qpos[m.jnt_qposadr[joint_id]]
            observation.motor.velocity[j] = d.qvel[m.jnt_dofadr[joint_id]]
            observation.motor.torque[j] = d.ctrl[act_id] * m.actuator_gear[act_id * 6]

    for j in range(12, NUM_MOTORS):
        observation.motor.position[j] = 0
        observation.motor.velocity[j] = 0
        observation.motor.torque[j] = 0

    for j in range(10):
        observation.joint.position[j] = 0
        observation.joint.velocity[j] = 0
        joint_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, AGILITY_NAMES[j + 20])
        if joint_id == -1:
            print("Couldn't find joint", AGILITY_NAMES[j + 20])
        else:
            observation.joint.position[j] = d.qpos[m.jnt_qposadr[joint_id]]
            observation.joint.velocity[j] = d.qvel[m.jnt_dofadr[joint_id]]

    gc.Update_(command, observation, LIMITS)

    for j in range(12):
        act_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, AGILITY_NAMES[j])
        if act_id == -1:
            raise Exception("Could not find actuator")
        d.ctrl[act_id] = (
            command.motors[j].torque
            + command.motors[j].damping
            * (command.motors[j].velocity - observation.motor.velocity[j])
        ) / m.actuator_gear[act_id * 6]

    mujoco.mj_step(m, d)
