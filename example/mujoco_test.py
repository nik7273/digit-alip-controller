import time
import mujoco
import mujoco.viewer
import numpy as np

import digit_controller_pybind as dc

NUM_MOTORS = 20

AGILITY_NAMES = [
    "left-leg.hip-roll",
    "left-leg.hip-yaw",
    "left-leg.hip-pitch",
    "left-leg.knee",
    "left-leg.toe-a",
    "left-leg.toe-b",
    "right-leg.hip-roll",
    "right-leg.hip-yaw",
    "right-leg.hip-pitch",
    "right-leg.knee",
    "right-leg.toe-a",
    "right-leg.toe-b",
    "left-leg.shoulder-roll",
    "left-leg.shoulder-pitch",
    "left-leg.shoulder-yaw",
    "left-leg.elbow",
    "right-leg.shoulder-roll",
    "right-leg.shoulder-pitch",
    "right-leg.shoulder-yaw",
    "right-leg.elbow",
    "left-leg.shin",
    "left-leg.tarsus",
    "left-leg.toe-pitch",
    "left-leg.toe-roll",
    "left-leg.heel-spring",
    "right-leg.shin",
    "right-leg.tarsus",
    "right-leg.toe-pitch",
    "right-leg.toe-roll",
    "right-leg.heel-spring",
]

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

def mujoco_test():
    # initialize the controller
    gc = dc.Digit_Controller()
    init_ctrl_mode = 2  # 0: standing_analytic, 1: standing_numeric, 2: walking
    flag_torque_only = 1  # 0: use AR method, 1: feedforward torque only
    gc.Initialize_(init_ctrl_mode, flag_torque_only)
    gc.Set_Initial_Standing_Gains_()
    gc.Set_Initial_Walking_Gains_()

    # test controller inside mujoco
    m = mujoco.MjModel.from_xml_path("../assets/digit-v3.xml")
    d = mujoco.MjData(m)

    # initialize position
    init_pos = np.array(
        [
            0.003237, -0.097065, 1.033634, 0.999986, -0.001082,-0.005159, -0.000101,  # base x y z, base quat
            0.361490, 0.000770, 0.286025,  # left-hip-roll, left-hip-yaw, left-hip-pitch,
            0.983743, -0.000811, 0.003661, 0.179545,  # left-achilles-rod-quat
            0.373352, 0, -0.346032, -0.009752,  # left-knee, left-shin, left-tarsus, left-heel-spring,
            -0.092650, 0.979409, 0.196089, 0.008040, 0.047357,  # left-toe-A, left-toe-A-rod quat
            0.084274, 0.976142, 0.212618, -0.007367, -0.043426,  # left-toe-B, left-toe-B-rod quat
            0.091354, -0.013601,  # left-toe-pitch, left-toe-roll
            -0.1506, 1.0922, 0.0017, -0.1391,  # left-shoulder-roll, left-shoulder-pitch, left-shoulder-yaw, left-elbow
            -0.360407, -0.000561, -0.286076,  # right-hip-roll, right-hip-yaw, right-hip-pitch,
            0.983702, 0.000907, 0.003658, -0.179766,  # right-achilles-rod-quat
            -0.372723, 0, 0.347843, 0.008955,  # right-knee, right-shin, right-tarsus, right-heel-spring,
            0.095860, 0.979324, -0.196096, 0.008399, -0.048996,  # right-toe-A, right-toe-A-rod quat
            -0.083562, 0.976162, -0.212611, -0.007402, 0.043016,  # right-toe-B, right-toe-B-rod quat
            -0.092658, 0.019828,  # right-toe-pitch, right-toe-roll
            0.1506, -1.0922, -0.0017, 0.1391,  # right-shoulder-roll, right-shoulder-pitch, right-shoulder-yaw, right-elbow
        ]
    )
    d.qpos[:] = init_pos
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)

    # Adjust base height to make sure foot contacts with ground    
    left_toe_roll_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "left-leg.toe-roll")
    world_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "world")
    dist_max = 0

    for con in d.contact:
        if (m.geom_bodyid[con.geom1] == world_id and m.geom_bodyid[con.geom2] == left_toe_roll_id):
            if abs(con.dist * con.frame[2]) > abs(dist_max):
                dist_max = con.dist * con.frame[2]

    right_toe_roll_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right-leg.toe-roll")
    world_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "world")
    for con in d.contact:
        if (m.geom_bodyid[con.geom1] == world_id and m.geom_bodyid[con.geom2] == right_toe_roll_id):
            if abs(con.dist * con.frame[2]) > abs(dist_max):
                dist_max = con.dist * con.frame[2]

    d.qpos[2] -= dist_max

    # Turn off control limits
    # for i in range(m.nu):
    #     m.actuator_ctrllimited[i] = False

    with mujoco.viewer.launch_passive(m, d) as viewer:
        # Close the viewer automatically after 30 wall-seconds.
        start = time.time()
        while viewer.is_running() and time.time() - start < 30:
            step_start = time.time()

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            # mujoco.mj_step(m, d)
            step_controller(gc, m, d)

            # Example modification of a viewer option: toggle contact points every two seconds.
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def step_controller(gc: dc.Digit_Controller, m: mujoco.MjModel, d: mujoco.MjData):
    command = dc.Command()
    observation = dc.Observation()

    observation.time = d.time
    observation.error = 0

    # Get base data
    baseId = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso.base")

    for j in range(3):
        observation.base.translation[j] = d.qpos[j]

    observation.base.orientation.w = d.qpos[3]
    observation.base.orientation.x = d.qpos[4]
    observation.base.orientation.y = d.qpos[5]
    observation.base.orientation.z = d.qpos[6]

    rotMat = np.zeros((9,), dtype=np.double)
    mujoco.mju_quat2Mat(rotMat, d.qpos[3:7])
    linVel = np.zeros((3,), dtype=np.double)
    # angVel = np.zeros((3,), dtype=np.double)
    # mujoco.mju_mulMatTVec(linVel, rotMat, d.qvel, 3, 3)
    linVel = rotMat.reshape((3,3)).T @ d.qvel[3:6]

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
            observation.motor.torque[j] = d.ctrl[act_id] * m.actuator_gear[act_id, 0]

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
            print("Could not find actuator")
            exit()
        d.ctrl[act_id] = (
            command.motors[j][0] # .torque
            + command.motors[j][1] # .damping
            * (command.motors[j][2] - observation.motor.velocity[j])
        ) / m.actuator_gear[act_id, 0]

    mujoco.mj_step(m, d)

if __name__ == "__main__":
    mujoco_test()
