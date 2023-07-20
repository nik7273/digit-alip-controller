#include "Digit_Controller.hpp"

using namespace Eigen;
/*==========================================================================
                        PUBLIC MEMBER FUNCTIONS
  ==========================================================================*/

/* Constructor */
Digit_Controller::Digit_Controller() {}

/* Destructor */
Digit_Controller::~Digit_Controller() {}

/* Initialize */
void Digit_Controller::Initialize_(const int mode, const int flag_torque_only)
{

    // Set_Ctrl_Mode_(mode); // set initial controller mode. Can be changed via keyboard/gamepad inputs
    ctrl_mode_ = mode;
    torque_only_ = flag_torque_only;
    Get_Config_Parameters_(); // retrieve config parameters at runtime via ROS
    return;
}

/* Update */
void Digit_Controller::Update_(llapi_command_t &command_update,
                               const llapi_observation_t &observation_update,
                               const llapi_limits_t *const limits_update)
{
    /* Extract Coordinate Vectors from Observation */
    double time_digit;

    Matrix<double, constants::NUM_Q_ALL, 1> q_all, qdot_all;
    Matrix<double, NUM_MOTORS, 1> q_motors, qdot_motors;
    Matrix<double, NUM_UNACT_JOINTS, 1> q_joints, qdot_joints;
    Digit_Controller::Extract_Observation_(time_digit, q_all, qdot_all, q_motors, qdot_motors, q_joints, qdot_joints, observation_update, limits_update);
    if (time_digit == time_digit_prev_)
    {
        // std::cout << "SAME SIM TIME -- YOU SHOULD NOT SEE THIS!\n\n";
        command_update = command_update_prev_;
    }
    else
    {
        // std::cout << "time_digit: " << time_digit << " (" << 1.0 / (time_digit - time_digit_prev_) << ")\n\n";

        // Choose controller to use
        switch (ctrl_mode_)
        {
        case CTRL_MODE::STANDING_ANALYTIC:
            Digit_Controller::Standing_Analytic_Update_(command_update, time_digit, q_all, qdot_all, q_motors, qdot_motors, limits_update);
            break;
        case CTRL_MODE::STANDING_NUMERIC:
            Digit_Controller::Standing_Numeric_Update_(command_update, time_digit, q_all, qdot_all, q_motors, qdot_motors, q_joints, qdot_joints, limits_update);
            break;
        case CTRL_MODE::WALKING:
            Digit_Controller::Walking_Update_(command_update, time_digit, q_all, qdot_all, q_motors, qdot_motors, q_joints, limits_update);
            break;
        }
        command_update_prev_ = command_update;
        time_digit_prev_ = time_digit;
    }
}

int Digit_Controller::Get_Ctrl_Mode_()
{
    return ctrl_mode_;
}
void Digit_Controller::Set_Ctrl_Mode_(int mode)
{
    ctrl_mode_ = mode;
}

void Digit_Controller::Print_Standing_Gains_()
{
    std::cout << "\n----- STANDING GAINS ------\n";

    std::cout << "Kp_hiproll_stand: " << kp_hiproll_stand_tuned_ << ", Kp_hipyaw_stand: " << kp_hipyaw_stand_tuned_
              << ", Kp_hippitch_stand: " << kp_hippitch_stand_tuned_ << ", Kp_knee_stand: " << kp_knee_stand_tuned_
              << ", Kp_toe_stand: " << kp_toe_stand_tuned_;

    // std::cout << "\nKp_shoulderroll: " << kp_shoulderroll_stand_tuned_ << ", Kp_shoulderpitch: " << kp_shoulderpitch_stand_tuned_
    //           << ", Kp_shoulderyaw: " << kp_shoulderyaw_stand_tuned_ << ", Kp_elbow: " << kp_elbow_stand_tuned_;

    std::cout << "\nKp_lateral_stand: " << kp_lateral_stand_tuned_ << ", Kd_lateral_stand: " << kd_lateral_stand_tuned_ << "\n";

    std::cout << "\nKp_knee_comp_stand: " << kp_knee_comp_stand_tuned_ << ", Kd_knee_comp_stand: " << kd_knee_comp_stand_tuned_ << "\n\n";
    return;
}
void Digit_Controller::Set_Initial_Standing_Gains_()
{
    kp_hiproll_stand_tuned_ = kp_hiproll_stand_base_;
    kp_hipyaw_stand_tuned_ = kp_hipyaw_stand_base_;
    kp_hippitch_stand_tuned_ = kp_hippitch_stand_base_;
    kp_knee_stand_tuned_ = kp_knee_stand_base_;
    kp_toe_stand_tuned_ = kp_toe_stand_base_;
    kp_shoulderroll_stand_tuned_ = kp_shoulderroll_stand_base_;
    kp_shoulderpitch_stand_tuned_ = kp_shoulderpitch_stand_base_;
    kp_shoulderyaw_stand_tuned_ = kp_shoulderyaw_stand_base_;
    kp_elbow_stand_tuned_ = kp_elbow_stand_base_;

    kp_lateral_stand_tuned_ = kp_lateral_stand_base_;
    kd_lateral_stand_tuned_ = kd_lateral_stand_base_;

    kp_knee_comp_stand_tuned_ = kp_knee_comp_stand_base_;
    kd_knee_comp_stand_tuned_ = kd_knee_comp_stand_base_;
    return;
}

void Digit_Controller::Print_Walking_Gains_()
{
    std::cout << "\n----- WALKING GAINS ------\n";
    std::cout << "Kp_hiproll_sw: " << kp_hiproll_sw_tuned_ << ", Kp_hipyaw_sw: " << kp_hipyaw_sw_tuned_
              << ", Kp_hippitch_sw: " << kp_hippitch_sw_tuned_ << ", Kp_knee_sw: " << kp_knee_sw_tuned_
              << ", Kp_toe_sw: " << kp_toe_sw_tuned_;

    std::cout << "\nKp_hiproll_st: " << kp_hiproll_st_tuned_ << ", Kp_hipyaw_st: " << kp_hipyaw_st_tuned_
              << ", Kp_hippitch_st: " << kp_hippitch_st_tuned_ << ", Kp_knee_st: " << kp_knee_st_tuned_
              << ", Kp_toe_st: " << kp_toe_st_tuned_;

    // std::cout << "\nKp_shoulderroll: " << kp_shoulderroll_tuned_ << ", Kp_shoulderpitch: " << kp_shoulderpitch_tuned_
    //           << ", Kp_shoulderyaw: " << kp_shoulderyaw_tuned_ << ", Kp_elbow: " << kp_elbow_tuned_;

    return;
}
void Digit_Controller::Set_Initial_Walking_Gains_()
{
    kp_hiproll_sw_tuned_ = kp_hiproll_sw_base_;
    kp_hipyaw_sw_tuned_ = kp_hipyaw_sw_base_;
    kp_hippitch_sw_tuned_ = kp_hippitch_sw_base_;
    kp_knee_sw_tuned_ = kp_knee_sw_base_;
    kp_toe_sw_tuned_ = kp_toe_sw_base_;

    kp_hiproll_st_tuned_ = kp_hiproll_st_base_;
    kp_hipyaw_st_tuned_ = kp_hipyaw_st_base_;
    kp_hippitch_st_tuned_ = kp_hippitch_st_base_;
    kp_knee_st_tuned_ = kp_knee_st_base_;
    kp_toe_st_tuned_ = kp_toe_st_base_;

    kp_shoulderroll_tuned_ = kp_shoulderroll_base_;
    kp_shoulderpitch_tuned_ = kp_shoulderpitch_base_;
    kp_shoulderyaw_tuned_ = kp_shoulderyaw_base_;
    kp_elbow_tuned_ = kp_elbow_base_;
    return;
}

/*==========================================================================
                                PRIVATE MEMBER FUNCTIONS
==========================================================================*/

/***************************  Extract Sensor & State Data ***************************/
void Digit_Controller::Extract_Observation_(double &time_digit,
                                            Ref<Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                            Ref<Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                                            Ref<Matrix<double, NUM_MOTORS, 1>> q_motors,
                                            Ref<Matrix<double, NUM_MOTORS, 1>> qdot_motors,
                                            Ref<Matrix<double, NUM_UNACT_JOINTS, 1>> q_joints,
                                            Ref<Matrix<double, NUM_UNACT_JOINTS, 1>> qdot_joints,
                                            const llapi_observation_t &observ,
                                            const llapi_limits_t *const lim)
{
    time_digit = observ.time;

    // IMU data: Using base which has a rotation (no translation) to IMU frame
    Vector3d q_pos = Map<const Vector3d>(observ.base.translation);
    Vector3d qdot_pos = Map<const Vector3d>(observ.base.linear_velocity);
    // Vector4d q_quat(observ.base.orientation.w,
    //                 observ.base.orientation.x,
    //                 observ.base.orientation.y,
    //                 observ.base.orientation.z);
    Vector3d q_angvel = Map<const Vector3d>(observ.base.angular_velocity);

    // Encoder data
    q_joints = Map<const Matrix<double, constants::NUM_PASSIVE_JOINTS, 1>>(observ.joint.position);
    q_motors = Map<const Matrix<double, constants::NUM_ACTUATOR_JOINTS, 1>>(observ.motor.position);
    qdot_joints = Map<const Matrix<double, constants::NUM_PASSIVE_JOINTS, 1>>(observ.joint.velocity);
    qdot_motors = Map<const Matrix<double, constants::NUM_ACTUATOR_JOINTS, 1>>(observ.motor.velocity);

    // Compute orientation information  % base orientation is corrected for imu transformation
    Vector3d euler_ang_temp;
    Vector3d euler_angvel_temp;
    Vector3d w_temp(observ.base.angular_velocity[0],
                    observ.base.angular_velocity[1],
                    observ.base.angular_velocity[2]);
    Quaterniond quat_temp(observ.base.orientation.w,
                          observ.base.orientation.x,
                          observ.base.orientation.y,
                          observ.base.orientation.z);

    transform_utils::Quaternion_To_Euler_And_Rates(euler_ang_temp, euler_angvel_temp, w_temp, quat_temp);
    Vector3d q_euler = euler_ang_temp;
    Vector3d qdot_euler = euler_angvel_temp;

    // Convert body velocity to world frame velocity (since position is in world frame)
    // std::cout << "qdot_pos: \n" << qdot_pos << "\n\n";
    Matrix<double, 3, 3> Rz_body_to_world;
    double yaw_world = q_euler[0];
    transform_utils::Euler_Yaw_Angle_To_Rotation_Matrix(Rz_body_to_world, yaw_world);
    qdot_pos = Rz_body_to_world * qdot_pos;
    // std::cout << "qdot_pos: \n" << qdot_pos << "\n\n";
    // std::cout << "****\n";

    // Base configuration variables
    Matrix<double, 6, 1> q_base, qdot_base;
    q_base << q_pos, q_euler;
    qdot_base << qdot_pos, qdot_euler;

    // Leg configuration variables (shin joint omitted)
    Matrix<double, constants::NUM_Q_LEG, 1> q_leg_left, q_leg_right, qdot_leg_left, qdot_leg_right;
    q_leg_left << q_motors[DigitMotors::LeftHipRoll],
        q_motors[DigitMotors::LeftHipYaw],
        q_motors[DigitMotors::LeftHipPitch],
        q_motors[DigitMotors::LeftKnee],
        q_joints[DigitJoints::LeftShin],
        q_joints[DigitJoints::LeftTarsus],
        q_joints[DigitJoints::LeftToePitch],
        q_joints[DigitJoints::LeftToeRoll];
    qdot_leg_left << qdot_motors[DigitMotors::LeftHipRoll],
        qdot_motors[DigitMotors::LeftHipYaw],
        qdot_motors[DigitMotors::LeftHipPitch],
        qdot_motors[DigitMotors::LeftKnee],
        qdot_motors[DigitJoints::LeftShin],
        qdot_joints[DigitJoints::LeftTarsus],
        qdot_joints[DigitJoints::LeftToePitch],
        qdot_joints[DigitJoints::LeftToeRoll];
    q_leg_right << q_motors[DigitMotors::RightHipRoll],
        q_motors[DigitMotors::RightHipYaw],
        q_motors[DigitMotors::RightHipPitch],
        q_motors[DigitMotors::RightKnee],
        q_joints[DigitJoints::RightShin],
        q_joints[DigitJoints::RightTarsus],
        q_joints[DigitJoints::RightToePitch],
        q_joints[DigitJoints::RightToeRoll];
    qdot_leg_right << qdot_motors[DigitMotors::RightHipRoll],
        qdot_motors[DigitMotors::RightHipYaw],
        qdot_motors[DigitMotors::RightHipPitch],
        qdot_motors[DigitMotors::RightKnee],
        qdot_motors[DigitJoints::RightShin],
        qdot_joints[DigitJoints::RightTarsus],
        qdot_joints[DigitJoints::RightToePitch],
        qdot_joints[DigitJoints::RightToeRoll];

    // Arm configuration variables
    Matrix<double, constants::NUM_Q_ARM, 1> q_arm_left, q_arm_right, qdot_arm_left, qdot_arm_right;
    q_arm_left << q_motors[DigitMotors::LeftShoulderRoll],
        q_motors[DigitMotors::LeftShoulderPitch],
        q_motors[DigitMotors::LeftShoulderYaw],
        q_motors[DigitMotors::LeftElbow];
    qdot_arm_left << qdot_motors[DigitMotors::LeftShoulderRoll],
        qdot_motors[DigitMotors::LeftShoulderPitch],
        qdot_motors[DigitMotors::LeftShoulderYaw],
        qdot_motors[DigitMotors::LeftElbow];
    q_arm_right << q_motors[DigitMotors::RightShoulderRoll],
        q_motors[DigitMotors::RightShoulderPitch],
        q_motors[DigitMotors::RightShoulderYaw],
        q_motors[DigitMotors::RightElbow];
    qdot_arm_right << qdot_motors[DigitMotors::RightShoulderRoll],
        qdot_motors[DigitMotors::RightShoulderPitch],
        qdot_motors[DigitMotors::RightShoulderYaw],
        qdot_motors[DigitMotors::RightElbow];

    // Body configuration variables
    Matrix<double, 2 * constants::NUM_Q_LEG + 2 * constants::NUM_Q_ARM, 1> q_body, qdot_body; // num_body = 2*constants::NUM_Q_LEG + 2*constants::NUM_Q_ARM;
    q_body << q_leg_left, q_arm_left, q_leg_right, q_arm_right;
    qdot_body << qdot_leg_left, qdot_arm_left, qdot_leg_right, qdot_arm_right;

    // Total configuration variables (FROST order)
    q_all << q_base, q_body;
    qdot_all << qdot_base, qdot_body;

    // std::cout << "q_all: \n"
    //           << q_all << "\n\n";
    // std::cout << "qdot_all: \n"
    //           << qdot_all << "\n\n";
    // std::cout << "q_all (qpos): \n" << q_pos << "\n\n";
    // std::cout << "q_all (qeul): \n" << q_euler << "\n\n";
    // std::cout << "qdot_all (qdot_pos): \n" << qdot_pos << "\n\n";
    // std::cout << "qdot_all (qdot_eul): \n" << qdot_euler << "\n\n";

    // q_all.setZero();
    // qdot_all.setZero();
    // q_all << 15.57,
    //     0.438336,
    //     1.02966,
    //     0.0310896,
    //     -0.0378948,
    //     0.00377518,
    //     0.430196,
    //     0.008701,
    //     0.467445,
    //     0.238548,
    //     0.00177714,
    //     -0.243282,
    //     0.322061,
    //     0.0595425,
    //     -0.148002,
    //     1.14808,
    //     0.000663577,
    //     -0.145645,
    //     -0.398852,
    //     -0.0123601,
    //     -0.248717,
    //     -0.397336,
    //     0.0250206,
    //     0.346091,
    //     -0.0153938,
    //     -0.0357742,
    //     0.149862,
    //     -1.07767,
    //     -0.000467261,
    //     0.146166;

    // qdot_all << 0.613614,
    //     -0.0433168,
    //     0.0149162,
    //     0.139065,
    //     0.343312,
    //     0.0157631,
    //     -0.131296,
    //     -0.0160582,
    //     -2.30971,
    //     -2.72161,
    //     -0.131296,
    //     2.89857,
    //     1.52575,
    //     0.203504,
    //     0.0045138,
    //     0.465106,
    //     2.54046e-05,
    //     0.00219412,
    //     -0.0275772,
    //     0.0713069,
    //     -0.164208,
    //     0.289423,
    //     -1.61981,
    //     -0.0418878,
    //     0.720199,
    //     -0.0329247,
    //     0.0202571,
    //     0.689771,
    //     0.00107993,
    //     0.00202264;

    // q_all(DIGIT_Q_COORD::qbase_yaw) = degToRad(0);
    // qdot_all(DIGIT_Q_COORD::qbase_pos_x) = 1.0;
    // qdot_all(DIGIT_Q_COORD::qbase_pos_y) = 0.0;

    // q_all(DIGIT_Q_COORD::qbase_yaw) = degToRad(90);
    // qdot_all(DIGIT_Q_COORD::qbase_pos_x) = 0.0;
    // qdot_all(DIGIT_Q_COORD::qbase_pos_y) = 1.0;

    return;
}
bool Digit_Controller::Get_Config_Parameters_()
{

    /* System Parameters */
    mass_ = 46.2104;

    /* Standing (Analytic) Parameters */
    standing_roll_des_analytic_ = 0.0;
    standing_pitch_des_analytic_ = 0.0;
    LA_des_analytic_ = 0.619;
    LL_des_analytic_ = 0.80;

    /* Standing (Numeric) Parameters */
    standing_com_height_des_ = 0.98;
    step_width_standing_des_ = 0.3;

    /* Walking Parameters */
    time_step_period_ = 0.3;
    // time_step_period_ = 0.38;
    kf_sample_time_ = 0.001;
    zH_ = 0.95;
    z_clearance_ = 0.1;
    s_clearance_ = 0.5;
    // step_width_ = 0.25;
    step_width_ = 0.3;
    u_knee_comp_ = 150;
    u_hiproll_comp_ = 30;
    fp_type_ = "1step";
    mu_ = 1.0;
    kx_body_ = 0.0;
    ky_body_ = 0.0;

    /* MPC Walking Parameters */
    dt_opt_mpc_ = 0.01;
    N_step_horizon_mpc_ = 4;
    // ufp_x_max_ = 0.5;
    ufp_x_max_ = 0.7;
    ufp_y_min_ = 0.1;
    ufp_y_max_ = 0.6;

    /* Inverse Kinematics Parameters */
    max_iter_ik_ = 20;
    tol_ik_ = 0.0001;
    ik_threshold_ = 0.005;
    scale_delta_ik_ = 0.1;

    /* Target Parameters (General) */
    target_type_ = "keyboard";

    /* Keyboard Target Parameters (Standing)*/
    x_offset_base_ = 0.0;
    y_offset_base_ = 0.0;
    z_offset_base_ = 0.0;
    yaw_offset_base_ = 0.0;
    pitch_offset_base_ = 0.0;
    roll_offset_base_ = 0.0;

    x_offset_inc_ = 0.0;
    y_offset_inc_ = 0.0;
    z_offset_inc_ = 0.0;
    yaw_offset_inc_ = 0.0;
    pitch_offset_inc_ = 0.0;
    roll_offset_inc_ = 0.0;

    x_offset_min_ = -0.5;
    y_offset_min_ = -0.05;
    z_offset_min_ = -0.4;
    yaw_offset_min_ = -0.5;
    pitch_offset_min_ = -0.5;
    roll_offset_min_ = -0.5;

    x_offset_max_ = 0.5;
    y_offset_max_ = 0.05;
    z_offset_max_ = 0;
    yaw_offset_max_ = 0.5;
    pitch_offset_max_ = 0.5;
    roll_offset_max_ = 0.5;

    standing_target_filter_param_ = 0.01;

    time_sway_yaw_period_ = 4;
    time_sway_pitch_period_ = 4;
    time_sway_roll_period_ = 4;
    time_sway_crouch_period_ = 4;
    sway_yaw_max_ = 0.25;
    sway_pitch_max_ = 0.15;
    sway_roll_max_ = 0.3;
    sway_crouch_max_ = 0;
    sway_crouch_min_ = -0.2;

    /* Keyboard Target Parameters (Walking)*/
    vel_x_filter_param_ = 0.01;
    vel_y_filter_param_ = 0.01;
    turn_rps_filter_param_ = 0.01;

    vel_x_des_base_ = 0;
    vel_y_des_base_ = 0;
    turn_rps_base_ = 0;

    vel_x_des_inc_ = 0;
    vel_y_des_inc_ = 0;
    turn_rps_inc_ = 0;

    vel_x_max_ = 0.1;
    vel_y_max_ = 0.1;
    turn_rps_max_ = 0.1;

    // Standing
    kp_hiproll_stand_base_ = 500;
    kp_hipyaw_stand_base_ = 300;
    kp_hippitch_stand_base_ = 300;
    kp_knee_stand_base_ = 800;
    kp_toe_stand_base_ = 200;
    kp_shoulderroll_stand_base_ = 100;
    kp_shoulderpitch_stand_base_ = 100;
    kp_shoulderyaw_stand_base_ = 100;
    kp_elbow_stand_base_ = 100;

    kp_lateral_stand_base_ = 1.1;
    kd_lateral_stand_base_ = 0.06;

    kp_knee_comp_stand_base_ = 1000;
    kd_knee_comp_stand_base_ = 20;

    // Walking
    kp_hiproll_sw_base_ = 500;
    kp_hipyaw_sw_base_ = 800;
    kp_hippitch_sw_base_ = 500;
    kp_knee_sw_base_ = 1000;
    kp_toe_sw_base_ = 200;

    kp_hiproll_st_base_ = 800;
    kp_hipyaw_st_base_ = 800;
    kp_hippitch_st_base_ = 800;
    kp_knee_st_base_ = 1000;

    kp_shoulderroll_base_ = 100;
    kp_shoulderpitch_base_ = 100;
    kp_shoulderyaw_base_ = 100;
    kp_elbow_base_ = 100;

    // Log Parameters
    path_to_gains_ = "";
    path_to_log_ = "";

    /* ==========================================
     *  Below parameters assigned with ros config
     *  ========================================== */
    // bool received_all = true;

    // /* System Parameters */
    // ros_utils::Check_Ros_Parameters(nh_, "digit_system/mass", mass_, received_all);

    // /* Standing (Analytic) Parameters */
    // ros_utils::Check_Ros_Parameters(nh_, "digit_standing/ik_analytic/standing_roll_des_analytic", standing_roll_des_analytic_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_standing/ik_analytic/standing_pitch_des_analytic", standing_pitch_des_analytic_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_standing/ik_analytic/LA_des_analytic", LA_des_analytic_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_standing/ik_analytic/LL_des_analytic", LL_des_analytic_, received_all);

    // /* Standing (Numeric) Parameters */
    // ros_utils::Check_Ros_Parameters(nh_, "digit_standing/ik_numeric/standing_com_height_des", standing_com_height_des_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_standing/ik_numeric/step_width_standing_des", step_width_standing_des_, received_all);

    // /* Walking Parameters */
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/time_step_period", time_step_period_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/kf_sample_time", kf_sample_time_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/zH", zH_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/z_clearance", z_clearance_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/s_clearance", s_clearance_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/step_width", step_width_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/u_knee_comp", u_knee_comp_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/u_hiproll_comp", u_hiproll_comp_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/fp_type", fp_type_, received_all);

    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/mu", mu_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/kx_body", kx_body_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_walking/ky_body", ky_body_, received_all);

    // /* MPC Walking Parameters */
    // ros_utils::Check_Ros_Parameters(nh_, "digit_mpc/dt_opt_mpc", dt_opt_mpc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_mpc/N_step_horizon", N_step_horizon_mpc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_mpc/ufp_x_max", ufp_x_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_mpc/ufp_y_min", ufp_y_min_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_mpc/ufp_y_max", ufp_y_max_, received_all);

    // /* Inverse Kinematics Parameters */
    // ros_utils::Check_Ros_Parameters(nh_, "digit_ik/max_iter_ik", max_iter_ik_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_ik/tol_ik", tol_ik_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_ik/ik_threshold", ik_threshold_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_ik/scale_delta_ik", scale_delta_ik_, received_all);

    // /* Target Parameters (General) */
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/target_type", target_type_, received_all);

    // /* Keyboard Target Parameters (Standing)*/
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/x_base", x_offset_base_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/y_base", y_offset_base_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/z_base", z_offset_base_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/yaw_base", yaw_offset_base_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/pitch_base", pitch_offset_base_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/roll_base", roll_offset_base_, received_all);

    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/x_inc", x_offset_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/y_inc", y_offset_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/z_inc", z_offset_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/yaw_inc", yaw_offset_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/pitch_inc", pitch_offset_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/roll_inc", roll_offset_inc_, received_all);

    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/x_min", x_offset_min_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/y_min", y_offset_min_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/z_min", z_offset_min_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/yaw_min", yaw_offset_min_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/pitch_min", pitch_offset_min_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/roll_min", roll_offset_min_, received_all);

    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/x_max", x_offset_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/y_max", y_offset_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/z_max", z_offset_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/yaw_max", yaw_offset_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/pitch_max", pitch_offset_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/roll_max", roll_offset_max_, received_all);

    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/filter_param", standing_target_filter_param_, received_all);

    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/T_sway_yaw_period", time_sway_yaw_period_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/T_sway_pitch_period", time_sway_pitch_period_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/T_sway_roll_period", time_sway_roll_period_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/T_sway_crouch_period", time_sway_crouch_period_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/sway_yaw_max", sway_yaw_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/sway_pitch_max", sway_pitch_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/sway_roll_max", sway_roll_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/sway_crouch_max", sway_crouch_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/standing_keyboard/sway_crouch_min", sway_crouch_min_, received_all);

    // /* Keyboard Target Parameters (Walking)*/
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_keyboard/vel_x_filter", vel_x_filter_param_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_keyboard/vel_y_filter", vel_y_filter_param_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_keyboard/turn_rps_filter", turn_rps_filter_param_, received_all);

    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_keyboard/vel_x_base", vel_x_des_base_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_keyboard/vel_y_base", vel_y_des_base_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_keyboard/turn_rps_base", turn_rps_base_, received_all);

    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_keyboard/vel_x_inc", vel_x_des_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_keyboard/vel_y_inc", vel_y_des_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_keyboard/turn_rps_inc", turn_rps_inc_, received_all);

    // /* Gamepad Target Parameters (Walking) */
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_gamepad/vel_x_max", vel_x_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_gamepad/vel_y_max", vel_y_max_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_targets/walking_gamepad/turn_rps_max", turn_rps_max_, received_all);

    // /* Gain Parameters */
    // ros_utils::Check_Ros_Parameters(nh_, "digit_gains/inc/kp_hiproll_inc", kp_hiproll_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_gains/inc/kp_hipyaw_inc", kp_hipyaw_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_gains/inc/kp_hippitch_inc", kp_hippitch_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_gains/inc/kp_knee_inc", kp_knee_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_gains/inc/kp_toe_inc", kp_toe_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_gains/inc/kp_shoulderroll_inc", kp_shoulderroll_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_gains/inc/kp_shoulderpitch_inc", kp_shoulderpitch_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_gains/inc/kp_shoulderyaw_inc", kp_shoulderyaw_inc_, received_all);
    // ros_utils::Check_Ros_Parameters(nh_, "digit_gains/inc/kp_elbow_inc", kp_elbow_inc_, received_all);

    // // Sim Gains
    // if (exp_mode_ == "sim")
    // {
    //     // Standing
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_hiproll_stand", kp_hiproll_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_hipyaw_stand", kp_hipyaw_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_hippitch_stand", kp_hippitch_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_knee_stand", kp_knee_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_toe_stand", kp_toe_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_shoulderroll_stand", kp_shoulderroll_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_shoulderpitch_stand", kp_shoulderpitch_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_shoulderyaw_stand", kp_shoulderyaw_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_elbow_stand", kp_elbow_stand_base_, received_all);

    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/numeric/kp_knee_comp_stand", kp_knee_comp_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/numeric/kd_knee_comp_stand", kd_knee_comp_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/numeric/kp_knee_comp_stand_inc", kp_knee_comp_stand_inc_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/numeric/kd_knee_comp_stand_inc", kd_knee_comp_stand_inc_, received_all);

    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/analytic/kp_lateral_stand", kp_lateral_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/analytic/kd_lateral_stand", kd_lateral_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/analytic/kp_lateral_stand_inc", kp_lateral_stand_inc_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/analytic/kd_lateral_stand_inc", kd_lateral_stand_inc_, received_all);

    //     // Walking
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_hiproll_stance", kp_hiproll_st_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_hipyaw_stance", kp_hipyaw_st_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_hippitch_stance", kp_hippitch_st_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_knee_stance", kp_knee_st_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_toe_stance", kp_toe_st_base_, received_all);

    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_hiproll_swing", kp_hiproll_sw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_hipyaw_swing", kp_hipyaw_sw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_hippitch_swing", kp_hippitch_sw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_knee_swing", kp_knee_sw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_toe_swing", kp_toe_sw_base_, received_all);

    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_shoulderroll", kp_shoulderroll_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_shoulderpitch", kp_shoulderpitch_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_shoulderyaw", kp_shoulderyaw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/sim/kp_elbow", kp_elbow_base_, received_all);

    //     // Log Parameters
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_log/path_to_log_sim", path_to_log_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_log/path_to_gains_sim", path_to_gains_, received_all);
    // }
    // else if (exp_mode_ == "real") // Real gains
    // {
    //     // Standing
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_hiproll_stand", kp_hiproll_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_hipyaw_stand", kp_hipyaw_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_hippitch_stand", kp_hippitch_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_knee_stand", kp_knee_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_toe_stand", kp_toe_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_shoulderroll_stand", kp_shoulderroll_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_shoulderpitch_stand", kp_shoulderpitch_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_shoulderyaw_stand", kp_shoulderyaw_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_elbow_stand", kp_elbow_stand_base_, received_all);

    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/numeric/kp_knee_comp_stand", kp_knee_comp_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/numeric/kd_knee_comp_stand", kd_knee_comp_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/numeric/kp_knee_comp_stand_inc", kp_knee_comp_stand_inc_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/numeric/kd_knee_comp_stand_inc", kd_knee_comp_stand_inc_, received_all);

    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/analytic/kp_lateral_stand", kp_lateral_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/analytic/kd_lateral_stand", kd_lateral_stand_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/analytic/kp_lateral_stand_inc", kp_lateral_stand_inc_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/analytic/kd_lateral_stand_inc", kd_lateral_stand_inc_, received_all);

    //     // Walking
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_hiproll_stance", kp_hiproll_st_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_hipyaw_stance", kp_hipyaw_st_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_hippitch_stance", kp_hippitch_st_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_knee_stance", kp_knee_st_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_toe_stance", kp_toe_st_base_, received_all);

    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_hiproll_swing", kp_hiproll_sw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_hipyaw_swing", kp_hipyaw_sw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_hippitch_swing", kp_hippitch_sw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_knee_swing", kp_knee_sw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_toe_swing", kp_toe_sw_base_, received_all);

    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_shoulderroll", kp_shoulderroll_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_shoulderpitch", kp_shoulderpitch_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_shoulderyaw", kp_shoulderyaw_base_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_gains/real/kp_elbow", kp_elbow_base_, received_all);

    //     // Log Parameters
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_log/path_to_log_real", path_to_log_, received_all);
    //     ros_utils::Check_Ros_Parameters(nh_, "digit_log/path_to_gains_real", path_to_gains_, received_all);
    // }

    return true;
}
void Digit_Controller::Compute_Grf_()
{
    // Use heelspring deflection to compute

    return;
}

/*************************** Standing Controller ***************************/
void Digit_Controller::Standing_Analytic_Update_(llapi_command_t &cmd,
                                                 const double &time_system,
                                                 const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                                 const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                                                 const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors,
                                                 const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors,
                                                 const llapi_limits_t *const lim)
{

    /* Initialize known desired motor positions/velocities */
    double q_LeftHipRoll_des = 0.3752458;
    double q_LeftHipYaw_des = 0.0;
    double q_LeftShoulderRoll_des = -0.3; // arms from default agility controller
    double q_LeftShoulderPitch_des = 0.943845;
    double q_LeftShoulderYaw_des = 0.0;
    double q_LeftElbow_des = 0.3633;

    double qdot_LeftHipRoll_des = 0.0;
    double qdot_LeftHipYaw_des = 0.0;
    double qdot_LeftHipPitch_des = 0.0;
    double qdot_LeftKnee_des = 0.0;
    double qdot_LeftShoulderRoll_des = 0.0;
    double qdot_LeftShoulderPitch_des = 0.0;
    double qdot_LeftShoulderYaw_des = 0.0;
    double qdot_LeftElbow_des = 0.0;

    double q_RightHipRoll_des = -q_LeftHipRoll_des; //-0.3752;
    double q_RightHipYaw_des = -q_LeftHipYaw_des;
    double q_RightShoulderRoll_des = -q_LeftShoulderRoll_des; // arms from default agility controller
    double q_RightShoulderPitch_des = -q_LeftShoulderPitch_des;
    double q_RightShoulderYaw_des = -q_LeftShoulderYaw_des;
    double q_RightElbow_des = -q_LeftElbow_des;

    double qdot_RightHipRoll_des = -qdot_LeftHipRoll_des;
    double qdot_RightHipYaw_des = -qdot_LeftHipYaw_des;
    double qdot_RightHipPitch_des = -qdot_LeftHipPitch_des;
    double qdot_RightKnee_des = -qdot_LeftKnee_des;
    double qdot_RightShoulderRoll_des = -qdot_LeftShoulderRoll_des;
    double qdot_RightShoulderPitch_des = -qdot_LeftShoulderPitch_des;
    double qdot_RightShoulderYaw_des = -qdot_LeftShoulderYaw_des;
    double qdot_RightElbow_des = -qdot_LeftElbow_des;

    double q_LeftHipPitch_des, q_LeftKnee_des, q_RightHipPitch_des, q_RightKnee_des;
    double q_roll = q_all(DIGIT_Q_COORD::qbase_roll);
    double qdot_roll = qdot_all(DIGIT_Q_COORD::qbase_roll);
    double left_roll_tune = -kp_lateral_stand_tuned_ * (q_roll - standing_roll_des_analytic_ + shift_lateral_) - kd_lateral_stand_tuned_ * qdot_roll; // shift_lateral added for transition from standing to walking
    double LL_left_des = std::min(LL_des_analytic_ + left_roll_tune, 0.9);
    double right_roll_tune = kp_lateral_stand_tuned_ * (q_roll - standing_roll_des_analytic_ + shift_lateral_) + kd_lateral_stand_tuned_ * qdot_roll;
    double LL_right_des = std::min(LL_des_analytic_ + right_roll_tune, 0.9);

    Digit_Controller::Standing_Analytic_Ik_From_Thigh_Knee_(q_LeftHipPitch_des, q_LeftKnee_des, LA_des_analytic_, LL_left_des, "left");     // Left
    Digit_Controller::Standing_Analytic_Ik_From_Thigh_Knee_(q_RightHipPitch_des, q_RightKnee_des, LA_des_analytic_, LL_right_des, "right"); // Right
    // q_LeftHipPitch_des -= 0.2;
    q_LeftKnee_des += 0.3;
    // q_RightHipPitch_des += 0.2;// Knee compensation for upright posture
    q_RightKnee_des -= 0.3;

    /* Desired Toe Pitch */
    double q_LeftToeA_des, q_LeftToeB_des, q_RightToeA_des, q_RightToeB_des;
    double qdot_LeftToeA_des, qdot_LeftToeB_des, qdot_RightToeA_des, qdot_RightToeB_des;

    // Center of mass position and velocity
    Vector3d p_com_world;
    Vector3d pdot_com_world;
    gen::kin::p_COM(p_com_world, q_all);
    gen::kin::v_COM(pdot_com_world, q_all, qdot_all);

    // Cartesian foot positions
    Vector3d p_left_toe_world;
    Vector3d p_right_toe_world;
    gen::kin::p_LeftToeFront(p_left_toe_world, q_all);
    gen::kin::p_RightToeFront(p_right_toe_world, q_all);

    // Transform positions into sagittal frame
    Matrix3d Rz_heading;
    transform_utils::Euler_Yaw_Angle_To_Rotation_Matrix(Rz_heading, q_all(DIGIT_Q_COORD::qbase_yaw)); // align body and world yaw for sagittal control
    Vector3d p_com_aligned = Rz_heading.transpose() * p_com_world;
    Vector3d pdot_com_aligned = Rz_heading.transpose() * pdot_com_world;
    Vector3d p_left_toe_aligned = Rz_heading.transpose() * p_left_toe_world;
    Vector3d p_right_toe_aligned = Rz_heading.transpose() * p_right_toe_world;

    // Compute x pos error between average foot and COM; set as error for PD control */
    Vector3d p_AvgToe = 0.5 * (p_left_toe_aligned + p_right_toe_aligned);
    double q_ToePitch_error = (p_com_aligned[0] - p_AvgToe[0]);
    q_ToePitch_error = q_ToePitch_error + 0.115254;   // COM error compensation (toe joint position not the best set point for COM)
    double qdot_ToePitch_error = pdot_com_aligned[0]; // Reduce base velocity (could also use cente of mass)

    // Update Desired Toe position (current position added now to get subtracted later in error calc)
    q_LeftToeA_des = q_ToePitch_error + q_motors[DigitMotors::LeftToeA];
    q_LeftToeB_des = -q_ToePitch_error + q_motors[DigitMotors::LeftToeB];
    q_RightToeA_des = -q_ToePitch_error + q_motors[DigitMotors::RightToeA];
    q_RightToeB_des = q_ToePitch_error + q_motors[DigitMotors::RightToeB];

    qdot_LeftToeA_des = qdot_ToePitch_error + qdot_motors[DigitMotors::LeftToeA];
    qdot_LeftToeB_des = qdot_ToePitch_error + qdot_motors[DigitMotors::LeftToeB];
    qdot_RightToeA_des = qdot_ToePitch_error + qdot_motors[DigitMotors::RightToeA];
    qdot_RightToeB_des = qdot_ToePitch_error + qdot_motors[DigitMotors::RightToeB];

    /* Create desired joints vector */
    // order: left leg, right leg, left arm, right arm
    // toeA and toeB terms omitted because PD error is directly calculated above
    Matrix<double, NUM_MOTORS, 1> q_motors_des, qdot_motors_des;
    q_motors_des << q_LeftHipRoll_des, q_LeftHipYaw_des, q_LeftHipPitch_des, q_LeftKnee_des, q_LeftToeA_des, q_LeftToeB_des,
        q_RightHipRoll_des, q_RightHipYaw_des, q_RightHipPitch_des, q_RightKnee_des, q_RightToeA_des, q_RightToeB_des,
        q_LeftShoulderRoll_des, q_LeftShoulderPitch_des, q_LeftShoulderYaw_des, q_LeftElbow_des,
        q_RightShoulderRoll_des, q_RightShoulderPitch_des, q_RightShoulderYaw_des, q_RightElbow_des;
    qdot_motors_des << qdot_LeftHipRoll_des, qdot_LeftHipYaw_des, qdot_LeftHipPitch_des, qdot_LeftKnee_des, qdot_LeftToeA_des, qdot_LeftToeB_des,
        qdot_RightHipRoll_des, qdot_RightHipYaw_des, qdot_RightHipPitch_des, qdot_RightKnee_des, qdot_RightToeA_des, qdot_RightToeB_des,
        qdot_LeftShoulderRoll_des, qdot_LeftShoulderPitch_des, qdot_LeftShoulderYaw_des, qdot_LeftElbow_des,
        qdot_RightShoulderRoll_des, qdot_RightShoulderPitch_des, qdot_RightShoulderYaw_des, qdot_RightElbow_des;

    /* Set Gain Matrices */
    Matrix<double, NUM_MOTORS, 1> kp_stand;
    kp_stand << kp_hiproll_stand_tuned_, kp_hipyaw_stand_tuned_, kp_hippitch_stand_tuned_, kp_knee_stand_tuned_, kp_toe_stand_tuned_, kp_toe_stand_tuned_,
        kp_hiproll_stand_tuned_, kp_hipyaw_stand_tuned_, kp_hippitch_stand_tuned_, kp_knee_stand_tuned_, kp_toe_stand_tuned_, kp_toe_stand_tuned_,
        kp_shoulderroll_stand_tuned_, kp_shoulderpitch_stand_tuned_, kp_shoulderyaw_stand_tuned_, kp_elbow_stand_tuned_,
        kp_shoulderroll_stand_tuned_, kp_shoulderpitch_stand_tuned_, kp_shoulderyaw_stand_tuned_, kp_elbow_stand_tuned_;

    /* PD Torque Control */
    Matrix<double, NUM_MOTORS, 1> q_motors_error = q_motors - q_motors_des;
    Matrix<double, NUM_MOTORS, 1> motor_torque_command = -kp_stand.cwiseProduct(q_motors_error);

    // Ramp Torque at begginning (need to tie it to set operation mode)
    // double t_ramp_period = 5.0;
    // if (t_system < t_ramp_period)
    // {
    //     Digit_Controller::ramp_torque(t_ramp_period);
    // }

    // Saturate torque if necessary
    Digit_Controller::Saturate_Torque_Command_(motor_torque_command, lim);

    /* Command Update */
    for (int i = 0; i < constants::NUM_ACTUATOR_JOINTS; ++i)
    {
        cmd.motors[i].torque = motor_torque_command[i];
        cmd.motors[i].velocity = 0.0;
        cmd.motors[i].damping = 1.0 * lim->damping_limit[i];
    }
    cmd.fallback_opmode = Damping;
    cmd.apply_command = true;

    /* Log data */
    // Digit_Controller::Standing_Analytic_Log_Data_(time_system,
    //                                               q_all,
    //                                               qdot_all,
    //                                               q_motors,
    //                                               qdot_motors,
    //                                               q_motors_des,
    //                                               qdot_motors_des,
    //                                               motor_torque_command,
    //                                               y_vc_stand,
    //                                               q_all_des,
    //                                               h_geom_act,
    //                                               h_geom_des,
    //                                               p_com_world,
    //                                               p_left_toe_world,
    //                                               p_right_toe_world);

    return;
}
void Digit_Controller::Standing_Analytic_Fk_From_Leg_Angle_Length_(double &LA,
                                                                   double &LL,
                                                                   const double &q_hippitch,
                                                                   const double &q_knee)
{
    double t2 = q_knee + 7.0 / 2.0e+2;
    double t3 = cos(t2);
    double t4 = t3 * 5.292e-1;
    double t5 = t4 + 5.300526400000001e-1;
    LA = q_hippitch - acos(1.0 / sqrt(t5) * (t4 + 5.601052800000001e-1) * 9.44822373393802e-1) + 1.0 / 1.0e+1;
    LL = sqrt(t5);

    return;
}
void Digit_Controller::Standing_Analytic_Ik_From_Thigh_Knee_(double &q_hippitch,
                                                             double &q_knee,
                                                             const double &LA,
                                                             const double &LL,
                                                             std::string whichLeg)
{
    double t2 = LL * LL;
    q_hippitch = LA + acos(((t2 + 3.005264000000002e-2) * 9.44822373393802e-1) / LL) - 1.0 / 1.0e+1;

    q_knee = -acos(t2 * 1.889644746787604 - 1.001611186696901) - 7.0 / 2.0e+2;

    q_hippitch = q_hippitch - constants::PI / 4.0;
    q_knee = q_knee + constants::PI / 2.0;

    if (whichLeg == "right")
    {
        q_hippitch = -q_hippitch;
        q_knee = -q_knee;
    }

    return;
}
void Digit_Controller::Standing_Analytic_Log_Data_(const double &time_system,
                                                   const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                                   const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                                                   const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors_act,
                                                   const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors_act,
                                                   const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors_des,
                                                   const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors_des,
                                                   const Ref<const Matrix<double, NUM_MOTORS, 1>> motor_torque_command,
                                                   const Ref<const Matrix<double, 3, 1>> y_vc_stand,
                                                   const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all_des,
                                                   const Ref<const Vector4d> h_geom_act,
                                                   const Ref<const Vector4d> h_geom_des,
                                                   const Ref<const Vector3d> p_com,
                                                   const Ref<const Vector3d> p_left_toe,
                                                   const Ref<const Vector3d> p_right_toe)
{
    ctrl_logfile_.open(path_to_log_, std::ofstream::out | std::ofstream::app);

    /* Log order
            time_system,
            q_all,
            qdot_all,const double &time_system,
            y_vc_stand,
            q_all_des,
            h_geom_act,
            h_geom_des,
            p_com,
            p_left_toe,
            p_right_toe,

    */
    // time_system
    ctrl_logfile_ << std::fixed << time_system << ",\n";

    /* q_all */
    for (auto i : std::vector<double>(q_all.data(), q_all.data() + q_all.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_all */
    for (auto i : std::vector<double>(qdot_all.data(), qdot_all.data() + qdot_all.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    // /* q_iter_stand */
    // for (auto i : std::vector<double>(q_iter_stand.data(), q_iter_stand.data() + q_iter_stand.size()))
    // {
    //     ctrl_logfile_ << i << ",";
    // }
    // ctrl_logfile_ << ",\n";

    // /* qdot_iter_stand */
    // for (auto i : std::vector<double>(qdot_iter_stand.data(), qdot_iter_stand.data() + qdot_iter_stand.size()))
    // {
    //     ctrl_logfile_ << i << ",";
    // }
    // ctrl_logfile_ << ",\n";

    /* q_motors_act */
    for (auto i : std::vector<double>(q_motors_act.data(), q_motors_act.data() + q_motors_act.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_motors_act */
    for (auto i : std::vector<double>(qdot_motors_act.data(), qdot_motors_act.data() + qdot_motors_act.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* q_motors_des */
    for (auto i : std::vector<double>(q_motors_des.data(), q_motors_des.data() + q_motors_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_motors_des */
    for (auto i : std::vector<double>(qdot_motors_des.data(), qdot_motors_des.data() + qdot_motors_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* motor_torque_command */
    for (auto i : std::vector<double>(motor_torque_command.data(), motor_torque_command.data() + motor_torque_command.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* y_vc_stand */
    for (auto i : std::vector<double>(y_vc_stand.data(), y_vc_stand.data() + y_vc_stand.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* q_all_des */
    for (auto i : std::vector<double>(q_all_des.data(), q_all_des.data() + q_all_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* h_geom_act */
    for (auto i : std::vector<double>(h_geom_des.data(), h_geom_des.data() + h_geom_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* h_geom_des */
    for (auto i : std::vector<double>(h_geom_act.data(), h_geom_act.data() + h_geom_act.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    // /* delta_q */
    // for (auto i : std::vector<double>(delta_q_square.data(), delta_q_square.data() + delta_q_square.size()))
    // {
    //     ctrl_logfile_ << i << ",";
    // }
    // ctrl_logfile_ << ",\n";

    /* p_com */
    for (auto i : std::vector<double>(p_com.data(), p_com.data() + p_com.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* p_left_toe */
    for (auto i : std::vector<double>(p_left_toe.data(), p_left_toe.data() + p_left_toe.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* p_right_toe */
    for (auto i : std::vector<double>(p_right_toe.data(), p_right_toe.data() + p_right_toe.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    ctrl_logfile_.close();

    return;
}

/*************************** Standing numerical IK Controller ***************************/
void Digit_Controller::Standing_Numeric_Update_(llapi_command_t &cmd,
                                                const double &time_system,
                                                const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                                const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                                                const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors,
                                                const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors,
                                                const Ref<const Matrix<double, NUM_UNACT_JOINTS, 1>> q_joints,
                                                const Ref<const Matrix<double, NUM_UNACT_JOINTS, 1>> qdot_joints,
                                                const llapi_limits_t *const lim)
{

    if (flag_standing_first_iter_)
    {
        yaw_init_ = q_all(DIGIT_Q_COORD::qbase_yaw);
        flag_standing_first_iter_ = false;
    }

    /* Avg heading */
    double heading;
    Matrix3d Rz_heading;
    transform_utils::Wrap_To_Pi(heading, q_all(DIGIT_Q_COORD::qbase_yaw)); // wrapped avg yaw direction
    // transform_utils::Wrap_To_Pi(heading, q_all(DIGIT_Q_COORD::qbase_yaw) + 0.5*(q_all(DIGIT_Q_COORD::qleftHipYaw) + q_all(DIGIT_Q_COORD::qrightHipYaw))); // wrapped avg yaw direction
    transform_utils::Euler_Yaw_Angle_To_Rotation_Matrix(Rz_heading, heading);

    /* Aligned toes */
    Vector3d p_left_toe_world, p_right_toe_world;
    gen::kin::p_toe_pitch_joint_left(p_left_toe_world, q_all);
    gen::kin::p_toe_pitch_joint_right(p_right_toe_world, q_all);
    Vector3d p_left_toe_aligned = Rz_heading.transpose() * p_left_toe_world;
    Vector3d p_right_toe_aligned = Rz_heading.transpose() * p_right_toe_world;

    /* Aligned com */
    Vector3d p_com_world, v_com_world;
    gen::kin::p_COM(p_com_world, q_all);
    gen::kin::v_COM(v_com_world, q_all, qdot_all);
    Vector3d p_com_aligned = Rz_heading.transpose() * p_com_world;
    Vector3d v_com_aligned = Rz_heading.transpose() * v_com_world;

    /* Virtual leg length */
    Matrix<double, 1, 1> LL_left_mat, LL_right_mat;
    gen::kin::LL_left(LL_left_mat, q_all);
    gen::kin::LL_right(LL_right_mat, q_all);

    /* Virtual Constraint */
    Matrix<double, 6, 1> y_vc_stand;
    Matrix<double, 1, 1> heading_mat(heading);
    gen::kin::y_vc_stand(y_vc_stand, q_all, heading_mat);

    /* Update targets based on sway values */
    double x_offset_des, y_offset_des, z_offset_des;
    double yaw_offset_des, pitch_offset_des, roll_offset_des;
    if (enable_sway_yaw_ || enable_sway_pitch_ || enable_sway_roll_ || enable_sway_crouch_)
    {
        std::cout << "Enter sway function\n\n";
        Digit_Controller::Standing_Sway_(time_system, q_all, x_offset_des, y_offset_des, z_offset_des, yaw_offset_des, pitch_offset_des, roll_offset_des); // uses the tuned
    }
    else
    {
        x_offset_des = x_offset_tuned_;
        y_offset_des = y_offset_tuned_;
        z_offset_des = z_offset_tuned_;
        yaw_offset_des = yaw_offset_tuned_;
        pitch_offset_des = pitch_offset_tuned_;
        roll_offset_des = roll_offset_tuned_;
    }

    /* Clamp Standing Targets */
    double x_offset_clamped, y_offset_clamped, z_offset_clamped, yaw_offset_clamped, pitch_offset_clamped, roll_offset_clamped;
    Digit_Controller::Standing_Clamp_Targets_(x_offset_clamped, y_offset_clamped, z_offset_clamped, yaw_offset_clamped, pitch_offset_clamped, roll_offset_clamped,
                                              x_offset_des, y_offset_des, z_offset_des, yaw_offset_des, pitch_offset_des, roll_offset_des);

    /* Filter Standing Targets */
    if (true)
    {
        Digit_Controller::Standing_Filter_Targets_(x_offset_clamped, y_offset_clamped, z_offset_clamped, yaw_offset_clamped, pitch_offset_clamped, roll_offset_clamped);
    }
    else
    {
        x_offset_filtered_ = x_offset_clamped;
        y_offset_filtered_ = y_offset_clamped;
        z_offset_filtered_ = z_offset_clamped;
        yaw_offset_filtered_ = yaw_offset_clamped;
        pitch_offset_filtered_ = pitch_offset_clamped;
        roll_offset_filtered_ = roll_offset_clamped;
    }

    /* Arm IK */
    // Vector3d left_hand_test, right_hand_test;
    // gen::kin::p_left_hand_wrt_base(left_hand_test, q_all, heading_mat);
    // gen::kin::p_right_hand_wrt_base(right_hand_test, q_all, heading_mat);
    // std::cout << "left: \n"
    //           << left_hand_test << "\n\n";
    // std::cout << "right: \n"
    //           << right_hand_test << "\n\n";

    Vector3d left_arm_wrt_base_aligned_des(0.15, 0.4, 0.1);
    Vector3d right_arm_wrt_base_aligned_des(0, -0.4, -0.2);

    Matrix<double, 6, 1> y_vc_arm;
    y_vc_arm << left_arm_wrt_base_aligned_des, right_arm_wrt_base_aligned_des;
    Matrix<double, 6, 1> ydot_vc_arm;
    ydot_vc_arm.setZero();

    Matrix<double, constants::NUM_Q_ALL, 1> q_all_arm_des, qdot_all_arm_des;
    q_all_arm_des = q_all;
    q_all_arm_des(DIGIT_Q_COORD::qbase_yaw) = yaw_init_ + yaw_offset_filtered_;
    q_all_arm_des(DIGIT_Q_COORD::qbase_pitch) = pitch_offset_filtered_;
    q_all_arm_des(DIGIT_Q_COORD::qbase_roll) = roll_offset_filtered_;

    // Digit_Controller::Standing_Compute_Arm_IK_(q_all_arm_des,
    //                                            qdot_all_arm_des,
    //                                            heading_mat,
    //                                            y_vc_arm,
    //                                            ydot_vc_arm);

    /* Main IK */
    // des needs an input to the lateral and forward positions of the desired vc
    Matrix<double, 6, 1> y_vc_stand_des(x_offset_filtered_, -0.5 * step_width_standing_des_ + y_offset_filtered_, standing_com_height_des_ + z_offset_filtered_, x_offset_filtered_, 0.5 * step_width_standing_des_ + y_offset_filtered_, standing_com_height_des_ + z_offset_filtered_); // com_wrt_left_x, com_wrt_left_y, com_wrt_left_z, com_wrt_right_x, com_wrt_right_y, com_wrt_right_z

    /* q_all_des rigid springs assumption */
    Matrix<double, constants::NUM_Q_ALL, 1> q_all_des = q_all;

    /* Fix euler coordinates to desired position */
    q_all_des(DIGIT_Q_COORD::qbase_yaw) = yaw_init_ + yaw_offset_filtered_;
    q_all_des(DIGIT_Q_COORD::qbase_pitch) = pitch_offset_filtered_;
    q_all_des(DIGIT_Q_COORD::qbase_roll) = roll_offset_filtered_;

    /* Fix arm and hipyaw joints to desired values */
    q_all_des(DIGIT_Q_COORD::qleftHipYaw) = 0;
    q_all_des(DIGIT_Q_COORD::qrightHipYaw) = 0;

    q_all_des(DIGIT_Q_COORD::qleftShoulderRoll) = q_all_arm_des(DIGIT_Q_COORD::qleftShoulderRoll);
    q_all_des(DIGIT_Q_COORD::qleftShoulderPitch) = q_all_arm_des(DIGIT_Q_COORD::qleftShoulderPitch);
    q_all_des(DIGIT_Q_COORD::qleftShoulderYaw) = q_all_arm_des(DIGIT_Q_COORD::qleftShoulderYaw);
    q_all_des(DIGIT_Q_COORD::qleftElbow) = q_all_arm_des(DIGIT_Q_COORD::qleftElbow);

    q_all_des(DIGIT_Q_COORD::qrightShoulderRoll) = q_all_arm_des(DIGIT_Q_COORD::qrightShoulderRoll);
    q_all_des(DIGIT_Q_COORD::qrightShoulderPitch) = q_all_arm_des(DIGIT_Q_COORD::qrightShoulderPitch);
    q_all_des(DIGIT_Q_COORD::qrightShoulderYaw) = q_all_arm_des(DIGIT_Q_COORD::qrightShoulderYaw);
    q_all_des(DIGIT_Q_COORD::qrightElbow) = q_all_arm_des(DIGIT_Q_COORD::qrightElbow);

    /* Rigid spring constraint
    q_all_des(DIGIT_Q_COORD::qleftKneeToShin) = 0;
    q_all_des(DIGIT_Q_COORD::qrightKneeToShin) = 0;
    q_all_des(DIGIT_Q_COORD::qleftShinToTarsus) = -q_all_des(DIGIT_Q_COORD::qleftKnee);
    q_all_des(DIGIT_Q_COORD::qrightShinToTarsus) = -q_all_des(DIGIT_Q_COORD::qrightKnee);

    /* IK Loop */
    int iter_ik = 1;
    double indices[6] = {
        DIGIT_Q_COORD::qleftHipRoll,
        DIGIT_Q_COORD::qleftHipPitch,
        DIGIT_Q_COORD::qleftKnee,
        DIGIT_Q_COORD::qrightHipRoll,
        DIGIT_Q_COORD::qrightHipPitch,
        DIGIT_Q_COORD::qrightKnee};

    Matrix<double, 6, 1> indices_limit_min(degToRad(-60), degToRad(-60), degToRad(-71), degToRad(-60), degToRad(-90), degToRad(-50));
    Matrix<double, 6, 1> indices_limit_max(degToRad(60), degToRad(90), degToRad(50), degToRad(60), degToRad(60), degToRad(71));

    Matrix<double, 6, 1> y_vc_stand_ik;
    Matrix<double, 6, 1> y_vc_stand_error;

    Matrix<double, 6, 1> q_all_delta;

    Matrix<double, 6, constants::NUM_Q_ALL> Jy_vc_stand;
    Matrix<double, 6, 6> Jy_vc_stand_square;
    ColPivHouseholderQR<Matrix<double, 6, 6>> Jy_vc_stand_square_QR;

    while (true)
    {
        // iteration check
        if (iter_ik > max_iter_ik_)
        {
            break;
        }

        // enforce rigid spring constraints
        q_all_des(DIGIT_Q_COORD::qleftShinToTarsus) = -q_all_des(DIGIT_Q_COORD::qleftKnee);
        q_all_des(DIGIT_Q_COORD::qrightShinToTarsus) = -q_all_des(DIGIT_Q_COORD::qrightKnee);

        // approximate geometric constraint to keep hip pitch rotation the same after knee extension/contraction
        // q_all_des(DIGIT_Q_COORD::qleftHipPitch) = q_all(DIGIT_Q_COORD::qleftHipPitch) + 0.5 * (q_all_des(DIGIT_Q_COORD::qleftKnee) - q_all(DIGIT_Q_COORD::qleftKnee));
        // q_all_des(DIGIT_Q_COORD::qrightHipPitch) = q_all(DIGIT_Q_COORD::qrightHipPitch) + 0.5 * (q_all_des(DIGIT_Q_COORD::qrightKnee) - q_all(DIGIT_Q_COORD::qrightKnee));

        // compute virtual constriant
        gen::kin::y_vc_stand(y_vc_stand_ik, q_all_des, heading_mat);

        // compute virtual constraint error
        y_vc_stand_error = y_vc_stand_ik - y_vc_stand_des;

        // check vc error against tolerance
        if (y_vc_stand_error.cwiseAbs().maxCoeff() < tol_ik_)
        {
            break;
        }

        // compute vc jacobians
        gen::kin::Jy_vc_stand(Jy_vc_stand.reshaped(6 * constants::NUM_Q_ALL, 1), q_all_des, heading_mat);

        // square jacobian
        Jy_vc_stand_square = Jy_vc_stand(all, indices);

        // rigid spring constraint
        Jy_vc_stand_square(all, {2}) = Jy_vc_stand_square(all, {2}) - Jy_vc_stand(all, {DIGIT_Q_COORD::qleftShinToTarsus});  // rigid left heelspring
        Jy_vc_stand_square(all, {5}) = Jy_vc_stand_square(all, {5}) - Jy_vc_stand(all, {DIGIT_Q_COORD::qrightShinToTarsus}); // rigid right heelspring

        // check conditioning of Jy_vc_stand_square
        Jy_vc_stand_square_QR = Jy_vc_stand_square.colPivHouseholderQr();
        Jy_vc_stand_square_QR.setThreshold(ik_threshold_);
        // std::cout << "threshold: " << Jy_vc_stand_square_QR.threshold() << "\n";
        // std::cout << "max_pivot: " << Jy_vc_stand_square_QR.maxPivot() << "\n";
        // std::cout << "R: \n" << Jy_vc_stand_square_QR.matrixR() << "\n";
        if (Jy_vc_stand_square_QR.rank() < 6)
        {
            std::cout << "========================\nILL-CONDITIONED JACOBIAN \n========================\n\n";
            std::cout << "Rank: " << Jy_vc_stand_square_QR.rank() << "\n\n";
            break;
        }

        // compute delta q
        q_all_delta = Jy_vc_stand_square_QR.solve(y_vc_stand_error);
        // Matrix<double, 6, 1> q_all_delta_unscaled = q_all_delta;
        // Matrix<double, 6, 1> check = Jy_vc_stand_square * q_all_delta;

        // limit delta q size
        double max_q_all_delta = q_all_delta.cwiseAbs().maxCoeff();
        if (max_q_all_delta > scale_delta_ik_)
        {
            q_all_delta = scale_delta_ik_ * (q_all_delta / max_q_all_delta);
        }

        // Add delta q to q_all_des at specified indices
        q_all_des(indices) -= q_all_delta;

        // clamp joint limits
        q_all_des(DIGIT_Q_COORD::qleftHipRoll) = std::max(std::min(q_all_des(DIGIT_Q_COORD::qleftHipRoll), indices_limit_max[0]), indices_limit_min[0]);
        q_all_des(DIGIT_Q_COORD::qleftHipPitch) = std::max(std::min(q_all_des(DIGIT_Q_COORD::qleftHipPitch), indices_limit_max[1]), indices_limit_min[1]);
        q_all_des(DIGIT_Q_COORD::qleftKnee) = std::max(std::min(q_all_des(DIGIT_Q_COORD::qleftKnee), indices_limit_max[2]), indices_limit_min[2]);
        q_all_des(DIGIT_Q_COORD::qrightHipRoll) = std::max(std::min(q_all_des(DIGIT_Q_COORD::qrightHipRoll), indices_limit_max[3]), indices_limit_min[3]);
        q_all_des(DIGIT_Q_COORD::qrightHipPitch) = std::max(std::min(q_all_des(DIGIT_Q_COORD::qrightHipPitch), indices_limit_max[4]), indices_limit_min[4]);
        q_all_des(DIGIT_Q_COORD::qrightKnee) = std::max(std::min(q_all_des(DIGIT_Q_COORD::qrightKnee), indices_limit_max[5]), indices_limit_min[5]);

        // increase iteration counter
        iter_ik++;
    }

    /* qdot_all_des with final jacobian if nonzero velocity */
    Matrix<double, constants::NUM_Q_ALL, 1> qdot_all_des;
    qdot_all_des.setZero();

    /* Desired Motor Positions */
    Matrix<double, NUM_MOTORS, 1> q_motors_des = q_motors;

    q_motors_des(DigitMotors::LeftHipRoll) = q_all_des(DIGIT_Q_COORD::qleftHipRoll);
    q_motors_des(DigitMotors::LeftHipPitch) = q_all_des(DIGIT_Q_COORD::qleftHipPitch);
    q_motors_des(DigitMotors::LeftKnee) = q_all_des(DIGIT_Q_COORD::qleftKnee);
    q_motors_des(DigitMotors::RightHipRoll) = q_all_des(DIGIT_Q_COORD::qrightHipRoll);
    q_motors_des(DigitMotors::RightHipPitch) = q_all_des(DIGIT_Q_COORD::qrightHipPitch);
    q_motors_des(DigitMotors::RightKnee) = q_all_des(DIGIT_Q_COORD::qrightKnee);

    q_motors_des(DigitMotors::LeftHipYaw) = yaw_offset_filtered_;
    q_motors_des(DigitMotors::RightHipYaw) = yaw_offset_filtered_;

    // q_motors_des(DigitMotors::LeftShoulderRoll) = q_all_des(DIGIT_Q_COORD::qleftShoulderRoll);
    // q_motors_des(DigitMotors::LeftShoulderPitch) = q_all_des(DIGIT_Q_COORD::qleftShoulderPitch);
    // q_motors_des(DigitMotors::LeftShoulderYaw) = q_all_des(DIGIT_Q_COORD::qleftShoulderYaw);
    // q_motors_des(DigitMotors::LeftElbow) = q_all_des(DIGIT_Q_COORD::qleftElbow);
    // q_motors_des(DigitMotors::RightShoulderRoll) = q_all_des(DIGIT_Q_COORD::qrightShoulderRoll);
    // q_motors_des(DigitMotors::RightShoulderPitch) = q_all_des(DIGIT_Q_COORD::qrightShoulderPitch);
    // q_motors_des(DigitMotors::RightShoulderYaw) = q_all_des(DIGIT_Q_COORD::qrightShoulderYaw);
    // q_motors_des(DigitMotors::RightElbow) = q_all_des(DIGIT_Q_COORD::qrightElbow);

    q_motors_des(DigitMotors::LeftShoulderRoll) = -0.15;
    q_motors_des(DigitMotors::LeftShoulderPitch) = 1.1;
    q_motors_des(DigitMotors::LeftShoulderYaw) = 0;
    q_motors_des(DigitMotors::LeftElbow) = -0.145;
    q_motors_des(DigitMotors::RightShoulderRoll) = 0.15;
    q_motors_des(DigitMotors::RightShoulderPitch) = -1.1;
    q_motors_des(DigitMotors::RightShoulderYaw) = 0;
    q_motors_des(DigitMotors::RightElbow) = 0.145;

    Vector3d com_midpoint_error = p_com_aligned - 0.50 * (p_left_toe_aligned + p_right_toe_aligned);
    double toe_pitch_error = com_midpoint_error[0];
    q_motors_des(DigitMotors::LeftToeA) = q_motors(DigitMotors::LeftToeA) + toe_pitch_error;
    q_motors_des(DigitMotors::LeftToeB) = q_motors(DigitMotors::LeftToeB) - toe_pitch_error;
    q_motors_des(DigitMotors::RightToeA) = q_motors(DigitMotors::RightToeA) - toe_pitch_error;
    q_motors_des(DigitMotors::RightToeB) = q_motors(DigitMotors::RightToeB) + toe_pitch_error;

    /* Desired Motor Velocities */
    Matrix<double, NUM_MOTORS, 1> qdot_motors_des;
    qdot_motors_des.setZero();

    /* Joint Error */
    Matrix<double, NUM_MOTORS, 1> q_motors_error = q_motors - q_motors_des;

    /* PD Torque Control */
    Matrix<double, NUM_MOTORS, 1> motor_torque_command;
    motor_torque_command.setZero();

    // legs
    motor_torque_command(DigitMotors::LeftHipRoll) = -kp_hiproll_stand_tuned_ * q_motors_error(DigitMotors::LeftHipRoll);
    motor_torque_command(DigitMotors::LeftHipYaw) = -kp_hipyaw_stand_tuned_ * q_motors_error(DigitMotors::LeftHipYaw);
    motor_torque_command(DigitMotors::LeftHipPitch) = -kp_hippitch_stand_tuned_ * q_motors_error(DigitMotors::LeftHipPitch);
    motor_torque_command(DigitMotors::LeftKnee) = -kp_knee_stand_tuned_ * q_motors_error(DigitMotors::LeftKnee);
    motor_torque_command(DigitMotors::RightHipRoll) = -kp_hiproll_stand_tuned_ * q_motors_error(DigitMotors::RightHipRoll);
    motor_torque_command(DigitMotors::RightHipYaw) = -kp_hipyaw_stand_tuned_ * q_motors_error(DigitMotors::RightHipYaw);
    motor_torque_command(DigitMotors::RightHipPitch) = -kp_hippitch_stand_tuned_ * q_motors_error(DigitMotors::RightHipPitch);
    motor_torque_command(DigitMotors::RightKnee) = -kp_knee_stand_tuned_ * q_motors_error(DigitMotors::RightKnee);

    // arms
    motor_torque_command(DigitMotors::LeftShoulderRoll) = -kp_shoulderroll_stand_tuned_ * q_motors_error(DigitMotors::LeftShoulderRoll);
    motor_torque_command(DigitMotors::LeftShoulderPitch) = -kp_shoulderpitch_stand_tuned_ * q_motors_error(DigitMotors::LeftShoulderPitch);
    motor_torque_command(DigitMotors::LeftShoulderYaw) = -kp_shoulderyaw_stand_tuned_ * q_motors_error(DigitMotors::LeftShoulderYaw);
    motor_torque_command(DigitMotors::LeftElbow) = -kp_elbow_stand_tuned_ * q_motors_error(DigitMotors::LeftElbow);
    motor_torque_command(DigitMotors::RightShoulderRoll) = -kp_shoulderroll_stand_tuned_ * q_motors_error(DigitMotors::RightShoulderRoll);
    motor_torque_command(DigitMotors::RightShoulderPitch) = -kp_shoulderpitch_stand_tuned_ * q_motors_error(DigitMotors::RightShoulderPitch);
    motor_torque_command(DigitMotors::RightShoulderYaw) = -kp_shoulderyaw_stand_tuned_ * q_motors_error(DigitMotors::RightShoulderYaw);
    motor_torque_command(DigitMotors::RightElbow) = -kp_elbow_stand_tuned_ * q_motors_error(DigitMotors::RightElbow);

    // toes
    motor_torque_command(DigitMotors::LeftToeA) = -kp_toe_stand_tuned_ * q_motors_error(DigitMotors::LeftToeA);
    motor_torque_command(DigitMotors::LeftToeB) = -kp_toe_stand_tuned_ * q_motors_error(DigitMotors::LeftToeB);
    motor_torque_command(DigitMotors::RightToeA) = -kp_toe_stand_tuned_ * q_motors_error(DigitMotors::RightToeA);
    motor_torque_command(DigitMotors::RightToeB) = -kp_toe_stand_tuned_ * q_motors_error(DigitMotors::RightToeB);

    // Knee Compensation
    double knee_comp = -kp_knee_comp_stand_tuned_ * -com_midpoint_error[1] - kd_knee_comp_stand_tuned_ * -v_com_aligned[1];
    motor_torque_command(DigitMotors::LeftKnee) += knee_comp;
    motor_torque_command(DigitMotors::RightKnee) += knee_comp;

    /* Modify torque if using feedforward torque alone */
    if (torque_only_)
    {
        for (int i = 0; i < constants::NUM_ACTUATOR_JOINTS; ++i)
        {
            motor_torque_command[i] += - 1*cmd.motors[i].damping * (qdot_motors[i] - 0.0);
        }
    }

    /* Saturate torque if necessary */
    Digit_Controller::Saturate_Torque_Command_(motor_torque_command, lim);

    /* Command Update */
    for (int i = 0; i < constants::NUM_ACTUATOR_JOINTS; ++i)
    {
        cmd.motors[i].torque = motor_torque_command[i];

        if (torque_only_)
        {
            cmd.motors[i].velocity = 0.0;
            cmd.motors[i].damping = 0.0;
        }
        else
        {
            cmd.motors[i].velocity = 0.0;
            cmd.motors[i].damping = 1.0 * lim->damping_limit[i];
        }
    }
    cmd.fallback_opmode = Locomotion; // Damping, Disabled, Locomotion
    cmd.apply_command = true;

    /* Log Data */
    Matrix<double, 2, 1> LL, LL_des;
    LL << LL_left_mat, LL_right_mat;
    LL_des << 0, 0;
    // Digit_Controller::Standing_Numierc_Log_Data_(time_system,
    //                                              q_all,
    //                                              qdot_all,
    //                                              q_all_des,
    //                                              qdot_all_des,
    //                                              q_motors,
    //                                              qdot_motors,
    //                                              q_motors_des,
    //                                              qdot_motors_des,
    //                                              motor_torque_command,
    //                                              y_vc_stand,
    //                                              y_vc_stand_des,
    //                                              LL,
    //                                              LL_des,
    //                                              q_all_delta,
    //                                              p_com_world,
    //                                              p_left_toe_world,
    //                                              p_right_toe_world);
    return;
}
void Digit_Controller::Standing_Compute_Arm_IK_(Ref<Matrix<double, constants::NUM_Q_ALL, 1>> q_all_arm_des,
                                                Ref<Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all_arm_des,
                                                const Ref<const Matrix<double, 1, 1>> st_heading,
                                                const Ref<const VectorXd> y_vc_arm_des,
                                                const Ref<const VectorXd> ydot_vc_arm_des)
{
    int iter_ik = 1;
    double indices[8] = {
        DIGIT_Q_COORD::qleftShoulderRoll,
        DIGIT_Q_COORD::qleftShoulderPitch,
        DIGIT_Q_COORD::qleftShoulderYaw,
        DIGIT_Q_COORD::qleftElbow,
        DIGIT_Q_COORD::qrightShoulderRoll,
        DIGIT_Q_COORD::qrightShoulderPitch,
        DIGIT_Q_COORD::qrightShoulderYaw,
        DIGIT_Q_COORD::qrightElbow};

    Matrix<double, 8, 1> indices_limit_max(degToRad(75), degToRad(145), degToRad(100), degToRad(77.5), degToRad(75), degToRad(145), degToRad(100), degToRad(77.5));
    Matrix<double, 8, 1> indices_limit_min = -indices_limit_max;

    Vector3d left_arm_wrt_base_aligned, right_arm_wrt_base_aligned;

    Matrix<double, 6, 1> y_vc_arm_ik;
    Matrix<double, 6, 1> y_vc_arm_error;

    Matrix<double, 8, 1> q_all_arm_delta;

    Matrix<double, 3, constants::NUM_Q_ALL> Jp_left_hand_aligned;
    Matrix<double, 3, constants::NUM_Q_ALL> Jp_right_hand_aligned;
    Matrix<double, 6, constants::NUM_Q_ALL> Jy_vc_arm;
    Matrix<double, 6, 8> Jy_vc_arm_red;

    ColPivHouseholderQR<Matrix<double, 6, 8>> Jy_vc_arm_red_QR;

    while (true)
    {
        // iteration check
        if (iter_ik > max_iter_ik_)
        {
            break;
        }

        // compute vc

        gen::kin::p_left_hand_wrt_base(left_arm_wrt_base_aligned, q_all_arm_des, st_heading);
        gen::kin::p_right_hand_wrt_base(right_arm_wrt_base_aligned, q_all_arm_des, st_heading);

        // std::cout << "left: \n" << left_arm_wrt_base_aligned << "\n\n";
        // std::cout << "right: \n" << right_arm_wrt_base_aligned << "\n\n";

        Matrix<double, 6, 1> y_vc_arm_ik;
        y_vc_arm_ik << left_arm_wrt_base_aligned, right_arm_wrt_base_aligned;

        // compute virtual constraint error
        y_vc_arm_error = y_vc_arm_ik - y_vc_arm_des;

        // std::cout << "y_vc_arm_ik: \n" << y_vc_arm_ik;
        // std::cout << "\ny_vc_arm_error: \n" << y_vc_arm_error;

        // check vc error against tolerance
        if (y_vc_arm_error.cwiseAbs().maxCoeff() < tol_ik_ * 10)
        {
            break;
        }

        // compute J_vc
        gen::kin::Jp_left_hand_wrt_base(Jp_left_hand_aligned.reshaped(3 * constants::NUM_Q_ALL, 1), q_all_arm_des, st_heading);
        gen::kin::Jp_right_hand_wrt_base(Jp_right_hand_aligned.reshaped(3 * constants::NUM_Q_ALL, 1), q_all_arm_des, st_heading);

        Jy_vc_arm << Jp_left_hand_aligned, Jp_right_hand_aligned;

        // reduced J_vc_arm
        Jy_vc_arm_red = Jy_vc_arm(all, indices);

        // check conditioning of Jy_vc_stand_square
        Jy_vc_arm_red_QR = Jy_vc_arm_red.colPivHouseholderQr();
        Jy_vc_arm_red_QR.setThreshold(ik_threshold_);
        // std::cout << "threshold: " << Jy_vc_arm_red_QR.threshold() << "\n";
        // std::cout << "max_pivot: " << Jy_vc_arm_red_QR.maxPivot() << "\n";
        // std::cout << "R: \n" << Jy_vc_arm_red_QR.matrixR() << "\n";
        if (Jy_vc_arm_red_QR.rank() < 6)
        {
            std::cout << "========================\nILL-CONDITIONED ARM JACOBIAN \n========================\n\n";
            std::cout << "Rank: " << Jy_vc_arm_red_QR.rank() << "\n\n";
            break;
        }

        // compute delta q
        q_all_arm_delta = Jy_vc_arm_red_QR.solve(y_vc_arm_error);
        // Matrix<double, 6, 1> check = Jy_vc_arm_red * q_all_arm_delta;
        // std::cout << "y_vc_arm_error: \n" << y_vc_arm_error << "\n";
        // std::cout << "check: \n" << check << "\n";

        // limit delta q size
        double max_q_all_arm_delta = q_all_arm_delta.cwiseAbs().maxCoeff();
        if (max_q_all_arm_delta > scale_delta_ik_ * 2)
        {
            q_all_arm_delta = scale_delta_ik_ * 2 * (q_all_arm_delta / max_q_all_arm_delta);
        }

        // add delta
        q_all_arm_des(indices) -= q_all_arm_delta;

        // clamp
        q_all_arm_des(DIGIT_Q_COORD::qleftShoulderRoll) = std::max(std::min(q_all_arm_des(DIGIT_Q_COORD::qleftShoulderRoll), indices_limit_max[0]), indices_limit_min[0]);
        q_all_arm_des(DIGIT_Q_COORD::qleftShoulderPitch) = std::max(std::min(q_all_arm_des(DIGIT_Q_COORD::qleftShoulderPitch), indices_limit_max[1]), indices_limit_min[1]);
        q_all_arm_des(DIGIT_Q_COORD::qleftShoulderYaw) = std::max(std::min(q_all_arm_des(DIGIT_Q_COORD::qleftShoulderYaw), indices_limit_max[2]), indices_limit_min[2]);
        q_all_arm_des(DIGIT_Q_COORD::qleftElbow) = std::max(std::min(q_all_arm_des(DIGIT_Q_COORD::qleftElbow), indices_limit_max[3]), indices_limit_min[3]);

        q_all_arm_des(DIGIT_Q_COORD::qrightShoulderRoll) = std::max(std::min(q_all_arm_des(DIGIT_Q_COORD::qrightShoulderRoll), indices_limit_max[4]), indices_limit_min[4]);
        q_all_arm_des(DIGIT_Q_COORD::qrightShoulderPitch) = std::max(std::min(q_all_arm_des(DIGIT_Q_COORD::qrightShoulderPitch), indices_limit_max[5]), indices_limit_min[5]);
        q_all_arm_des(DIGIT_Q_COORD::qrightShoulderYaw) = std::max(std::min(q_all_arm_des(DIGIT_Q_COORD::qrightShoulderYaw), indices_limit_max[6]), indices_limit_min[6]);
        q_all_arm_des(DIGIT_Q_COORD::qrightElbow) = std::max(std::min(q_all_arm_des(DIGIT_Q_COORD::qrightElbow), indices_limit_max[7]), indices_limit_min[7]);

        // increase iteration
        iter_ik++;
    }
    std::cout << "\niter_ik: " << iter_ik << "\n";
    return;
}
void Digit_Controller::Standing_Sway_(const double &time_digit,
                                      const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                      double &x_offset_des,
                                      double &y_offset_des,
                                      double &z_offset_des,
                                      double &yaw_offset_des,
                                      double &pitch_offset_des,
                                      double &roll_offset_des)
{
    /* Do not add sway to these offsets */
    x_offset_des = x_offset_tuned_;
    y_offset_des = y_offset_tuned_;
    z_offset_des = 0;
    yaw_offset_des = 0;
    pitch_offset_des = 0;
    roll_offset_des = 0;

    /* sway yaw */
    if (enable_sway_yaw_)
    {
        /* Update at start of trajectory */
        if (sway_yaw_start_)
        {
            time_sway_yaw_start_ = time_digit;
            sway_yaw_init_ = q_all(DIGIT_Q_COORD::qbase_yaw) - yaw_init_; // must take into account initial yaw orientation in world frame
            sway_yaw_final_ = sway_yaw_dir_ * sway_yaw_max_;
            sway_yaw_start_ = false;
        }

        /* Compute phase variable s -> [0,1] */
        double s_sway_yaw = (time_digit - time_sway_yaw_start_) / time_sway_yaw_period_;

        /* Compute bezier coefficients (could probably be moved inside of the start conditional above)*/
        int bezier_degree = 6;
        const int bezier_degree_plus1 = 7;
        Matrix<double, bezier_degree_plus1, 1> alpha_sway_yaw;
        bezier_utils::Smooth_Bezier_Coefficients(sway_yaw_init_, sway_yaw_final_, bezier_degree, alpha_sway_yaw);

        /* Compute desired offset */
        yaw_offset_des = bezier_utils::Bezier(s_sway_yaw, alpha_sway_yaw);

        /* Check if trajectory has ended. If so update with new trajectory */
        if (s_sway_yaw >= 1)
        {
            sway_yaw_dir_ *= -1;
            sway_yaw_start_ = true;
        }
    }
    /* sway pitch */
    if (enable_sway_pitch_)
    {
        /* Update at start of trajectory */
        if (sway_pitch_start_)
        {
            time_sway_pitch_start_ = time_digit;
            sway_pitch_init_ = q_all(DIGIT_Q_COORD::qbase_pitch);
            sway_pitch_final_ = sway_pitch_dir_ * sway_pitch_max_;
            sway_pitch_start_ = false;
        }

        /* Compute phase variable s -> [0,1] */
        double s_sway_pitch = (time_digit - time_sway_pitch_start_) / time_sway_pitch_period_;

        /* Compute bezier coefficients (could probably be moved inside of the start conditional above)*/
        int bezier_degree = 6;
        const int bezier_degree_plus1 = 7;
        Matrix<double, bezier_degree_plus1, 1> alpha_sway_pitch;
        bezier_utils::Smooth_Bezier_Coefficients(sway_pitch_init_, sway_pitch_final_, bezier_degree, alpha_sway_pitch);

        /* Compute desired offset */
        pitch_offset_des = bezier_utils::Bezier(s_sway_pitch, alpha_sway_pitch);

        /* Check if trajectory has ended. If so update with new trajectory */
        if (s_sway_pitch >= 1)
        {
            sway_pitch_dir_ *= -1;
            sway_pitch_start_ = true;
        }
    }

    /* sway roll */
    if (enable_sway_roll_)
    {
        /* Update at start of trajectory */
        if (sway_roll_start_)
        {
            time_sway_roll_start_ = time_digit;
            sway_roll_init_ = q_all(DIGIT_Q_COORD::qbase_roll);
            sway_roll_final_ = sway_roll_dir_ * sway_roll_max_;
            sway_roll_start_ = false;
        }

        /* Compute phase variable s -> [0,1] */
        double s_sway_roll = (time_digit - time_sway_roll_start_) / time_sway_roll_period_;

        /* Compute bezier coefficients (could probably be moved inside of the start conditional above)*/
        int bezier_degree = 6;
        const int bezier_degree_plus1 = 7;
        Matrix<double, bezier_degree_plus1, 1> alpha_sway_roll;
        bezier_utils::Smooth_Bezier_Coefficients(sway_roll_init_, sway_roll_final_, bezier_degree, alpha_sway_roll);

        /* Compute desired offset */
        roll_offset_des = bezier_utils::Bezier(s_sway_roll, alpha_sway_roll);

        /* Check if trajectory has ended. If so update with new trajectory */
        if (s_sway_roll >= 1)
        {
            sway_roll_dir_ *= -1;
            sway_roll_start_ = true;
        }
    }

    /* sway crouch */ // use sway_crouch_max and sway_crouch_min instead of using sway_dir
    if (enable_sway_crouch_)
    {
        // std::cout << "Entered enable crouch function \n\n";
        /* Update at start of trajectory */
        if (sway_crouch_start_)
        {
            // std::cout << "Starting if statmeent \n\n";

            time_sway_crouch_start_ = time_digit;
            sway_crouch_init_ = q_all(DIGIT_Q_COORD::qbase_pos_z) - standing_com_height_des_;
            if (sway_crouch_dir_ == 1)
            {
                sway_crouch_final_ = sway_crouch_max_;
            }
            else
            {
                sway_crouch_final_ = sway_crouch_min_;
            }

            sway_crouch_start_ = false;
        }

        /* Compute phase variable s -> [0,1] */
        double s_sway_crouch = (time_digit - time_sway_crouch_start_) / time_sway_crouch_period_;
        std::cout << "sway_crouch_dir: " << sway_crouch_dir_ << "\n";
        std::cout << "s_crouch: " << s_sway_crouch << "\n";
        std::cout << "sway_crouch_init: " << sway_crouch_init_ << "\n";

        /* Compute bezier coefficients (could probably be moved inside of the start conditional above)*/
        int bezier_degree = 6;
        const int bezier_degree_plus1 = 7;
        Matrix<double, bezier_degree_plus1, 1> alpha_sway_crouch;
        bezier_utils::Smooth_Bezier_Coefficients(sway_crouch_init_, sway_crouch_final_, bezier_degree, alpha_sway_crouch);
        // std::cout << "Calculated bezier coefficients \n\n";
        std::cout << "alpha_sway_crouch: \n"
                  << alpha_sway_crouch << "\n";

        /* Compute desired offset */
        z_offset_des = bezier_utils::Bezier(s_sway_crouch, alpha_sway_crouch);
        std::cout << "z_offset: " << z_offset_des << "\n\n";
        /* Check if trajectory has ended. If so update with new trajectory */
        if (s_sway_crouch >= 1)
        {
            sway_crouch_dir_ *= -1;
            sway_crouch_start_ = true;
        }
    }

    return;
}
void Digit_Controller::Standing_Clamp_Targets_(double &x_offset_clamped,
                                               double &y_offset_clamped,
                                               double &z_offset_clamped,
                                               double &yaw_offset_clamped,
                                               double &pitch_offset_clamped,
                                               double &roll_offset_clamped,
                                               const double &x_offset_target,
                                               const double &y_offset_target,
                                               const double &z_offset_target,
                                               const double &yaw_offset_target,
                                               const double &pitch_offset_target,
                                               const double &roll_offset_target)
{
    x_offset_clamped = std::min(std::max(x_offset_target, x_offset_min_), x_offset_max_);
    y_offset_clamped = std::min(std::max(y_offset_target, y_offset_min_), y_offset_max_);
    z_offset_clamped = std::min(std::max(z_offset_target, z_offset_min_), z_offset_max_);
    yaw_offset_clamped = std::min(std::max(yaw_offset_target, yaw_offset_min_), yaw_offset_max_);
    pitch_offset_clamped = std::min(std::max(pitch_offset_target, pitch_offset_min_), pitch_offset_max_);
    roll_offset_clamped = std::min(std::max(roll_offset_target, roll_offset_min_), roll_offset_max_);

    return;
}
void Digit_Controller::Standing_Filter_Targets_(const double &x_offset_clamped,
                                                const double &y_offset_clamped,
                                                const double &z_offset_clamped,
                                                const double &yaw_offset_clamped,
                                                const double &pitch_offset_clamped,
                                                const double &roll_offset_clamped)
{
    double a = standing_target_filter_param_;
    x_offset_filtered_ = (1 - a) * x_offset_filtered_ + a * x_offset_clamped;
    y_offset_filtered_ = (1 - a) * y_offset_filtered_ + a * y_offset_clamped;
    z_offset_filtered_ = (1 - a) * z_offset_filtered_ + a * z_offset_clamped;
    yaw_offset_filtered_ = (1 - a) * yaw_offset_filtered_ + a * yaw_offset_clamped;
    pitch_offset_filtered_ = (1 - a) * pitch_offset_filtered_ + a * pitch_offset_clamped;
    roll_offset_filtered_ = (1 - a) * roll_offset_filtered_ + a * roll_offset_clamped;
}
void Digit_Controller::Standing_Numeric_Log_Data_(const double &time_system,
                                                  const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                                  const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                                                  const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_iter_stand,
                                                  const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_iter_stand,
                                                  const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors_act,
                                                  const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors_act,
                                                  const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors_des,
                                                  const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors_des,
                                                  const Ref<const Matrix<double, NUM_MOTORS, 1>> motor_torque_command,
                                                  const Ref<const Matrix<double, 6, 1>> y_vc_stand,
                                                  const Ref<const Matrix<double, 6, 1>> y_vc_stand_des,
                                                  const Ref<const Matrix<double, 2, 1>> LL,
                                                  const Ref<const Matrix<double, 2, 1>> LL_des,
                                                  const Ref<const Matrix<double, 6, 1>> delta_q_square,
                                                  const Ref<const Vector3d> p_com,
                                                  const Ref<const Vector3d> p_left_toe,
                                                  const Ref<const Vector3d> p_right_toe)
{
    ctrl_logfile_.open(path_to_log_, std::ofstream::out | std::ofstream::app);

    /* Log order
            time_system,
            q_all,
            qdot_all,
            q_iter_stand,
            qdot_iter_stand,
            q_motors_act,
            qdot_motors_act,
            q_motors_des,
            qdot_motors_des,
            motor_torque_command,
            y_vc_stand,
            y_vc_stand_des,
            LL,
            LL_des,
            delta_q,
            p_com,
            p_left_toe,
            p_right_toe,

    */
    // time_system
    ctrl_logfile_ << std::fixed << time_system << ",\n";

    /* q_all */
    for (auto i : std::vector<double>(q_all.data(), q_all.data() + q_all.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_all */
    for (auto i : std::vector<double>(qdot_all.data(), qdot_all.data() + qdot_all.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* q_iter_stand */
    for (auto i : std::vector<double>(q_iter_stand.data(), q_iter_stand.data() + q_iter_stand.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_iter_stand */
    for (auto i : std::vector<double>(qdot_iter_stand.data(), qdot_iter_stand.data() + qdot_iter_stand.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* q_motors_act */
    for (auto i : std::vector<double>(q_motors_act.data(), q_motors_act.data() + q_motors_act.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_motors_act */
    for (auto i : std::vector<double>(qdot_motors_act.data(), qdot_motors_act.data() + qdot_motors_act.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* q_motors_des */
    for (auto i : std::vector<double>(q_motors_des.data(), q_motors_des.data() + q_motors_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_motors_des */
    for (auto i : std::vector<double>(qdot_motors_des.data(), qdot_motors_des.data() + qdot_motors_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* motor_torque_command */
    for (auto i : std::vector<double>(motor_torque_command.data(), motor_torque_command.data() + motor_torque_command.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* y_vc_stand */
    for (auto i : std::vector<double>(y_vc_stand.data(), y_vc_stand.data() + y_vc_stand.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* y_vc_stand_des */
    for (auto i : std::vector<double>(y_vc_stand_des.data(), y_vc_stand_des.data() + y_vc_stand_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* LL */
    for (auto i : std::vector<double>(LL.data(), LL.data() + LL.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* LL_des */
    for (auto i : std::vector<double>(LL_des.data(), LL_des.data() + LL_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* delta_q */
    for (auto i : std::vector<double>(delta_q_square.data(), delta_q_square.data() + delta_q_square.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* p_com */
    for (auto i : std::vector<double>(p_com.data(), p_com.data() + p_com.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* p_left_toe */
    for (auto i : std::vector<double>(p_left_toe.data(), p_left_toe.data() + p_left_toe.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* p_right_toe */
    for (auto i : std::vector<double>(p_right_toe.data(), p_right_toe.data() + p_right_toe.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    ctrl_logfile_.close();

    return;
}

/***************************  Walking ALIP Controller (1Step or MPC FP) Functions ***************************/
void Digit_Controller::Walking_Update_(llapi_command_t &cmd,
                                       const double &time_system,
                                       const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                       const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                                       const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors,
                                       const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors,
                                       const Ref<const Matrix<double, NUM_UNACT_JOINTS, 1>> q_joints,
                                       const llapi_limits_t *const lim)
{
    /* Initialize controller */
    if (flag_walking_first_iter_)
    {
        Digit_Controller::Walking_Initialize_(time_system, q_all, qdot_all);
        flag_walking_first_iter_ = false;
    }

    /* Check for Impact with GRF or Time lapse since prev impact */
    Digit_Controller::Walking_Check_Stance_Switch_(time_system, q_joints);

    /* Update Only once at impact */
    if (stance_switch_)
    {
        Digit_Controller::Walking_Update_Variables_At_Impact_(time_system, q_all);
        stance_switch_ = false;
    }

    /* Update walking variables at each loop iteration */
    double kx_world, ky_world;
    double s_time, sdot_time;
    Vector3d p_st_toe_aligned, p_sw_toe_aligned;
    Vector3d p_com_wrt_st_aligned;
    Vector3d v_com_aligned;
    Vector3d L_st_obs_aligned, L_sw_obs_aligned;
    Vector4d y_vc_walk_flat, ydot_vc_walk_flat;
    Digit_Controller::Walking_Update_Variables_Each_Loop_(kx_world, ky_world, s_time, sdot_time, p_st_toe_aligned, p_sw_toe_aligned, p_com_wrt_st_aligned, v_com_aligned, L_st_obs_aligned, L_sw_obs_aligned, y_vc_walk_flat, ydot_vc_walk_flat,
                                                          time_system, q_all, qdot_all);

    // Kalman Filter Angular Momentum
    Digit_Controller::Walking_Kalman_Filter_Angular_Momentum_(p_com_wrt_st_aligned, L_st_obs_aligned); // First update Angular momentum estimate

    // Filter Target Variables
    Digit_Controller::Walking_Filter_Targets_();

    // Desired Virtual Constriants
    Vector4d y_virt_fp_des, ydot_virt_fp_des;
    Digit_Controller::Walking_Compute_1Step_Foot_Placement_(y_virt_fp_des, ydot_virt_fp_des,
                                                            s_time, sdot_time, kx_world, ky_world, p_com_wrt_st_aligned);

    /* Iterative Inverse Kinematics */
    Matrix<double, constants::NUM_Q_ALL, 1> q_all_des, qdot_all_des; // generalized coordinate results from ik (fp virtual contraints --> q virtual constraints )
    Digit_Controller::Walking_Compute_IK_(q_all_des, qdot_all_des,
                                          kx_world, ky_world, st_heading_bos_mat_, y_virt_fp_des, ydot_virt_fp_des, q_all, qdot_all);

    /* Compute Desired Motors */
    Matrix<double, NUM_MOTORS, 1> q_motors_des, qdot_motors_des;
    q_motors_des.setZero();
    qdot_motors_des.setZero();
    Digit_Controller::Walking_Compute_Desired_Motors_(q_motors_des, qdot_motors_des, q_all_des, qdot_all_des);

    q_motors_des_copy = q_motors_des;
    qdot_motors_des_copy = qdot_motors_des;
    q_all_des_copy = q_all_des;
    qdot_all_des_copy = qdot_all_des;

    /* Motor joint error */
    Matrix<double, NUM_MOTORS, 1> q_motors_error = q_motors - q_motors_des;

    /* PD Control */
    Matrix<double, NUM_MOTORS, 1> motor_torque_command;
    motor_torque_command.setZero();

    if (stance_leg_ == -1) // left stance
    {
        // torso orientation
        motor_torque_command(DigitMotors::LeftHipRoll) = -kp_hiproll_st_tuned_ * q_all(DIGIT_Q_COORD::qbase_roll);     //  torso roll
        motor_torque_command(DigitMotors::LeftHipPitch) = -kp_hippitch_st_tuned_ * -q_all(DIGIT_Q_COORD::qbase_pitch); // torso pitch

        // legs
        // motor_torque_command(DigitMotors::LeftHipRoll) = -kp_hiproll_st_tuned_ * q_motors_error(DigitMotors::LeftHipRoll);
        motor_torque_command(DigitMotors::LeftHipYaw) = -kp_hipyaw_st_tuned_ * q_motors_error(DigitMotors::LeftHipYaw);
        // motor_torque_command(DigitMotors::LeftHipPitch) = -kp_hippitch_st_tuned_ * q_motors_error(DigitMotors::LeftHipPitch);
        motor_torque_command(DigitMotors::LeftKnee) = -kp_knee_st_tuned_ * q_motors_error(DigitMotors::LeftKnee);
        motor_torque_command(DigitMotors::RightHipRoll) = -kp_hiproll_sw_tuned_ * q_motors_error(DigitMotors::RightHipRoll);
        motor_torque_command(DigitMotors::RightHipYaw) = -kp_hipyaw_sw_tuned_ * q_motors_error(DigitMotors::RightHipYaw);
        motor_torque_command(DigitMotors::RightHipPitch) = -kp_hippitch_sw_tuned_ * q_motors_error(DigitMotors::RightHipPitch);
        motor_torque_command(DigitMotors::RightKnee) = -kp_knee_sw_tuned_ * q_motors_error(DigitMotors::RightKnee);
    }
    else // right stance
    {
        // torso orientation
        motor_torque_command(DigitMotors::RightHipRoll) = -kp_hiproll_st_tuned_ * q_all(DIGIT_Q_COORD::qbase_roll);
        motor_torque_command(DigitMotors::RightHipPitch) = -kp_hippitch_st_tuned_ * q_all(DIGIT_Q_COORD::qbase_pitch);

        // legs
        motor_torque_command(DigitMotors::LeftHipRoll) = -kp_hiproll_sw_tuned_ * q_motors_error(DigitMotors::LeftHipRoll);
        motor_torque_command(DigitMotors::LeftHipYaw) = -kp_hipyaw_sw_tuned_ * q_motors_error(DigitMotors::LeftHipYaw);
        motor_torque_command(DigitMotors::LeftHipPitch) = -kp_hippitch_sw_tuned_ * q_motors_error(DigitMotors::LeftHipPitch);
        motor_torque_command(DigitMotors::LeftKnee) = -kp_knee_sw_tuned_ * q_motors_error(DigitMotors::LeftKnee);
        // motor_torque_command(DigitMotors::RightHipRoll) = -kp_hiproll_st_tuned_ * q_motors_error(DigitMotors::RightHipRoll);
        motor_torque_command(DigitMotors::RightHipYaw) = -kp_hipyaw_st_tuned_ * q_motors_error(DigitMotors::RightHipYaw);
        // motor_torque_command(DigitMotors::RightHipPitch) = -kp_hippitch_st_tuned_ * q_motors_error(DigitMotors::RightHipPitch);
        motor_torque_command(DigitMotors::RightKnee) = -kp_knee_st_tuned_ * q_motors_error(DigitMotors::RightKnee);
    }
    // arms
    motor_torque_command(DigitMotors::LeftShoulderRoll) = -kp_shoulderroll_tuned_ * q_motors_error(DigitMotors::LeftShoulderRoll);
    motor_torque_command(DigitMotors::LeftShoulderPitch) = -kp_shoulderpitch_tuned_ * q_motors_error(DigitMotors::LeftShoulderPitch);
    motor_torque_command(DigitMotors::LeftShoulderYaw) = -kp_shoulderyaw_tuned_ * q_motors_error(DigitMotors::LeftShoulderYaw);
    motor_torque_command(DigitMotors::LeftElbow) = -kp_elbow_tuned_ * q_motors_error(DigitMotors::LeftElbow);
    motor_torque_command(DigitMotors::RightShoulderRoll) = -kp_shoulderroll_tuned_ * q_motors_error(DigitMotors::RightShoulderRoll);
    motor_torque_command(DigitMotors::RightShoulderPitch) = -kp_shoulderpitch_tuned_ * q_motors_error(DigitMotors::RightShoulderPitch);
    motor_torque_command(DigitMotors::RightShoulderYaw) = -kp_shoulderyaw_tuned_ * q_motors_error(DigitMotors::RightShoulderYaw);
    motor_torque_command(DigitMotors::RightElbow) = -kp_elbow_tuned_ * q_motors_error(DigitMotors::RightElbow);

    // toes
    motor_torque_command(DigitMotors::LeftToeA) = -kp_toe_sw_tuned_ * q_motors_error(DigitMotors::LeftToeA);
    motor_torque_command(DigitMotors::LeftToeB) = -kp_toe_sw_tuned_ * q_motors_error(DigitMotors::LeftToeB);
    motor_torque_command(DigitMotors::RightToeA) = -kp_toe_sw_tuned_ * q_motors_error(DigitMotors::RightToeA);
    motor_torque_command(DigitMotors::RightToeB) = -kp_toe_sw_tuned_ * q_motors_error(DigitMotors::RightToeB);

    // Stance compensation
    if (stance_leg_ == -1)
    {
        motor_torque_command(DigitMotors::LeftKnee) += u_knee_comp_;
        motor_torque_command(DigitMotors::LeftHipRoll) -= u_hiproll_comp_;
    }
    else
    {
        motor_torque_command(DigitMotors::RightKnee) -= u_knee_comp_;
        motor_torque_command(DigitMotors::RightHipRoll) += u_hiproll_comp_;
    }

    // Ramp Torque (if needed)
    // Digit_Controller::Ramp_Torque_(0.05);

    // Saturate torque
    Digit_Controller::Saturate_Torque_Command_(motor_torque_command, lim);

    /* Command Update */
    for (int i = 0; i < constants::NUM_ACTUATOR_JOINTS; ++i)
    {
        cmd.motors[i].torque = motor_torque_command[i];
        cmd.motors[i].velocity = qdot_motors_des[i];
        cmd.motors[i].damping = 0.8 * lim->damping_limit[i];
    }
    if (stance_leg_ == -1)
    {
        cmd.motors[DigitMotors::LeftToeA].torque = 0;
        cmd.motors[DigitMotors::LeftToeB].torque = 0;
        cmd.motors[DigitMotors::LeftToeA].damping = 0;
        cmd.motors[DigitMotors::LeftToeB].damping = 0;
    }
    else
    {
        cmd.motors[DigitMotors::RightToeA].torque = 0;
        cmd.motors[DigitMotors::RightToeB].torque = 0;
        cmd.motors[DigitMotors::RightToeA].damping = 0;
        cmd.motors[DigitMotors::RightToeB].damping = 0;
    }
    cmd.fallback_opmode = Locomotion;
    cmd.apply_command = true;

    /* Log Walking Data */
    // std::cout << "stance leg (left == -1): " << stance_leg_ << "\n\n";
    // Digit_Controller::Walking_Log_Data_(time_system,
    //                                     s_time,
    //                                     sdot_time,
    //                                     kx_world,
    //                                     ky_world,
    //                                     st_heading_bos_,
    //                                     stance_leg_,
    //                                     q_all,
    //                                     qdot_all,
    //                                     q_all_des,
    //                                     qdot_all_des,
    //                                     q_motors,
    //                                     qdot_motors,
    //                                     q_motors_des,
    //                                     qdot_motors_des,
    //                                     motor_torque_command,
    //                                     y_vc_walk_flat,
    //                                     ydot_vc_walk_flat,
    //                                     y_virt_fp_des,
    //                                     ydot_virt_fp_des,
    //                                     p_st_toe_aligned,
    //                                     p_sw_toe_aligned,
    //                                     p_com_wrt_st_aligned,
    //                                     v_com_aligned,
    //                                     L_st_obs_aligned,
    //                                     L_sw_obs_aligned);

    return;
}
void Digit_Controller::Walking_Initialize_(const double &time_system,
                                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all)
{
    stance_switch_ = false;
    stance_leg_ = -1;

    /* Update time */
    time_step_start_ = time_system;

    /* Kalman Filter */
    Vector3d p_left_toe_world, p_right_toe_world;
    gen::kin::p_toe_pitch_joint_left(p_left_toe_world, q_all);
    gen::kin::p_toe_pitch_joint_right(p_right_toe_world, q_all);

    Vector3d L_left_toe_obs_world, L_right_toe_obs_world;
    gen::kin::L_world_about_point(L_left_toe_obs_world, q_all, qdot_all, p_left_toe_world);
    gen::kin::L_world_about_point(L_right_toe_obs_world, q_all, qdot_all, p_right_toe_world);

    /* Always start with left leg stance */
    assert(stance_leg_ == -1);

    /* Q Indices */
    idx_q_st_hiproll_ = DIGIT_Q_COORD::qleftHipRoll;
    idx_q_st_hipyaw_ = DIGIT_Q_COORD::qleftHipYaw;
    idx_q_st_hippitch_ = DIGIT_Q_COORD::qleftHipPitch;
    idx_q_st_knee_ = DIGIT_Q_COORD::qleftKnee;
    idx_q_st_KneeToShin_ = DIGIT_Q_COORD::qleftKneeToShin;
    idx_q_st_ShinToTarsus_ = DIGIT_Q_COORD::qleftShinToTarsus;

    idx_q_sw_hiproll_ = DIGIT_Q_COORD::qrightHipRoll;
    idx_q_sw_hipyaw_ = DIGIT_Q_COORD::qrightHipYaw;
    idx_q_sw_hippitch_ = DIGIT_Q_COORD::qrightHipPitch;
    idx_q_sw_knee_ = DIGIT_Q_COORD::qrightKnee;
    idx_q_sw_KneeToShin_ = DIGIT_Q_COORD::qrightKneeToShin;
    idx_q_sw_ShinToTarsus_ = DIGIT_Q_COORD::qrightShinToTarsus;

    /* Motor Indices */
    idx_m_st_hiproll_ = DigitMotors::LeftHipRoll;
    idx_m_st_hipyaw_ = DigitMotors::LeftHipYaw;
    idx_m_st_hippitch_ = DigitMotors::LeftHipPitch;
    idx_m_st_knee_ = DigitMotors::LeftKnee;

    idx_m_sw_hiproll_ = DigitMotors::RightHipRoll;
    idx_m_sw_hipyaw_ = DigitMotors::RightHipYaw;
    idx_m_sw_hippitch_ = DigitMotors::RightHipPitch;
    idx_m_sw_knee_ = DigitMotors::RightKnee;

    /* Target Heading */
    target_heading_ = q_all(DIGIT_Q_COORD::qbase_yaw);

    /* Stance Heading */
    transform_utils::Wrap_To_Pi(st_heading_bos_, q_all(DIGIT_Q_COORD::qbase_yaw) - q_all(DIGIT_Q_COORD::qleftHipYaw)); // stance heading
    st_heading_bos_mat_ << st_heading_bos_;
    transform_utils::Euler_Yaw_Angle_To_Rotation_Matrix(Rz_st_heading_bos_, st_heading_bos_); // update yaw rotation matrix

    /* relative position of swing toe w.r.t stance at beginning of step */
    p_sw_wrt_st_toe_bos_aligned_ = Rz_st_heading_bos_.transpose() * (p_right_toe_world - p_left_toe_world);

    /* relative position of com wrt st toe at beginning of step */
    Vector3d p_com_world;
    gen::kin::p_COM(p_com_world, q_all);
    p_com_wrt_st_bos_aligned_ = Rz_st_heading_bos_.transpose() * (p_com_world - p_left_toe_world);

    /* Kalman Filter variables for Angular Momentum */
    L_left_toe_obs_aligned_ = Rz_st_heading_bos_.transpose() * L_left_toe_obs_world;
    L_right_toe_obs_aligned_ = Rz_st_heading_bos_.transpose() * L_right_toe_obs_world;
    Lx_st_kf_aligned_ = L_left_toe_obs_aligned_[0];
    Ly_st_kf_aligned_ = L_left_toe_obs_aligned_[1];
    cov_Lx_st_kf_aligned_ = 0.25;
    cov_Ly_st_kf_aligned_ = 0.25;

    /* Motor Limits */
    q_motors_min_ << degToRad(-60), // left-hip-roll
        degToRad(-40),              // left-hip-yaw
        degToRad(-60),              // left-hip-pitch
        degToRad(-71),              // left-knee
        degToRad(-55.4),            // left-toe-A
        degToRad(-52.3),            // left-toe-B
        degToRad(-60),              // right-hip-roll
        degToRad(-40),              // right-hip-yaw
        degToRad(-90),              // right-hip-pitch
        degToRad(-50),              // right-knee
        degToRad(-55.4),            // right-toe-A
        degToRad(-52.3),            // right-toe-B
        degToRad(-75),              // left-shoulder-roll
        degToRad(-145),             // left-shoulder-pitch
        degToRad(-100),             // left-shoulder-yaw
        degToRad(-77.5),            // left-elbow
        degToRad(-75),              // right-shoulder-roll
        degToRad(-145),             // right-shoulder-pitch
        degToRad(-100),             // right-shoulder-yaw
        degToRad(-77.5);            // right-elbow
    q_motors_max_ << degToRad(60),  // left-hip-roll
        degToRad(40),               // left-hip-yaw
        degToRad(90),               // left-hip-pitch
        degToRad(50),               // left-knee
        degToRad(51.3),             // left-toe-A
        degToRad(72.45),            // left-toe-B
        degToRad(60),               // right-hip-roll
        degToRad(40),               // right-hip-yaw
        degToRad(60),               // right-hip-pitch
        degToRad(71),               // right-knee
        degToRad(51.3),             // right-toe-A
        degToRad(72.45),            // right-toe-B
        degToRad(75),               // left-shoulder-roll
        degToRad(145),              // left-shoulder-pitch
        degToRad(100),              // left-shoulder-yaw
        degToRad(77.5),             // left-elbow
        degToRad(75),               // right-shoulder-roll
        degToRad(145),              // right-shoulder-pitch
        degToRad(100),              // right-shoulder-yaw
        degToRad(77.5);             // right-elbow

    return;
}
void Digit_Controller::Walking_Check_Stance_Switch_(const double time_system,
                                                    const Ref<const Matrix<double, NUM_UNACT_JOINTS, 1>> q_joints)
{
    /* Check time since gait began */
    double s = (time_system - time_step_start_) / time_step_period_;
    // double s = 1.0;

    /* Measure heel spring deflection */
    double swing_heelspring_deflection;
    if (stance_leg_ == -1)
    {
        swing_heelspring_deflection = q_joints[DigitJoints::RightHeelSpring];
    }
    else
    {
        swing_heelspring_deflection = q_joints[DigitJoints::LeftHeelSpring];
    }

    /* Check if impact event has occurred */
    if (s > 1.01 || (std::abs(swing_heelspring_deflection) > contact_deflection_ && s > 0.5))
    {
        stance_switch_ = true;
    }
    return;
}
void Digit_Controller::Walking_Update_Variables_At_Impact_(const double &time_system,
                                                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all)
{
    /* Change Stance Leg */
    stance_leg_ = -1.0 * stance_leg_; // switch stance leg

    /* Update time */
    time_step_start_ = time_system;

    /* Update target heading */
    transform_utils::Wrap_To_Pi(target_heading_, target_heading_ + turn_rps_tuned_ * time_step_period_);

    /* Stance variable update */
    Vector3d p_left_toe_world, p_right_toe_world, p_com_world;
    if (stance_leg_ == -1) // -1: left stance
    {
        // Stance Heading
        transform_utils::Wrap_To_Pi(st_heading_bos_, q_all(DIGIT_Q_COORD::qbase_yaw) - q_all(DIGIT_Q_COORD::qleftHipYaw)); // wrapped avg yaw direction
        st_heading_bos_mat_ << st_heading_bos_;
        transform_utils::Euler_Yaw_Angle_To_Rotation_Matrix(Rz_st_heading_bos_, st_heading_bos_); // update yaw rotation matrix

        // relative position of swing toe w.r.t stance at beginning of step
        gen::kin::p_toe_pitch_joint_left(p_left_toe_world, q_all);
        gen::kin::p_toe_pitch_joint_right(p_right_toe_world, q_all);
        p_sw_wrt_st_toe_bos_aligned_ = Rz_st_heading_bos_.transpose() * (p_right_toe_world - p_left_toe_world);

        // relative position of com wrt st toe at beginning of step
        gen::kin::p_COM(p_com_world, q_all);
        p_com_wrt_st_bos_aligned_ = Rz_st_heading_bos_.transpose() * (p_com_world - p_left_toe_world);

        // Kalman Filter variables for Angular Momentum
        Lx_st_kf_aligned_ = L_left_toe_obs_aligned_[0];
        Ly_st_kf_aligned_ = L_left_toe_obs_aligned_[1];
        cov_Lx_st_kf_aligned_ = 0.25;
        cov_Ly_st_kf_aligned_ = 0.25;

        // Q Indices
        idx_q_st_hiproll_ = DIGIT_Q_COORD::qleftHipRoll;
        idx_q_st_hipyaw_ = DIGIT_Q_COORD::qleftHipYaw;
        idx_q_st_hippitch_ = DIGIT_Q_COORD::qleftHipPitch;
        idx_q_st_knee_ = DIGIT_Q_COORD::qleftKnee;
        idx_q_st_KneeToShin_ = DIGIT_Q_COORD::qleftKneeToShin;
        idx_q_st_ShinToTarsus_ = DIGIT_Q_COORD::qleftShinToTarsus;

        idx_q_sw_hiproll_ = DIGIT_Q_COORD::qrightHipRoll;
        idx_q_sw_hipyaw_ = DIGIT_Q_COORD::qrightHipYaw;
        idx_q_sw_hippitch_ = DIGIT_Q_COORD::qrightHipPitch;
        idx_q_sw_knee_ = DIGIT_Q_COORD::qrightKnee;
        idx_q_sw_KneeToShin_ = DIGIT_Q_COORD::qrightKneeToShin;
        idx_q_sw_ShinToTarsus_ = DIGIT_Q_COORD::qrightShinToTarsus;

        // Motor Indices
        idx_m_st_hiproll_ = DigitMotors::LeftHipRoll;
        idx_m_st_hipyaw_ = DigitMotors::LeftHipYaw;
        idx_m_st_hippitch_ = DigitMotors::LeftHipPitch;
        idx_m_st_knee_ = DigitMotors::LeftKnee;

        idx_m_sw_hiproll_ = DigitMotors::RightHipRoll;
        idx_m_sw_hipyaw_ = DigitMotors::RightHipYaw;
        idx_m_sw_hippitch_ = DigitMotors::RightHipPitch;
        idx_m_sw_knee_ = DigitMotors::RightKnee;
    }
    else // right stance
    {
        // Stance Heading
        transform_utils::Wrap_To_Pi(st_heading_bos_, q_all(DIGIT_Q_COORD::qbase_yaw) - q_all(DIGIT_Q_COORD::qrightHipYaw));
        st_heading_bos_mat_ << st_heading_bos_;
        transform_utils::Euler_Yaw_Angle_To_Rotation_Matrix(Rz_st_heading_bos_, st_heading_bos_); // update yaw rotation matrix

        // relative position of swing toe w.r.t stance at beginning of step
        gen::kin::p_toe_pitch_joint_left(p_left_toe_world, q_all);
        gen::kin::p_toe_pitch_joint_right(p_right_toe_world, q_all);
        p_sw_wrt_st_toe_bos_aligned_ = Rz_st_heading_bos_.transpose() * (p_left_toe_world - p_right_toe_world);

        // relative position of com wrt st toe at beginning of step
        gen::kin::p_COM(p_com_world, q_all);
        p_com_wrt_st_bos_aligned_ = Rz_st_heading_bos_.transpose() * (p_com_world - p_right_toe_world);

        // Kalman Filter variables for Angular Momentum
        Lx_st_kf_aligned_ = L_right_toe_obs_aligned_[0];
        Ly_st_kf_aligned_ = L_right_toe_obs_aligned_[1];
        cov_Lx_st_kf_aligned_ = 0.25;
        cov_Ly_st_kf_aligned_ = 0.25;

        // Q Indices
        idx_q_st_hiproll_ = DIGIT_Q_COORD::qrightHipRoll;
        idx_q_st_hipyaw_ = DIGIT_Q_COORD::qrightHipYaw;
        idx_q_st_hippitch_ = DIGIT_Q_COORD::qrightHipPitch;
        idx_q_st_knee_ = DIGIT_Q_COORD::qrightKnee;
        idx_q_st_KneeToShin_ = DIGIT_Q_COORD::qrightKneeToShin;
        idx_q_st_ShinToTarsus_ = DIGIT_Q_COORD::qrightShinToTarsus;

        idx_q_sw_hiproll_ = DIGIT_Q_COORD::qleftHipRoll;
        idx_q_sw_hipyaw_ = DIGIT_Q_COORD::qleftHipYaw;
        idx_q_sw_hippitch_ = DIGIT_Q_COORD::qleftHipPitch;
        idx_q_sw_knee_ = DIGIT_Q_COORD::qleftKnee;
        idx_q_sw_KneeToShin_ = DIGIT_Q_COORD::qleftKneeToShin;
        idx_q_sw_ShinToTarsus_ = DIGIT_Q_COORD::qleftShinToTarsus;

        // Motor Indices
        idx_m_st_hiproll_ = DigitMotors::RightHipRoll;
        idx_m_st_hipyaw_ = DigitMotors::RightHipYaw;
        idx_m_st_hippitch_ = DigitMotors::RightHipPitch;
        idx_m_st_knee_ = DigitMotors::RightKnee;

        idx_m_sw_hiproll_ = DigitMotors::LeftHipRoll;
        idx_m_sw_hipyaw_ = DigitMotors::LeftHipYaw;
        idx_m_sw_hippitch_ = DigitMotors::LeftHipPitch;
        idx_m_sw_knee_ = DigitMotors::LeftKnee;
    }
    return;
}
void Digit_Controller::Walking_Update_Variables_Each_Loop_(double &kx_world,
                                                           double &ky_world,
                                                           double &s_time,
                                                           double &sdot_time,
                                                           Ref<Vector3d> p_st_toe_aligned,
                                                           Ref<Vector3d> p_sw_toe_aligned,
                                                           Ref<Vector3d> p_com_wrt_st_aligned,
                                                           Ref<Vector3d> v_com_aligned,
                                                           Ref<Vector3d> L_st_obs_aligned,
                                                           Ref<Vector3d> L_sw_obs_aligned,
                                                           Ref<Vector4d> y_vc_walk_flat,
                                                           Ref<Vector4d> ydot_vc_walk_flat,
                                                           const double &time_system,
                                                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all)
{
    /* Terrain */
    Vector3d k_normal_body(kx_body_, ky_body_, -1);
    Vector3d k_normal_world = Rz_st_heading_bos_ * k_normal_body;
    // Vector3d k_normal_world = Rz_target_heading_bos_ * k_normal_body; // might need to normalize
    kx_world = k_normal_world(0);
    ky_world = k_normal_world(1);

    /* Update Phase Variable */
    // s_time = 1.0;
    s_time = (time_system - time_step_start_) / (time_step_period_);
    sdot_time = 1.0 / time_step_period_;

    /* Toes */
    Vector3d p_left_toe_world, p_right_toe_world;
    gen::kin::p_toe_pitch_joint_left(p_left_toe_world, q_all);
    gen::kin::p_toe_pitch_joint_right(p_right_toe_world, q_all);

    Vector3d p_left_toe_aligned = Rz_st_heading_bos_.transpose() * p_left_toe_world;
    Vector3d p_right_toe_aligned = Rz_st_heading_bos_.transpose() * p_right_toe_world;

    /* Center of Mass wrt stance */
    Vector3d p_com_world, v_com_world;
    gen::kin::p_COM(p_com_world, q_all);

    Vector3d p_com_aligned = Rz_st_heading_bos_.transpose() * p_com_world;
    Vector3d p_com_wrt_left_toe_aligned = Rz_st_heading_bos_.transpose() * (p_com_world - p_left_toe_world);
    Vector3d p_com_wrt_right_toe_aligned = Rz_st_heading_bos_.transpose() * (p_com_world - p_right_toe_world);

    /* Center of Mass velocity */
    gen::kin::v_COM(v_com_world, q_all, qdot_all);
    v_com_aligned = Rz_st_heading_bos_.transpose() * v_com_world;
    // std::cout << "v_com_world: \n"
    //           << v_com_world << "\n\n";
    // std::cout << "v_com_aligned: \n"
    //           << v_com_aligned << "\n\n";

    /* (Observed) Angular Momentum */
    Vector3d L_left_toe_obs_world, L_right_toe_obs_world;
    gen::kin::L_world_about_point(L_left_toe_obs_world, q_all, qdot_all, p_left_toe_world);
    gen::kin::L_world_about_point(L_right_toe_obs_world, q_all, qdot_all, p_right_toe_world);

    L_left_toe_obs_aligned_ = Rz_st_heading_bos_.transpose() * L_left_toe_obs_world;
    L_right_toe_obs_aligned_ = Rz_st_heading_bos_.transpose() * L_right_toe_obs_world;
    // std::cout << "L_left_toe_obs_world: \n"
    //           << L_left_toe_obs_world << "\n\n";
    // std::cout << "L_left_toe_obs_aligned: \n"
    //           << L_left_toe_obs_aligned_ << "\n\n";

    /* Stance and swing leg assignment */
    if (stance_leg_ == -1)
    {
        p_st_toe_aligned = p_left_toe_aligned;
        p_sw_toe_aligned = p_right_toe_aligned;
        p_com_wrt_st_aligned = p_com_wrt_left_toe_aligned;
        L_st_obs_aligned = L_left_toe_obs_aligned_;
        L_sw_obs_aligned = L_right_toe_obs_aligned_;

        gen::kin::y_vc_walk_flat_LS(y_vc_walk_flat, q_all, st_heading_bos_mat_);
        // std::cout << "yaw: " << q_all(DIGIT_Q_COORD::qbase_yaw) << ", heading: " << st_heading_bos_ << "\n";
        // std::cout << "y_vc: \n"
        //           << y_vc_walk_flat << "\n\n";

        // double yaw2 = degToRad(-135);
        // double head2;
        // transform_utils::Wrap_To_Pi(head2, yaw2 - q_all(DIGIT_Q_COORD::qleftHipYaw));
        // Matrix<double, 1, 1> head2_mat(head2);
        // Matrix<double, constants::NUM_Q_ALL, 1> q_all2 = q_all;
        // q_all2(DIGIT_Q_COORD::qbase_yaw) = yaw2;
        // Vector4d y_vc2;
        // gen::kin::y_vc_walk_flat_LS(y_vc2, q_all2, head2_mat);
        // std::cout << "yaw2: " << yaw2 << ", head2: " << head2 << "\n";
        // std::cout << "y_vc2: \n"
        //           << y_vc2 << "\n\n";
    }
    else
    {
        p_st_toe_aligned = p_right_toe_aligned;
        p_sw_toe_aligned = p_left_toe_aligned;
        p_com_wrt_st_aligned = p_com_wrt_right_toe_aligned;
        L_st_obs_aligned = L_right_toe_obs_aligned_;
        L_sw_obs_aligned = L_left_toe_obs_aligned_;

        gen::kin::y_vc_walk_flat_RS(y_vc_walk_flat, q_all, st_heading_bos_mat_);
        // std::cout << "yaw: " << q_all(DIGIT_Q_COORD::qbase_yaw) << ", heading: " << st_heading_bos_ << "\n";
        // std::cout << "y_vc: \n"
        //           << y_vc_walk_flat << "\n\n";

        // double yaw2 = degToRad(270);
        // double head2;
        // transform_utils::Wrap_To_Pi(head2, yaw2 - q_all(DIGIT_Q_COORD::qrightHipYaw));
        // Matrix<double, 1, 1> head2_mat(head2);
        // Matrix<double, constants::NUM_Q_ALL, 1> q_all2 = q_all;
        // q_all2(DIGIT_Q_COORD::qbase_yaw) = yaw2;
        // Vector4d y_vc2;
        // gen::kin::y_vc_walk_flat_RS(y_vc2, q_all2, head2_mat);
        // std::cout << "yaw2: " << yaw2 << ", head2: " << head2 << "\n";
        // std::cout << "y_vc2: \n"
        //           << y_vc2 << "\n\n";
    }

    return;
}
void Digit_Controller::Walking_Kalman_Filter_Angular_Momentum_(const Ref<const Vector3d> p_com_wrt_st_aligned,
                                                               const Ref<const Vector3d> L_st_obs_aligned)
{
    /* Lx */
    // double At_Lx = 1;
    // double At_y = -sample_time_ctrl_ * mass_ * 9.81;
    // double Ct_Lx = 1;
    // double Qt_Lx = 0.5 * 0.5;
    // double Qt_yc = 0.1 * 0.1;
    // double Lx_st_kf_aligned_prev = Lx_st_kf_aligned_;
    // double cov_Lx_st_kf_aligned_prev = cov_Lx_st_kf_aligned_;

    // double Lx_predict_aligned = At_Lx * Lx_st_kf_aligned_prev + At_y * p_com_wrt_st_aligned[1];
    // double cov_Lx_predict = At_Lx * cov_Lx_st_kf_aligned_prev * At_Lx + At_y * Qt_yc * At_Lx;

    // double Kt_Lx = cov_Lx_predict * Ct_Lx * pow(Ct_Lx * cov_Lx_predict * Ct_Lx + Qt_Lx, -1);
    // Lx_st_kf_aligned_ = Lx_st_kf_aligned_prev + Kt_Lx * (L_st_obs_aligned(0) - Ct_Lx * Lx_predict_aligned);
    // cov_Lx_st_kf_aligned_ = (1 - Kt_Lx * Ct_Lx) * cov_Lx_predict;
    Lx_st_kf_aligned_ = L_st_obs_aligned(0);

    /* Ly */
    // double At_Ly = 1;                               // state matrix Ly
    // double At_x = sample_time_ctrl_ * mass_ * 9.81; // state matrix xc
    // double Ct_Ly = 1;                               // outuput matrix is identity
    // double Qt_Ly = 0.5 * 0.5;
    // double Qt_xc = 0.1 * 0.1;
    // double Ly_st_kf_aligned_prev = Ly_st_kf_aligned_;
    // double cov_Ly_st_kf_aligned_prev = cov_Ly_st_kf_aligned_;

    // double Ly_predict_aligned = At_Ly * Ly_st_kf_aligned_prev + At_x * p_com_wrt_st_aligned[0];
    // double cov_Ly_predict = At_Ly * cov_Ly_st_kf_aligned_prev * At_Ly + At_x * Qt_xc * At_x;

    // double Kt_Ly = cov_Ly_predict * Ct_Ly * pow(Ct_Ly * cov_Ly_predict * Ct_Ly + Qt_Ly, -1);
    // Ly_st_kf_aligned_ = Ly_st_kf_aligned_prev + Kt_Ly * (L_st_obs_aligned(1) - Ct_Ly * Ly_predict_aligned);
    // cov_Ly_st_kf_aligned_ = (1 - Kt_Ly * Ct_Ly) * cov_Ly_predict;
    Ly_st_kf_aligned_ = L_st_obs_aligned(1);

    return;
}
void Digit_Controller::Walking_Filter_Targets_()
{
    /* First order filter */
    double p_vx = vel_x_filter_param_;
    double p_vy = vel_y_filter_param_;
    double p_turn_rps = turn_rps_filter_param_;
    vel_x_des_filtered_ = (1 - p_vx) * vel_x_des_filtered_ + p_vx * vel_x_des_tuned_;
    vel_y_des_filtered_ = (1 - p_vy) * vel_y_des_filtered_ + p_vy * vel_y_des_tuned_;
    turn_rps_filtered_ = (1 - p_turn_rps) * turn_rps_filtered_ + p_turn_rps * turn_rps_tuned_;

    // std::cout << "vel_x_tuned: " << vel_x_des_tuned_ << "\n";
    // std::cout << "vel_x_filtered: " << vel_x_des_filtered_ << "\n";

    return;
}
void Digit_Controller::Walking_Compute_1Step_Foot_Placement_(Ref<Vector4d> y_virt_fp_des,
                                                             Ref<Vector4d> ydot_virt_fp_des,
                                                             const double &s_time,
                                                             const double &sdot_time,
                                                             const double &kx_world,
                                                             const double &ky_world,
                                                             const Ref<const Vector3d> p_com_wrt_st_aligned)
{
    // Foot placment (Assumed to align with stance toe heading at beginning of step)
    double lip_constant = sqrt(9.81 / zH_);
    double T_r = (1 - s_time) * time_step_period_;

    // Rotate target velocities into body velocities (vel_x_des and vel_y_des are in target frame)
    Vector3d vel_des_target(vel_x_des_filtered_, vel_y_des_filtered_, 0);
    // double target_st_heading_diff;
    // Matrix<double,3,3> Rz_target_st_heading_diff;
    // transform_utils::Wrap_To_Pi(target_st_heading_diff, target_heading_ - st_heading_bos_);
    // transform_utils::Euler_Yaw_Angle_To_Rotation_Matrix(Rz_target_st_heading_diff, target_st_heading_diff);
    // Vector3d vel_des_st = Rz_target_st_heading_diff.transpose() * vel_des_target;
    Vector3d vel_des_st = vel_des_target;

    // Lx and Ly des
    double Lx_des, Ly_des;
    double xc = p_com_wrt_st_aligned(0);
    double yc = p_com_wrt_st_aligned(1);
    if (stance_leg_ == -1)
    {
        Lx_des = (-mass_ * zH_ * vel_des_st(1)) - 0.5 * mass_ * zH_ * step_width_ * ((lip_constant * sinh(lip_constant * time_step_period_)) / (1 + cosh(lip_constant * time_step_period_)));
    }
    else
    {
        Lx_des = (-mass_ * zH_ * vel_des_st(1)) + 0.5 * mass_ * zH_ * step_width_ * ((lip_constant * sinh(lip_constant * time_step_period_)) / (1 + cosh(lip_constant * time_step_period_)));
    }
    Ly_des = mass_ * zH_ * vel_des_st(0);

    // Compute desired swing toe position wrt stance toe at end of current step
    double xc_eos_est = cosh(lip_constant * T_r) * xc + (1 / (mass_ * zH_ * lip_constant)) * sinh(lip_constant * T_r) * Ly_st_kf_aligned_;
    double yc_eos_est = cosh(lip_constant * T_r) * yc - (1 / (mass_ * zH_ * lip_constant)) * sinh(lip_constant * T_r) * Lx_st_kf_aligned_;

    // Use AlIP to estimate angular momentum at end of current step   ... fix time remaining s_time
    double Lx_eos_est = -1 * mass_ * zH_ * lip_constant * sinh(lip_constant * T_r) * yc + cosh(lip_constant * T_r) * Lx_st_kf_aligned_;
    double Ly_eos_est = mass_ * zH_ * lip_constant * sinh(lip_constant * T_r) * xc + cosh(lip_constant * T_r) * Ly_st_kf_aligned_;

    // Use ALIP model to compute relative com positions at end of current step (foot placement solution). Assumes Ly and Lx are set at beginning and end of step based on desired values
    double p_com_wrt_sw_eos_x = (Ly_des - cosh(lip_constant * time_step_period_) * Ly_eos_est) / (mass_ * zH_ * lip_constant * sinh(lip_constant * time_step_period_)); // swing at eos is the stance at next beginning of step (bos)
    double p_com_wrt_sw_eos_y = -(Lx_des - cosh(lip_constant * time_step_period_) * Lx_eos_est) / (mass_ * zH_ * lip_constant * sinh(lip_constant * time_step_period_));

    Vector3d p_sw_wrt_st_des(xc_eos_est - p_com_wrt_sw_eos_x, yc_eos_est - p_com_wrt_sw_eos_y, 0);

    /* Limit x foot placement to avoid hyperextension */
    double max_x_step = 0.7;
    p_sw_wrt_st_des(0) = std::max(std::min(p_sw_wrt_st_des(0), max_x_step), -max_x_step);

    /* Limit y foot placement so that legs dont collide */
    double max_y_step = 0.6;
    double min_y_step = 0.10;
    p_sw_wrt_st_des(1) = stance_leg_ * std::max(std::min(std::abs(p_sw_wrt_st_des(1)), max_y_step), min_y_step);

    // std::cout << "ufp_x_sol (1step): " << p_sw_wrt_st_des(0) << "\n";
    // std::cout << "ufp_y_sol (1step): " << p_sw_wrt_st_des(1) << "\n";

    /* Align foot placement with target direction */
    Vector3d p_sw_wrt_st_des_aligned = p_sw_wrt_st_des; // body to target heading
    // Vector3d p_sw_wrt_st_des_aligned = Rz_target_heading_bos_ * p_sw_wrt_st_des; // body to target heading
    double p_sw_wrt_st_des_x_aligned = p_sw_wrt_st_des_aligned(0); // Going from body to world so no transpose
    double p_sw_wrt_st_des_y_aligned = p_sw_wrt_st_des_aligned(1);

    // Desired Virtual Constraints related to foot placement
    // des p_sw_wrt_st_x: 0.5 * (1 + cos(M_PI * s_time_)) * p_sw_wrt_st_toe_bos_[0] + (1 - cos(M_PI * s_time_)) * p_sw_wrt_st_des_x)
    y_virt_fp_des(0) = 0.5 * (p_sw_wrt_st_toe_bos_aligned_(0) - p_sw_wrt_st_des_x_aligned) * cos(M_PI * s_time) + 0.5 * (p_sw_wrt_st_toe_bos_aligned_(0) + p_sw_wrt_st_des_x_aligned); // p_sw_wrt_st_aligned_
    ydot_virt_fp_des(0) = -0.5 * M_PI * sdot_time * (p_sw_wrt_st_toe_bos_aligned_(0) - p_sw_wrt_st_des_x_aligned) * sin(M_PI * s_time);

    // p_sw_wrt_st_aligned_y
    y_virt_fp_des(1) = 0.5 * (p_sw_wrt_st_toe_bos_aligned_(1) - p_sw_wrt_st_des_y_aligned) * cos(M_PI * s_time) + 0.5 * (p_sw_wrt_st_toe_bos_aligned_(1) + p_sw_wrt_st_des_y_aligned);
    ydot_virt_fp_des(1) = -0.5 * M_PI * sdot_time * (p_sw_wrt_st_toe_bos_aligned_(1) - p_sw_wrt_st_des_y_aligned) * sin(M_PI * s_time);

    // p_sw_wrt_st_aligned_z
    double para_a2 = z_clearance_ / (s_clearance_ - 1) - (z_clearance_ - p_sw_wrt_st_toe_bos_aligned_(2) + kx_world * p_sw_wrt_st_des_x_aligned + ky_world * p_sw_wrt_st_des_y_aligned) / s_clearance_;
    double para_a1 = -(z_clearance_ - p_sw_wrt_st_toe_bos_aligned_(2) + kx_world * p_sw_wrt_st_des_x_aligned + ky_world * p_sw_wrt_st_des_y_aligned + p_sw_wrt_st_toe_bos_aligned_(2) * s_clearance_ * s_clearance_ - kx_world * s_clearance_ * s_clearance_ * p_sw_wrt_st_des_x_aligned - ky_world * s_clearance_ * s_clearance_ * p_sw_wrt_st_des_y_aligned) / (s_clearance_ * (s_clearance_ - 1));
    double para_a0 = p_sw_wrt_st_toe_bos_aligned_(2);
    y_virt_fp_des(2) = para_a2 * s_time * s_time + para_a1 * s_time + para_a0;
    ydot_virt_fp_des(2) = (2 * s_time * para_a2 + para_a1) * sdot_time; // time derivative

    // p_com_proj_z
    y_virt_fp_des(3) = zH_;
    ydot_virt_fp_des(3) = 0;

    return;
}
void Digit_Controller::Walking_Compute_IK_(Ref<Matrix<double, constants::NUM_Q_ALL, 1>> q_all_des,
                                           Ref<Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all_des,
                                           const double &kx_world,
                                           const double &ky_world,
                                           const Ref<const Matrix<double, 1, 1>> st_heading_mat,
                                           const Ref<const Vector4d> y_vc_walk_flat_ik_des,
                                           const Ref<const Vector4d> ydot_vc_walk_flat_ik_des,
                                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all)
{
    /* IK Variables */
    int iter_ik = 1;

    Vector2d k_world(kx_world, ky_world);

    Vector4d y_vc_walk_flat_ik;
    Vector4d y_vc_walk_flat_ik_error;
    Matrix<double, 4, constants::NUM_Q_ALL> Jy_vc_walk_flat_ik;
    Matrix<double, 4, 4> Jy_vc_walk_flat_ik_square;
    ColPivHouseholderQR<Matrix<double, 4, 4>> Jy_vc_walk_flat_ik_square_QR;

    Vector4d q_ik_delta;

    std::vector<int> q_ik_indices{idx_q_sw_hiproll_, idx_q_sw_hippitch_, idx_q_sw_knee_, idx_q_st_knee_};
    Vector4d q_ik_lim_min(q_motors_min_(idx_m_sw_hiproll_), q_motors_min_(idx_m_sw_hippitch_), q_motors_min_(idx_m_sw_knee_), q_motors_min_(idx_m_st_knee_));
    Vector4d q_ik_lim_max(q_motors_max_(idx_m_sw_hiproll_), q_motors_max_(idx_m_sw_hippitch_), q_motors_max_(idx_m_sw_knee_), q_motors_max_(idx_m_st_knee_));

    /* IK q_all_des guess */
    q_all_des = q_all;
    qdot_all_des = qdot_all;

    // Enforce Rigid springs
    q_all_des(idx_q_st_KneeToShin_) = 0;
    q_all_des(idx_q_sw_KneeToShin_) = 0;
    q_all_des(idx_q_st_ShinToTarsus_) = -q_all(idx_q_st_knee_);
    q_all_des(idx_q_sw_ShinToTarsus_) = -q_all(idx_q_sw_knee_);

    qdot_all_des(idx_q_st_KneeToShin_) = 0;
    qdot_all_des(idx_q_sw_KneeToShin_) = 0;
    qdot_all_des(idx_q_st_ShinToTarsus_) = -qdot_all_des(idx_q_st_knee_);
    qdot_all_des(idx_q_sw_ShinToTarsus_) = -qdot_all_des(idx_q_sw_knee_);

    /* IK Loop */
    while (true)
    {
        // iteration check
        if (iter_ik > max_iter_ik_)
        {
            break;
        }

        /* Enforce rigid springs and geometry */
        q_all_des(idx_q_st_ShinToTarsus_) = -q_all_des(idx_q_st_knee_);
        q_all_des(idx_q_sw_ShinToTarsus_) = -q_all_des(idx_q_sw_knee_);

        /* Enforce hip geometry on stance leg since it is not controller through ik but will be controlled through torso balance */
        q_all_des(idx_q_st_hippitch_) = q_all(idx_q_st_hippitch_) + 0.5 * (q_all_des(idx_q_st_knee_) - q_all(idx_q_st_knee_));

        /* Compute y_virt_fp_ik */
        if (stance_leg_ == -1)
        {
            gen::kin::y_vc_walk_flat_LS(y_vc_walk_flat_ik, q_all_des, st_heading_mat);
            gen::kin::Jy_vc_walk_flat_LS(Jy_vc_walk_flat_ik.reshaped(4 * constants::NUM_Q_ALL, 1), q_all_des, st_heading_mat);
            // std::cout << "yaw: " << q_all_des(DIGIT_Q_COORD::qbase_yaw) << ", heading: " << st_heading_mat(0) << "\n";
            // std::cout << "y_vc_ik: \n"
            //           << y_vc_walk_flat_ik << "\n\n";

            // double yaw2 = degToRad(-135);
            // double head2;
            // transform_utils::Wrap_To_Pi(head2, yaw2 - q_all_des(DIGIT_Q_COORD::qleftHipYaw));
            // Matrix<double, 1, 1> head2_mat(head2);
            // Matrix<double, constants::NUM_Q_ALL, 1> q_all2 = q_all_des;
            // q_all2(DIGIT_Q_COORD::qbase_yaw) = yaw2;
            // Vector4d y_vc2;
            // gen::kin::y_vc_walk_flat_LS(y_vc2, q_all2, head2_mat);
            // std::cout << "yaw2: " << yaw2 << ", head2: " << head2 << "\n";
            // std::cout << "y_vc2: \n"
            //           << y_vc2 << "\n\n";
        }
        else
        {
            gen::kin::y_vc_walk_flat_RS(y_vc_walk_flat_ik, q_all_des, st_heading_mat);
            gen::kin::Jy_vc_walk_flat_RS(Jy_vc_walk_flat_ik.reshaped(4 * constants::NUM_Q_ALL, 1), q_all_des, st_heading_mat);
            // std::cout << "yaw: " << q_all_des(DIGIT_Q_COORD::qbase_yaw) << ", heading: " << st_heading_mat(0) << "\n";
            // std::cout << "y_vc_ik: \n"
            //           << y_vc_walk_flat_ik << "\n\n";

            // double yaw2 = degToRad(-135);
            // double head2;
            // transform_utils::Wrap_To_Pi(head2, yaw2 - q_all_des(DIGIT_Q_COORD::qrightHipYaw));
            // Matrix<double, 1, 1> head2_mat(head2);
            // Matrix<double, constants::NUM_Q_ALL, 1> q_all2 = q_all_des;
            // q_all2(DIGIT_Q_COORD::qbase_yaw) = yaw2;
            // Vector4d y_vc2;
            // gen::kin::y_vc_walk_flat_RS(y_vc2, q_all2, head2_mat);
            // std::cout << "yaw2: " << yaw2 << ", head2: " << head2 << "\n";
            // std::cout << "y_vc2: \n"
            //           << y_vc2 << "\n\n";
        }

        /* Check error tolerance and break if small enough */
        y_vc_walk_flat_ik_error = y_vc_walk_flat_ik - y_vc_walk_flat_ik_des; // vc error

        /* Check vc tolerance */
        if (y_vc_walk_flat_ik_error.cwiseAbs().maxCoeff() < tol_ik_)
        {
            break;
        }

        /* Compute output jacobian */
        Jy_vc_walk_flat_ik_square = Jy_vc_walk_flat_ik(all, q_ik_indices);

        /* Enforce rigid spring */
        Jy_vc_walk_flat_ik_square(all, 2) = Jy_vc_walk_flat_ik(all, idx_q_sw_knee_) - Jy_vc_walk_flat_ik(all, idx_q_sw_ShinToTarsus_); // for z fp
        Jy_vc_walk_flat_ik_square(all, 3) = Jy_vc_walk_flat_ik(all, idx_q_st_knee_) - Jy_vc_walk_flat_ik(all, idx_q_st_ShinToTarsus_); // for com projection

        /* Enforce geometric constraint on stance leg */
        Jy_vc_walk_flat_ik_square(all, 3) = Jy_vc_walk_flat_ik_square(all, 3) + 0.5 * Jy_vc_walk_flat_ik(all, idx_q_st_hippitch_); // hip pitch positive direction is opposite of cassie on left leg

        /* QR decomposition of Jacobian */
        Jy_vc_walk_flat_ik_square_QR = Jy_vc_walk_flat_ik_square.colPivHouseholderQr();

        /* Jacobian conditioning check */
        Jy_vc_walk_flat_ik_square_QR.setThreshold(ik_threshold_);
        // std::cout << "threshold: " << Jy_vc_walk_flat_ik_square_QR.threshold() << "\n";
        // std::cout << "max_pivot: " << Jy_vc_walk_flat_ik_square_QR.maxPivot() << "\n";
        // std::cout << "R: \n"
        //           << Jy_vc_walk_flat_ik_square_QR.matrixR() << "\n";
        if (Jy_vc_walk_flat_ik_square_QR.rank() < 4)
        {
            std::cout << "========================\nILL-CONDITIONED JACOBIAN \n========================\n\n";
            std::cout << "Rank: " << Jy_vc_walk_flat_ik_square_QR.rank() << "\n\n";
        }

        /* compute q_all_delta */
        q_ik_delta = Jy_vc_walk_flat_ik_square_QR.solve(y_vc_walk_flat_ik_error);

        /* limit delta size */
        double max_q_all_delta = q_ik_delta.cwiseAbs().maxCoeff();
        if (max_q_all_delta > scale_delta_ik_)
        {
            q_ik_delta = scale_delta_ik_ * (q_ik_delta / max_q_all_delta);
        }

        /* Add delta */
        q_all_des(q_ik_indices) -= q_ik_delta;

        /* clamp joint limits */
        q_all_des(idx_q_sw_hiproll_) = std::max(std::min(q_all_des(idx_q_sw_hiproll_), q_ik_lim_max[0]), q_ik_lim_min[0]);
        q_all_des(idx_q_sw_hippitch_) = std::max(std::min(q_all_des(idx_q_sw_hippitch_), q_ik_lim_max[1]), q_ik_lim_min[1]);
        q_all_des(idx_q_sw_knee_) = std::max(std::min(q_all_des(idx_q_sw_knee_), q_ik_lim_max[2]), q_ik_lim_min[2]);
        q_all_des(idx_q_st_knee_) = std::max(std::min(q_all_des(idx_q_st_knee_), q_ik_lim_max[3]), q_ik_lim_min[3]);

        /* Increase iteration */
        iter_ik++;
    }
    // std::cout << "iterations: " << iter_ik << "\n";
    // std::cout << "y_error: \n"
    //           << y_vc_walk_flat_ik_error << "\n\n";

    /* desired velocities */
    qdot_all_des(q_ik_indices) = Jy_vc_walk_flat_ik_square_QR.solve(ydot_vc_walk_flat_ik_des);

    return;
}
void Digit_Controller::Walking_Compute_Desired_Motors_(Ref<Matrix<double, NUM_MOTORS, 1>> q_motors_des,
                                                       Ref<Matrix<double, NUM_MOTORS, 1>> qdot_motors_des,
                                                       const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all_des,
                                                       const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all_des)
{
    /* Hip yaw */
    double target_stance_offset;
    transform_utils::Wrap_To_Pi(target_stance_offset, target_heading_ - st_heading_bos_);
    // std::cout << "target_heading: " << target_heading_ << "\n";
    // std::cout << "st_heading_bos: " << st_heading_bos_ << "\n";
    // std::cout << "target_stance_offset: " << target_stance_offset << "\n\n";
    q_motors_des(DigitMotors::LeftHipYaw) = 0;  // left looks good
    q_motors_des(DigitMotors::RightHipYaw) = 0; // 0

    // q_motors_des(DigitMotors::LeftHipYaw) = -0.5 * target_stance_offset;  // left looks good
    // q_motors_des(DigitMotors::RightHipYaw) = -0.5 * target_stance_offset; // 0
    qdot_motors_des(DigitMotors::LeftHipYaw) = 0;
    qdot_motors_des(DigitMotors::RightHipYaw) = 0;

    /* Legs except for stance hip roll and hip pitch */
    if (stance_leg_ == -1) // left stance
    {
        // q_motors_des(DigitMotors::LeftHipRoll) = q_all_des(DIGIT_Q_COORD::qleftHipRoll);
        // q_motors_des(DigitMotors::LeftHipPitch) = q_all_des(DIGIT_Q_COORD::qleftHipPitch);
        q_motors_des(DigitMotors::LeftKnee) = q_all_des(DIGIT_Q_COORD::qleftKnee);
        q_motors_des(DigitMotors::RightHipRoll) = q_all_des(DIGIT_Q_COORD::qrightHipRoll);
        q_motors_des(DigitMotors::RightHipPitch) = q_all_des(DIGIT_Q_COORD::qrightHipPitch);
        q_motors_des(DigitMotors::RightKnee) = q_all_des(DIGIT_Q_COORD::qrightKnee);

        // qdot_motors_des(DigitMotors::LeftHipRoll) = qdot_all_des(DIGIT_Q_COORD::qleftHipRoll);
        // qdot_motors_des(DigitMotors::LeftHipPitch) = qdot_all_des(DIGIT_Q_COORD::qleftHipPitch);
        qdot_motors_des(DigitMotors::LeftKnee) = qdot_all_des(DIGIT_Q_COORD::qleftKnee);
        qdot_motors_des(DigitMotors::RightHipRoll) = qdot_all_des(DIGIT_Q_COORD::qrightHipRoll);
        qdot_motors_des(DigitMotors::RightHipPitch) = qdot_all_des(DIGIT_Q_COORD::qrightHipPitch);
        qdot_motors_des(DigitMotors::RightKnee) = qdot_all_des(DIGIT_Q_COORD::qrightKnee);
    }
    else // right stance
    {
        q_motors_des(DigitMotors::LeftHipRoll) = q_all_des(DIGIT_Q_COORD::qleftHipRoll);
        q_motors_des(DigitMotors::LeftHipPitch) = q_all_des(DIGIT_Q_COORD::qleftHipPitch);
        q_motors_des(DigitMotors::LeftKnee) = q_all_des(DIGIT_Q_COORD::qleftKnee);
        // q_motors_des(DigitMotors::RightHipRoll) = q_all_des(DIGIT_Q_COORD::qrightHipRoll);
        // q_motors_des(DigitMotors::RightHipPitch) = q_all_des(DIGIT_Q_COORD::qrightHipPitch);
        q_motors_des(DigitMotors::RightKnee) = q_all_des(DIGIT_Q_COORD::qrightKnee);

        qdot_motors_des(DigitMotors::LeftHipRoll) = qdot_all_des(DIGIT_Q_COORD::qleftHipRoll);
        qdot_motors_des(DigitMotors::LeftHipPitch) = qdot_all_des(DIGIT_Q_COORD::qleftHipPitch);
        qdot_motors_des(DigitMotors::LeftKnee) = qdot_all_des(DIGIT_Q_COORD::qleftKnee);
        // qdot_motors_des(DigitMotors::RightHipRoll) = qdot_all_des(DIGIT_Q_COORD::qrightHipRoll);
        // qdot_motors_des(DigitMotors::RightHipPitch) = qdot_all_des(DIGIT_Q_COORD::qrightHipPitch);
        qdot_motors_des(DigitMotors::RightKnee) = qdot_all_des(DIGIT_Q_COORD::qrightKnee);
    }

    // toes
    if (stance_leg_ == -1) // left stance
    {
        // stance (not used bc zero stance foot torque)
        q_motors_des(DigitMotors::LeftToeA) = 0;
        q_motors_des(DigitMotors::LeftToeB) = 0;

        // swing
        double swing_toe_pitch = 0; // kx_world
        double swing_toe_roll = 0;  // ky_world
        double swing_toeA, swing_toeB;
        Digit_Controller::Toe_Joints_To_Motors_(swing_toeA, swing_toeB, swing_toe_pitch, swing_toe_roll, "right");
        q_motors_des(DigitMotors::RightToeA) = swing_toeA;
        q_motors_des(DigitMotors::RightToeB) = swing_toeB;
    }
    else
    {
        // stance (not used)
        q_motors_des(DigitMotors::RightToeA) = 0;
        q_motors_des(DigitMotors::RightToeB) = 0;

        // swing
        double swing_toe_pitch = 0; // kx_world
        double swing_toe_roll = 0;  // ky_world
        double swing_toeA, swing_toeB;
        Digit_Controller::Toe_Joints_To_Motors_(swing_toeA, swing_toeB, swing_toe_pitch, swing_toe_roll, "left");
        q_motors_des(DigitMotors::LeftToeA) = swing_toeA;
        q_motors_des(DigitMotors::LeftToeB) = swing_toeB;
    }

    // arms
    double shoulder_roll_des = -0.15;
    double shoulder_pitch_des = 1.1;
    double shoulder_yaw_des = 0;
    double elbow_des = -0.145;

    q_motors_des(DigitMotors::LeftShoulderRoll) = shoulder_roll_des;
    q_motors_des(DigitMotors::LeftShoulderPitch) = shoulder_pitch_des;
    q_motors_des(DigitMotors::LeftShoulderYaw) = shoulder_yaw_des;
    q_motors_des(DigitMotors::LeftElbow) = elbow_des;

    q_motors_des(DigitMotors::RightShoulderRoll) = -shoulder_roll_des;
    q_motors_des(DigitMotors::RightShoulderPitch) = -shoulder_pitch_des;
    q_motors_des(DigitMotors::RightShoulderYaw) = -shoulder_yaw_des;
    q_motors_des(DigitMotors::RightElbow) = -elbow_des;

    return;
}
void Digit_Controller::Walking_Log_Data_(const double &time_system,
                                         const double &s_time,
                                         const double &sdot_time,
                                         const double &kx_world,
                                         const double &ky_world,
                                         const double &st_heading_bos,
                                         const int &stance_leg,
                                         const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                         const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                                         const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all_des,
                                         const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all_des,
                                         const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors,
                                         const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors,
                                         const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors_des,
                                         const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors_des,
                                         const Ref<const Matrix<double, NUM_MOTORS, 1>> motor_torque_command,
                                         const Ref<const Matrix<double, 4, 1>> y_vc_walk_flat,
                                         const Ref<const Matrix<double, 4, 1>> ydot_vc_walk_flat,
                                         const Ref<const Matrix<double, 4, 1>> y_vc_walk_flat_des,
                                         const Ref<const Matrix<double, 4, 1>> ydot_vc_walk_flat_des,
                                         const Ref<const Vector3d> p_st_toe_aligned,
                                         const Ref<const Vector3d> p_sw_toe_aligned,
                                         const Ref<const Vector3d> p_com_wrt_st_aligned,
                                         const Ref<const Vector3d> v_com_aligned,
                                         const Ref<const Vector3d> L_st_obs_aligned,
                                         const Ref<const Vector3d> L_sw_obs_aligned)
{
    // ctrl_logfile_.open(path_to_log_, std::ofstream::out | std::ofstream::app);

    /* Log order
            time_system,
            s_time,
            sdot_time,
            kx_world,
            ky_world,
            st_heading_bos,
            stance_leg
            q_all,
            qdot_all,
            q_all_des,
            qdot_all_des,
            q_motors,
            qdot_motors,
            q_motors_des,
            qdot_motors_des,
            motor_torque_command,
            y_vc_walk_flat,
            ydot_vc_walk_flat,
            y_vc_walk_flat_des,
            ydot_vc_walk_flat_des,
            p_st_toe_aligned,
            p_sw_toe_aligned,
            p_com_wrt_st_aligned,
            v_com_aligned,
            L_st_obs_aligned,
            L_sw_obs_aligned


    /* time_system */
    ctrl_logfile_ << std::fixed << time_system << ",\n";

    /* s_time */
    ctrl_logfile_ << std::fixed << s_time << ",\n";

    /* sdot_time */
    ctrl_logfile_ << std::fixed << sdot_time << ",\n";

    /* kx_world */
    ctrl_logfile_ << std::fixed << kx_world << ",\n";

    /* ky_world */
    ctrl_logfile_ << std::fixed << ky_world << ",\n";

    /* st_heading_bos */
    ctrl_logfile_ << std::fixed << st_heading_bos << ",\n";

    /* stance_leg */
    ctrl_logfile_ << std::fixed << stance_leg << ",\n";

    /* q_all */
    for (auto i : std::vector<double>(q_all.data(), q_all.data() + q_all.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_all */
    for (auto i : std::vector<double>(qdot_all.data(), qdot_all.data() + qdot_all.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* q_all_des */
    for (auto i : std::vector<double>(q_all_des.data(), q_all_des.data() + q_all_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_all_des */
    for (auto i : std::vector<double>(qdot_all_des.data(), qdot_all_des.data() + qdot_all_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* q_motors */
    for (auto i : std::vector<double>(q_motors.data(), q_motors.data() + q_motors.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_motors */
    for (auto i : std::vector<double>(qdot_motors.data(), qdot_motors.data() + qdot_motors.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* q_motors_des */
    for (auto i : std::vector<double>(q_motors_des.data(), q_motors_des.data() + q_motors_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* qdot_motors_des */
    for (auto i : std::vector<double>(qdot_motors_des.data(), qdot_motors_des.data() + qdot_motors_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* motor_torque_command */
    for (auto i : std::vector<double>(motor_torque_command.data(), motor_torque_command.data() + motor_torque_command.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* y_vc_walk_flat */
    for (auto i : std::vector<double>(y_vc_walk_flat.data(), y_vc_walk_flat.data() + y_vc_walk_flat.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* ydot_vc_walk_flat */
    for (auto i : std::vector<double>(ydot_vc_walk_flat.data(), ydot_vc_walk_flat.data() + ydot_vc_walk_flat.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* y_vc_walk_flat_des */
    for (auto i : std::vector<double>(y_vc_walk_flat_des.data(), y_vc_walk_flat_des.data() + y_vc_walk_flat_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* ydot_vc_walk_flat_des */
    for (auto i : std::vector<double>(ydot_vc_walk_flat_des.data(), ydot_vc_walk_flat_des.data() + ydot_vc_walk_flat_des.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* p_st_toe_aligned */
    for (auto i : std::vector<double>(p_st_toe_aligned.data(), p_st_toe_aligned.data() + p_st_toe_aligned.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* p_sw_toe_aligned */
    for (auto i : std::vector<double>(p_sw_toe_aligned.data(), p_sw_toe_aligned.data() + p_sw_toe_aligned.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* p_com_wrt_st_aligned */
    for (auto i : std::vector<double>(p_com_wrt_st_aligned.data(), p_com_wrt_st_aligned.data() + p_com_wrt_st_aligned.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* v_com_aligned */
    for (auto i : std::vector<double>(v_com_aligned.data(), v_com_aligned.data() + v_com_aligned.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* L_st_obs_aligned */
    for (auto i : std::vector<double>(L_st_obs_aligned.data(), L_st_obs_aligned.data() + L_st_obs_aligned.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    /* L_sw_obs_aligned */
    for (auto i : std::vector<double>(L_sw_obs_aligned.data(), L_sw_obs_aligned.data() + L_sw_obs_aligned.size()))
    {
        ctrl_logfile_ << i << ",";
    }
    ctrl_logfile_ << ",\n";

    // ctrl_logfile_.close();

    return;
}

/***************************  Utilitiy Functions ***************************/
void Digit_Controller::Ramp_Torque_(const double &t_ramp_period)
{
    // double ratio = (time_system_ - (time_mode_change_ + time_transition_)) / t_ramp_period;
    // motor_torque_command_ = (1 - ratio) * motor_torque_measured_ + ratio * motor_torque_command_;
    return;
}
void Digit_Controller::Saturate_Torque_Command_(Ref<Matrix<double, NUM_MOTORS, 1>> motor_torque_command,
                                                const llapi_limits_t *const lim)
{
    for (int i = 0; i < constants::NUM_ACTUATOR_JOINTS; i++)
    {
        double max_torque = lim->torque_limit[i];
        double min_torque = -max_torque;
        motor_torque_command[i] = std::min(std::max(motor_torque_command[i], min_torque), max_torque);
    }
    return;
}
void Digit_Controller::Torque_Compensation_()
{
    return;
}
void Digit_Controller::Toe_Joints_To_Motors_(double &toeA,
                                             double &toeB,
                                             const double &toe_pitch,
                                             const double &toe_roll,
                                             const std::string &which_leg)
{
    /* Data-based regression coefficients */
    // quadratic regression form used:
    // c0 + c1*x1 + c2*x2 + c3*x1*x2 + c4*x1^2 + c5*x2^2
    VectorXd coeff_left_toeA(6, 1);
    coeff_left_toeA << -0.000020104839746, -0.953186364443127, 0.287300022579382, 0.126383330897287, -0.104545118387107, 0.069993489815834;
    VectorXd coeff_left_toeB(6, 1);
    coeff_left_toeB << -0.001510578286345, 0.956673782118180, 0.289477357556638, 0.130724573976602, 0.109486127052803, -0.052344284202339;
    VectorXd coeff_right_toeA(6, 1);
    coeff_right_toeA << -0.002408137779226, -0.954674775623872, 0.291954695875912, -0.127328321732240, 0.117814537445815, -0.072417634014895;
    VectorXd coeff_right_toeB(6, 1);
    coeff_right_toeB << 0.000790978956749, 0.958090904797311, 0.294126877935193, -0.124774580828592, -0.107364114291381, 0.055477221375772;

    if (which_leg == "left") // left toe
    {
        /* Left leg swing */
        double xp = toe_pitch;
        double xr = toe_roll;
        VectorXd A_left(6, 1);
        A_left << 1.0, xp, xr, xp * xr, xp * xp, xr * xr;
        toeA = A_left.transpose() * coeff_left_toeA;
        toeB = A_left.transpose() * coeff_left_toeB;
    }
    else // right toe
    {
        /* Right leg swing */
        double xp = toe_pitch;
        double xr = toe_roll;
        VectorXd A_right(6, 1);
        A_right << 1, xp, xr, xp * xr, xp * xp, xr * xr;
        toeA = A_right.transpose() * coeff_right_toeA;
        toeB = A_right.transpose() * coeff_right_toeB;
    }

    return;
}
void Digit_Controller::Compute_GRF_()
{
    return;
}
