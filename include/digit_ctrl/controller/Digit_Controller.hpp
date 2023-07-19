/**
 *  @file   Digit_Controller.hpp
 *  @author Grant Gibson
 *  @brief  Class for Digit Controller
 *  @date   November 22, 2020
 **/
#pragma once

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <ctime>
#include <cmath>


#include "eigen340/Eigen/Dense"

#include "utils.hpp"

#include "lowlevelapi.h"

#include "gen/kin/p_hip_flexion_left.hh"
#include "gen/kin/p_hip_flexion_right.hh"
#include "gen/kin/p_toe_pitch_joint_left.hh"
#include "gen/kin/p_toe_pitch_joint_right.hh"

#include "gen/kin/p_COM.hh"
#include "gen/kin/Jp_COM.hh"
#include "gen/kin/v_COM.hh"
#include "gen/kin/p_LeftToeMid.hh"
#include "gen/kin/p_RightToeMid.hh"
#include "gen/kin/p_LeftToeFront.hh"
#include "gen/kin/p_RightToeFront.hh"

#include "gen/kin/L_world_about_point.hh"
#include "gen/kin/Jdq_L_world_about_point.hh"

#include "gen/kin/LL_left.hh"
#include "gen/kin/LL_right.hh"

#include "gen/kin/y_vc_stand.hh"
#include "gen/kin/Jy_vc_stand.hh"
#include "gen/kin/y_vc_stand_v2.hh"
#include "gen/kin/Jy_vc_stand_v2.hh"
#include "gen/kin/y_vc_stand_v3.hh"
#include "gen/kin/Jy_vc_stand_v3.hh"

#include "gen/kin/y_vc_walk_flat_LS.hh"
#include "gen/kin/y_vc_walk_flat_RS.hh"
#include "gen/kin/Jy_vc_walk_flat_LS.hh"
#include "gen/kin/Jy_vc_walk_flat_RS.hh"

#include "gen/kin/p_left_hand_wrt_base.hh"
#include "gen/kin/p_right_hand_wrt_base.hh"
#include "gen/kin/Jp_left_hand_wrt_base.hh"
#include "gen/kin/Jp_right_hand_wrt_base.hh"


#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)

// extern "C"
// {
// }
using namespace Eigen;

class Digit_Controller
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*==========================================================================
                                  PUBLIC ENUMS
    ==========================================================================*/
  enum CTRL_MODE
  {
    STANDING_ANALYTIC = 0,
    STANDING_NUMERIC = 1,
    WALKING = 2,
  };

  enum DIGIT_Q_COORD
  {
    qbase_pos_x,
    qbase_pos_y,
    qbase_pos_z,
    qbase_yaw,
    qbase_pitch,
    qbase_roll,
    qleftHipRoll,
    qleftHipYaw,
    qleftHipPitch,
    qleftKnee,
    qleftKneeToShin,
    qleftShinToTarsus,
    qleftToePitch,
    qleftToeRoll,
    qleftShoulderRoll,
    qleftShoulderPitch,
    qleftShoulderYaw,
    qleftElbow,
    qrightHipRoll,
    qrightHipYaw,
    qrightHipPitch,
    qrightKnee,
    qrightKneeToShin,
    qrightShinToTarsus,
    qrightToePitch,
    qrightToeRoll,
    qrightShoulderRoll,
    qrightShoulderPitch,
    qrightShoulderYaw,
    qrightElbow
  };

  /*==========================================================================
                              PUBLIC MEMBER FUNCTIONS
    ==========================================================================*/
  /* Constructor */
  Digit_Controller();

  /* Destructor */
  virtual ~Digit_Controller();

  /* Initialize */
  void Initialize_(const int mode, const int flag_torque_only);

  /* Update */
  void Update_(llapi_command_t &command_update,
               const llapi_observation_t &observation_update,
               const llapi_limits_t *const limits_update);

  /* Getters, Setters, Printers, Savers */
  int Get_Ctrl_Mode_();
  void Set_Ctrl_Mode_(int mode);

  // void Get_Op_Mode_(const digit_msgs::Digit_Json_Info &msg);

  void Print_Standing_Gains_();
  void Set_Initial_Standing_Gains_();
  // void Set_Keyboard_Standing_Gains_(const digit_msgs::Digit_Keyboard_Standing_Gains &msg);

  void Print_Walking_Gains_();
  void Set_Initial_Walking_Gains_();

  // void Print_Keyboard_Standing_Targets_();
  // void Set_Keyboard_Standing_Targets_(const digit_msgs::Digit_Keyboard_Standing_Targets &msg);

  // void Print_Keyboard_Walking_Targets_();
  // void Set_Keyboard_Walking_Targets_(const digit_msgs::Digit_Keyboard_Walking_Targets &msg);

  // void Print_Gamepad_Walking_Targets_();
  // void Set_Gamepad_Walking_Targets_(const digit_msgs::Digit_Gamepad_Walking_Targets &msg);

  // std::string Get_Target_Type_();

  // void Save_Gains_();

  // void Close_Logs_();

  /*==========================================================================
                               PRIVATE MEMBER FUNCTIONS
    ==========================================================================*/

  /*************************** ROS Parameter Functions ***************************/
  bool Get_Config_Parameters_();

  /*************************** Observation Functions ***************************/
  void Extract_Observation_(double &time_digit,
                            Ref<Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                            Ref<Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                            Ref<Matrix<double, NUM_MOTORS, 1>> q_motors,
                            Ref<Matrix<double, NUM_MOTORS, 1>> qdot_motors,
                            Ref<Matrix<double, NUM_UNACT_JOINTS, 1>> q_joints,
                            Ref<Matrix<double, NUM_UNACT_JOINTS, 1>> qdot_joints,
                            const llapi_observation_t &observ,
                            const llapi_limits_t *const lim);

  /*************************** GRF Functions ***************************/
  void Compute_Grf_();

  /*************************** Standing (Analytic) Controller Functions ***************************/
  void Standing_Analytic_Update_(llapi_command_t &cmd,
                                 const double &time_system,
                                 const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                 const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                                 const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors,
                                 const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors,
                                 const llapi_limits_t *const lim);
  void Standing_Analytic_Fk_From_Leg_Angle_Length_(double &LA,
                                                   double &LL,
                                                   const double &q_hippitch,
                                                   const double &q_knee);
  void Standing_Analytic_Ik_From_Thigh_Knee_(double &q_hippitch,
                                             double &q_knee,
                                             const double &LA,
                                             const double &LL,
                                             std::string whichLeg);
  void Standing_Analytic_Log_Data_(const double &time_system,
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
                                   const Ref<const Vector3d> p_right_toe);

  /*************************** Standing (Numeric) Controller Functions***************************/
  void Standing_Numeric_Update_(llapi_command_t &cmd,
                                const double &time_system,
                                const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                                const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                                const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors,
                                const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors,
                                const Ref<const Matrix<double, NUM_UNACT_JOINTS, 1>> q_joints,
                                const Ref<const Matrix<double, NUM_UNACT_JOINTS, 1>> qdot_joints,
                                const llapi_limits_t *const lim);
  void Standing_Compute_Arm_IK_(Ref<Matrix<double, constants::NUM_Q_ALL, 1>> q_all_arm_des,
                                Ref<Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all_arm_des,
                                const Ref<const Matrix<double,1,1>> st_heading,
                                const Ref<const VectorXd> y_vc_arm_des,
                                const Ref<const VectorXd> ydot_vc_arm_des);
  void Standing_Sway_(const double &time_digit,
                      const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                      double &x_offset_des,
                      double &y_offset_des,
                      double &z_offset_des,
                      double &yaw_offset_des,
                      double &pitch_offset_des,
                      double &roll_offset_des);
  void Standing_Clamp_Targets_(double &x_offset_clamped,
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
                               const double &roll_offset_target);
  void Standing_Filter_Targets_(const double &x_offset_clamped,
                                const double &y_offset_clamped,
                                const double &z_offset_clamped,
                                const double &yaw_offset_clamped,
                                const double &pitch_offset_clamped,
                                const double &roll_offset_clamped);
  void Standing_Numeric_Log_Data_(const double &time_system,
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
                                  const Ref<const Vector3d> p_right_toe);

  /*************************** Walking ALIP Controller (1Step or MPC FP) Functions ***************************/
  void Walking_Constructor_();
  void Walking_Update_(llapi_command_t &cmd,
                       const double &time_system,
                       const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                       const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all,
                       const Ref<const Matrix<double, NUM_MOTORS, 1>> q_motors,
                       const Ref<const Matrix<double, NUM_MOTORS, 1>> qdot_motors,
                       const Ref<const Matrix<double, NUM_UNACT_JOINTS, 1>> q_joints,
                       const llapi_limits_t *const lim);
  void Walking_Initialize_(const double &time_system,
                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all);
  void Walking_Check_Stance_Switch_(const double time_system,
                                    const Ref<const Matrix<double, NUM_UNACT_JOINTS, 1>> q_joints);
  void Walking_Update_Variables_At_Impact_(const double &time_system,
                                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all);
  void Walking_Update_Variables_Each_Loop_(double &kx_world,
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
                                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all);
  void Walking_Kalman_Filter_Angular_Momentum_(const Ref<const Vector3d> p_com_wrt_st_aligned,
                                               const Ref<const Vector3d> L_st_obs_aligned);
  void Walking_Filter_Targets_();
  void Walking_Compute_1Step_Foot_Placement_(Ref<Vector4d> y_virt_fp_des,
                                             Ref<Vector4d> ydot_virt_fp_des,
                                             const double &s_time,
                                             const double &sdot_time,
                                             const double &kx_world,
                                             const double &ky_world,
                                             const Ref<const Vector3d> p_com_wrt_st_aligned);
  // void Walking_Compute_MPC_Foot_Placement_(Ref<Vector4d> y_virt_fp_des,
  //                                          Ref<Vector4d> ydot_virt_fp_des,
  //                                          const double &s_time,
  //                                          const double &sdot_time,
  //                                          const double &kx_world,
  //                                          const double &ky_world,
  //                                          const Ref<const Vector3d> p_com_wrt_st_aligned);
  void Walking_Compute_IK_(Ref<Matrix<double, constants::NUM_Q_ALL, 1>> q_all_des,
                           Ref<Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all_des,
                           const double &kx_world,
                           const double &ky_world,
                           const Ref<const Matrix<double, 1, 1>> st_heading_mat,
                           const Ref<const Vector4d> y_vc_walk_flat_ik_des,
                           const Ref<const Vector4d> ydot_vc_walk_flat_ik_des,
                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all,
                           const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all);
  void Walking_Compute_Desired_Motors_(Ref<Matrix<double, NUM_MOTORS, 1>> q_motors_des,
                                       Ref<Matrix<double, NUM_MOTORS, 1>> qdot_motors_des,
                                       const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> q_all_des,
                                       const Ref<const Matrix<double, constants::NUM_Q_ALL, 1>> qdot_all_des);
  void Walking_Log_Data_(const double &time_system,
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
                         const Ref<const Vector3d> L_sw_obs_aligned);

  /*************************** Utility Functions ***************************/
  void Ramp_Torque_(const double &t_ramp_period);
  void Saturate_Torque_Command_(Ref<Matrix<double, NUM_MOTORS, 1>> motor_torque_command,
                                const llapi_limits_t *const lim);
  void Torque_Compensation_();
  void Toe_Joints_To_Motors_(double &toeA,
                             double &toeB,
                             const double &toe_pitch,
                             const double &toe_roll,
                             const std::string &which_leg);
  void Compute_GRF_();

  /*==========================================================================
                               PRIVATE MEMBER VARIABLES
    ==========================================================================*/

  /************************ ROS Variables ************************/
  // ros::NodeHandle nh_;

  /************************ System Variables ************************/
  bool stance_switch_ = false; // initial step begins without switching
  int ctrl_mode_;              // controller choose from enum ctrl_mode
  int torque_only_;
  double time_digit_prev_;
  double contact_deflection_ = 0.015; // heelspring defelction threshold for impact. right heelspring has positive deflection, left has negative
  std::string exp_mode_;
  std::string op_mode_ = "unknown (not connected";
  Matrix<double, NUM_MOTORS, 1> q_motors_min_;
  Matrix<double, NUM_MOTORS, 1> q_motors_max_;
  llapi_command_t command_update_prev_;

  /************************ Standing Variables ************************/
  double x_offset_filtered_ = 0;
  double y_offset_filtered_ = 0;
  double z_offset_filtered_ = 0;
  double yaw_offset_filtered_ = 0;
  double pitch_offset_filtered_ = 0;
  double roll_offset_filtered_ = 0;

  bool flag_standing_first_iter_ = true;
  double yaw_init_;

  bool enable_sway_yaw_ = false;
  bool enable_sway_pitch_ = false;
  bool enable_sway_roll_ = false;
  bool enable_sway_crouch_ = false;
  bool sway_yaw_start_ = true;
  bool sway_pitch_start_ = true;
  bool sway_roll_start_ = true;
  bool sway_crouch_start_ = true;
  int sway_yaw_dir_ = 1;
  int sway_pitch_dir_ = 1;
  int sway_roll_dir_ = 1;
  int sway_crouch_dir_ = 1;

  double time_sway_yaw_start_;
  double time_sway_pitch_start_;
  double time_sway_roll_start_;
  double time_sway_crouch_start_;
  double sway_yaw_init_;
  double sway_pitch_init_;
  double sway_roll_init_;
  double sway_crouch_init_;
  double sway_yaw_final_;
  double sway_pitch_final_;
  double sway_roll_final_;
  double sway_crouch_final_;

  /************************ Walking Variables ************************/
  // General
  bool flag_walking_first_iter_ = true;
  int stance_leg_;         // 1 right stance. Update_At_Impact makes the left stance be first with this convention.
  double time_step_start_; // time at start of step

  // Foot placement
  Vector3d p_sw_wrt_st_toe_bos_aligned_;
  Vector3d p_com_wrt_st_bos_aligned_;

  // MPC foot placement
  int mpc_iter_ = 0;
  int n_xlip_ = 4; // dimension of xlip state
  int n_ufp_ = 2;  // dimension of ufp control
  int N_k_;        // total # of discretized steps in optimization
  int N_fp_;       // number of foot placement predictions
  int N_xsol_;     // total size of xlip state solution trajectory (vectorized)
  int N_ufpsol_;   // total size of ufp solution trajectory (vectorized)
  std::vector<double> xlip_guess_prev_;
  std::vector<double> ufp_guess_prev_;
  // casadi::Function f_solver_LS_;
  // casadi::Function f_solver_RS_;

  // Heading
  double target_heading_;
  double st_heading_bos_;
  Matrix<double, 1, 1> st_heading_bos_mat_;
  Matrix3d Rz_st_heading_bos_;

  // Angular Momentum (Kalman Filter)
  double Lx_st_kf_aligned_;
  double Ly_st_kf_aligned_;
  double cov_Lx_st_kf_aligned_;
  double cov_Ly_st_kf_aligned_;
  Vector3d L_left_toe_obs_aligned_;
  Vector3d L_right_toe_obs_aligned_;

  // Q Indices
  int idx_q_st_hiproll_;
  int idx_q_st_hipyaw_;
  int idx_q_st_hippitch_;
  int idx_q_st_knee_;
  int idx_q_st_KneeToShin_;
  int idx_q_st_ShinToTarsus_;

  int idx_q_sw_hiproll_;
  int idx_q_sw_hipyaw_;
  int idx_q_sw_hippitch_;
  int idx_q_sw_knee_;
  int idx_q_sw_KneeToShin_;
  int idx_q_sw_ShinToTarsus_;

  // Motor Indices
  int idx_m_st_hiproll_;
  int idx_m_st_hipyaw_;
  int idx_m_st_hippitch_;
  int idx_m_st_knee_;
  int idx_m_st_ShinToTarsus_;

  int idx_m_sw_hiproll_;
  int idx_m_sw_hipyaw_;
  int idx_m_sw_hippitch_;
  int idx_m_sw_knee_;
  int idx_m_sw_ShinToTarsus_;

  /************************ Transition (Stand2Walk) Variables ************************/
  double shift_lateral_ = 0.0;

  /************************ Target Variables ************************/
  double vel_x_des_tuned_ = 0;
  double vel_y_des_tuned_ = 0;
  double turn_rps_tuned_ = 0;

  double vel_x_des_filtered_ = 0;
  double vel_y_des_filtered_ = 0;
  double turn_rps_filtered_ = 0;

  double x_offset_tuned_;
  double y_offset_tuned_;
  double z_offset_tuned_;
  double yaw_offset_tuned_;
  double pitch_offset_tuned_;
  double roll_offset_tuned_;

  /************************ Gain Variables ************************/
  DiagonalMatrix<double, constants::NUM_ACTUATOR_JOINTS> kp_stand_;
  DiagonalMatrix<double, constants::NUM_ACTUATOR_JOINTS> kp_right_st_;
  DiagonalMatrix<double, constants::NUM_ACTUATOR_JOINTS> kp_left_st_;

  // Standing
  double kp_hiproll_stand_tuned_;
  double kp_hipyaw_stand_tuned_;
  double kp_hippitch_stand_tuned_;
  double kp_knee_stand_tuned_;
  double kp_toe_stand_tuned_;
  double kp_shoulderroll_stand_tuned_;
  double kp_shoulderpitch_stand_tuned_;
  double kp_shoulderyaw_stand_tuned_;
  double kp_elbow_stand_tuned_;

  double kp_lateral_stand_tuned_;
  double kd_lateral_stand_tuned_;

  double kp_knee_comp_stand_tuned_;
  double kd_knee_comp_stand_tuned_;

  // Walking
  double kp_hiproll_sw_tuned_;
  double kp_hipyaw_sw_tuned_;
  double kp_hippitch_sw_tuned_;
  double kp_knee_sw_tuned_;
  double kp_toe_sw_tuned_;
  double kp_hiproll_st_tuned_;
  double kp_hipyaw_st_tuned_;
  double kp_hippitch_st_tuned_;
  double kp_knee_st_tuned_;
  double kp_toe_st_tuned_;
  double kp_shoulderroll_tuned_;
  double kp_shoulderpitch_tuned_;
  double kp_shoulderyaw_tuned_;
  double kp_elbow_tuned_;

  /************************ Log Variables */
  std::ofstream ctrl_logfile_;

  /************************ ROS Configuration Parameters ************************/
  // System Parameters
  double mass_;

  // Standing (Analytic) Parameters
  double standing_roll_des_analytic_;
  double standing_pitch_des_analytic_;
  double LA_des_analytic_;
  double LL_des_analytic_;

  // Standing (Numeric) Parameters
  double standing_com_height_des_;
  double step_width_standing_des_;

  // General Walking Parameters
  double time_step_period_;
  double kf_sample_time_;
  double zH_;
  double z_clearance_;
  double s_clearance_;
  double step_width_;
  double u_knee_comp_;
  double u_hiproll_comp_;
  std::string fp_type_;

  double mu_;      // terrain friction
  double kx_body_; // slope in body frame x axis (radians)
  double ky_body_; // " " y-axis (radians)

  // MPC Walking Parameters
  double dt_opt_mpc_;
  double N_step_horizon_mpc_;
  double ufp_x_max_;
  double ufp_y_min_;
  double ufp_y_max_;

  // Inverse Kinematics Parameters
  double scale_delta_ik_;
  int max_iter_ik_;
  double tol_ik_;
  double ik_threshold_;

  // General Target Parameters
  std::string target_type_;

  // Keyboard Targets (Standing) Parameters
  double x_offset_base_;
  double y_offset_base_;
  double z_offset_base_;
  double yaw_offset_base_;
  double pitch_offset_base_;
  double roll_offset_base_;

  double x_offset_inc_;
  double y_offset_inc_;
  double z_offset_inc_;
  double yaw_offset_inc_;
  double pitch_offset_inc_;
  double roll_offset_inc_;

  double x_offset_max_;
  double x_offset_min_;
  double y_offset_max_;
  double y_offset_min_;
  double z_offset_max_;
  double z_offset_min_;
  double yaw_offset_max_;
  double yaw_offset_min_;
  double pitch_offset_max_;
  double pitch_offset_min_;
  double roll_offset_max_;
  double roll_offset_min_;

  double standing_target_filter_param_;

  double time_sway_yaw_period_; // how long it takes to complete one trajectory
  double time_sway_pitch_period_;
  double time_sway_roll_period_;
  double time_sway_crouch_period_;
  double sway_yaw_max_; // limits of the sway motion
  double sway_pitch_max_;
  double sway_roll_max_;
  double sway_crouch_max_;
  double sway_crouch_min_;

  // Keyboard Targets (Walking) Parameters
  double vel_x_filter_param_;
  double vel_y_filter_param_;
  double turn_rps_filter_param_;
  
  double vel_x_des_base_;
  double vel_y_des_base_;
  double turn_rps_base_;

  double vel_x_des_inc_;
  double vel_y_des_inc_;
  double turn_rps_inc_;

  // Gamepad Targets (Walking) Parameters
  double vel_x_max_;
  double vel_y_max_;
  double turn_rps_max_;

  // Log Parameters
  std::string path_to_log_;
  std::string path_to_gains_;

  // Gain increment paramters
  double kp_hiproll_inc_;
  double kp_hipyaw_inc_;
  double kp_hippitch_inc_;
  double kp_knee_inc_;
  double kp_toe_inc_;
  double kp_shoulderroll_inc_;
  double kp_shoulderpitch_inc_;
  double kp_shoulderyaw_inc_;
  double kp_elbow_inc_;

  // Standing Gain Parameters
  double kp_hiproll_stand_base_;
  double kp_hipyaw_stand_base_;
  double kp_hippitch_stand_base_;
  double kp_knee_stand_base_;
  double kp_toe_stand_base_;
  double kp_shoulderroll_stand_base_;
  double kp_shoulderpitch_stand_base_;
  double kp_shoulderyaw_stand_base_;
  double kp_elbow_stand_base_;

  double kp_lateral_stand_base_;
  double kd_lateral_stand_base_;
  double kp_lateral_stand_inc_;
  double kd_lateral_stand_inc_;

  double kp_knee_comp_stand_base_;
  double kd_knee_comp_stand_base_;
  double kp_knee_comp_stand_inc_;
  double kd_knee_comp_stand_inc_;

  /* Walking Gains */
  double kp_hiproll_sw_base_;
  double kp_hipyaw_sw_base_;
  double kp_hippitch_sw_base_;
  double kp_knee_sw_base_;
  double kp_toe_sw_base_;
  double kp_hiproll_st_base_;
  double kp_hipyaw_st_base_;
  double kp_hippitch_st_base_;
  double kp_knee_st_base_;
  double kp_toe_st_base_;
  double kp_shoulderroll_base_;
  double kp_shoulderpitch_base_;
  double kp_shoulderyaw_base_;
  double kp_elbow_base_;

  // desired motions
  Matrix<double, NUM_MOTORS, 1> q_motors_des_copy;
  Matrix<double, NUM_MOTORS, 1> qdot_motors_des_copy;
  Matrix<double, constants::NUM_Q_ALL, 1> q_all_des_copy;
  Matrix<double, constants::NUM_Q_ALL, 1> qdot_all_des_copy;
};
