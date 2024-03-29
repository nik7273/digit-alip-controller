#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "Digit_Controller.hpp"
#include "lowlevelapi.h"

namespace py = pybind11;

PYBIND11_MODULE(digit_controller_pybind, m) {
    m.doc() = "pybind11 digit_controller_pybind plugin"; // optional module docstring

    // digit controller
    py::class_<Digit_Controller>(m, "Digit_Controller")
        .def(py::init<>())
        .def("Initialize_", &Digit_Controller::Initialize_)
        .def("Set_Initial_Standing_Gains_", &Digit_Controller::Set_Initial_Standing_Gains_)
        .def("Set_Initial_Walking_Gains_", &Digit_Controller::Set_Initial_Walking_Gains_)
        .def("Update_", &Digit_Controller::Update_)
        .def("Set_Ctrl_Mode_", &Digit_Controller::Set_Ctrl_Mode_)
        .def("Print_Standing_Gains_", &Digit_Controller::Print_Standing_Gains_)
        .def("Print_Walking_Gains_", &Digit_Controller::Print_Walking_Gains_)
        .def("Standing_Analytic_Update_", &Digit_Controller::Standing_Analytic_Update_)
        .def("Standing_Numeric_Update_", &Digit_Controller::Standing_Numeric_Update_)
        .def("Walking_Update_", &Digit_Controller::Walking_Update_)
        .def("Walking_Initialize_", &Digit_Controller::Walking_Initialize_)
        .def("Walking_Check_Stance_Switch_", &Digit_Controller::Walking_Check_Stance_Switch_)
        .def("Walking_Update_Variables_At_Impact_", &Digit_Controller::Walking_Update_Variables_At_Impact_)
        .def("Walking_Update_Variables_Each_Loop_", &Digit_Controller::Walking_Update_Variables_Each_Loop_)
        .def("Walking_Kalman_Filter_Angular_Momentum_", &Digit_Controller::Walking_Kalman_Filter_Angular_Momentum_)
        .def("Walking_Filter_Targets_", &Digit_Controller::Walking_Filter_Targets_)
        .def("Walking_Compute_1Step_Foot_Placement_", &Digit_Controller::Walking_Compute_1Step_Foot_Placement_)
        .def("Walking_Compute_IK_", &Digit_Controller::Walking_Compute_IK_)
        .def("Walking_Compute_Desired_Motors_", &Digit_Controller::Walking_Compute_Desired_Motors_)
        .def("Walking_Log_Data_", &Digit_Controller::Walking_Log_Data_)
        .def("Ramp_Torque_", &Digit_Controller::Ramp_Torque_)
        .def("Saturate_Torque_Command_", &Digit_Controller::Saturate_Torque_Command_)
        .def("Torque_Compensation_", &Digit_Controller::Torque_Compensation_)
        .def("Toe_Joints_To_Motors_", &Digit_Controller::Toe_Joints_To_Motors_)
        .def("Compute_GRF_", &Digit_Controller::Compute_GRF_)
        .def_readwrite("stance_switch", &Digit_Controller::stance_switch_)
        .def_readwrite("ctrl_mode", &Digit_Controller::ctrl_mode_)
	    .def_readwrite("torque_only", &Digit_Controller::torque_only_)
	    .def_readwrite("time_digit_prev", &Digit_Controller::time_digit_prev_)
	    .def_readwrite("contact_deflection", &Digit_Controller::contact_deflection_)
	    .def_readwrite("exp_mode", &Digit_Controller::exp_mode_)
	    .def_readwrite("op_mode", &Digit_Controller::op_mode_)
	    .def_readwrite("q_motors_min", &Digit_Controller::q_motors_min_)
	    .def_readwrite("q_motors_max", &Digit_Controller::q_motors_max_)
	    .def_readwrite("x_offset_filtered", &Digit_Controller::x_offset_filtered_)
	    .def_readwrite("y_offset_filtered", &Digit_Controller::y_offset_filtered_)
        .def_readwrite("yaw_offset_filtered_", &Digit_Controller::yaw_offset_filtered_)
        .def_readwrite("pitch_offset_filtered_", &Digit_Controller::pitch_offset_filtered_)
        .def_readwrite("roll_offset_filtered_", &Digit_Controller::roll_offset_filtered_)
        .def_readwrite("flag_standing_first_iter_", &Digit_Controller::flag_standing_first_iter_)
        .def_readwrite("yaw_init_", &Digit_Controller::yaw_init_)
        .def_readwrite("enable_sway_yaw_", &Digit_Controller::enable_sway_yaw_)
        .def_readwrite("enable_sway_pitch_", &Digit_Controller::enable_sway_pitch_)
        .def_readwrite("enable_sway_roll_", &Digit_Controller::enable_sway_roll_)
        .def_readwrite("enable_sway_crouch_", &Digit_Controller::enable_sway_crouch_)
        .def_readwrite("sway_yaw_start_", &Digit_Controller::sway_yaw_start_)
        .def_readwrite("sway_pitch_start_", &Digit_Controller::sway_pitch_start_)
        .def_readwrite("sway_roll_start_", &Digit_Controller::sway_roll_start_)
        .def_readwrite("sway_crouch_start_", &Digit_Controller::sway_crouch_start_)
        .def_readwrite("sway_yaw_dir_", &Digit_Controller::sway_yaw_dir_)
        .def_readwrite("sway_pitch_dir_", &Digit_Controller::sway_pitch_dir_)
        .def_readwrite("sway_roll_dir_", &Digit_Controller::sway_roll_dir_)
        .def_readwrite("sway_crouch_dir_", &Digit_Controller::sway_crouch_dir_)
        .def_readwrite("time_sway_yaw_start_", &Digit_Controller::time_sway_yaw_start_)
        .def_readwrite("time_sway_pitch_start_", &Digit_Controller::time_sway_pitch_start_)
        .def_readwrite("time_sway_roll_start_", &Digit_Controller::time_sway_roll_start_)
        .def_readwrite("time_sway_crouch_start_", &Digit_Controller::time_sway_crouch_start_)
        .def_readwrite("sway_yaw_init_", &Digit_Controller::sway_yaw_init_)
        .def_readwrite("sway_pitch_init_", &Digit_Controller::sway_pitch_init_)
        .def_readwrite("sway_roll_init_", &Digit_Controller::sway_roll_init_)
        .def_readwrite("sway_crouch_init_", &Digit_Controller::sway_crouch_init_)
        .def_readwrite("sway_yaw_final_", &Digit_Controller::sway_yaw_final_)
        .def_readwrite("sway_pitch_final_", &Digit_Controller::sway_pitch_final_)
        .def_readwrite("sway_roll_final_", &Digit_Controller::sway_roll_final_)
        .def_readwrite("sway_crouch_final_", &Digit_Controller::sway_crouch_final_)
        .def_readwrite("flag_walking_first_iter_", &Digit_Controller::flag_walking_first_iter_)
        .def_readwrite("stance_leg_", &Digit_Controller::stance_leg_)
        .def_readwrite("time_step_start_", &Digit_Controller::time_step_start_)
        .def_readwrite("p_sw_wrt_st_toe_bos_aligned_", &Digit_Controller::p_sw_wrt_st_toe_bos_aligned_)
        .def_readwrite("p_com_wrt_st_bos_aligned_", &Digit_Controller::p_com_wrt_st_bos_aligned_)
        .def_readwrite("mpc_iter_", &Digit_Controller::mpc_iter_)
        .def_readwrite("n_xlip_", &Digit_Controller::n_xlip_)
        .def_readwrite("n_ufp_", &Digit_Controller::n_ufp_)
        .def_readwrite("N_k_", &Digit_Controller::N_k_)
        .def_readwrite("N_fp_", &Digit_Controller::N_fp_)
        .def_readwrite("N_xsol_", &Digit_Controller::N_xsol_)
        .def_readwrite("N_ufpsol_", &Digit_Controller::N_ufpsol_)
        .def_readwrite("xlip_guess_prev_", &Digit_Controller::xlip_guess_prev_)
        .def_readwrite("ufp_guess_prev_", &Digit_Controller::ufp_guess_prev_)
        .def_readwrite("target_heading_", &Digit_Controller::target_heading_)
        .def_readwrite("st_heading_bos_", &Digit_Controller::st_heading_bos_)
        .def_readwrite("st_heading_bos_mat_", &Digit_Controller::st_heading_bos_mat_)
        .def_readwrite("Rz_st_heading_bos_", &Digit_Controller::Rz_st_heading_bos_)
        .def_readwrite("Lx_st_kf_aligned_", &Digit_Controller::Lx_st_kf_aligned_)
        .def_readwrite("Ly_st_kf_aligned_", &Digit_Controller::Ly_st_kf_aligned_)
        .def_readwrite("cov_Lx_st_kf_aligned_", &Digit_Controller::cov_Lx_st_kf_aligned_)
        .def_readwrite("cov_Ly_st_kf_aligned_", &Digit_Controller::cov_Ly_st_kf_aligned_)
        .def_readwrite("L_left_toe_obs_aligned_", &Digit_Controller::L_left_toe_obs_aligned_)
        .def_readwrite("L_right_toe_obs_aligned_", &Digit_Controller::L_right_toe_obs_aligned_)
        .def_readwrite("idx_q_st_hiproll_", &Digit_Controller::idx_q_st_hiproll_)
        .def_readwrite("idx_q_st_hipyaw_", &Digit_Controller::idx_q_st_hipyaw_)
        .def_readwrite("idx_q_st_hippitch_", &Digit_Controller::idx_q_st_hippitch_)
        .def_readwrite("idx_q_st_knee_", &Digit_Controller::idx_q_st_knee_)
        .def_readwrite("idx_q_st_KneeToShin_", &Digit_Controller::idx_q_st_KneeToShin_)
        .def_readwrite("idx_q_st_ShinToTarsus_", &Digit_Controller::idx_q_st_ShinToTarsus_)
        .def_readwrite("idx_q_sw_hiproll_", &Digit_Controller::idx_q_sw_hiproll_)
        .def_readwrite("idx_q_sw_hipyaw_", &Digit_Controller::idx_q_sw_hipyaw_)
        .def_readwrite("idx_q_sw_hippitch_", &Digit_Controller::idx_q_sw_hippitch_)
        .def_readwrite("idx_q_sw_knee_", &Digit_Controller::idx_q_sw_knee_)
        .def_readwrite("idx_q_sw_KneeToShin_", &Digit_Controller::idx_q_sw_KneeToShin_)
        .def_readwrite("idx_q_sw_ShinToTarsus_", &Digit_Controller::idx_q_sw_ShinToTarsus_)
        .def_readwrite("idx_m_st_hiproll_", &Digit_Controller::idx_m_st_hiproll_)
        .def_readwrite("idx_m_st_hipyaw_", &Digit_Controller::idx_m_st_hipyaw_)
        .def_readwrite("idx_m_st_hippitch_", &Digit_Controller::idx_m_st_hippitch_)
        .def_readwrite("idx_m_st_knee_", &Digit_Controller::idx_m_st_knee_)
        .def_readwrite("idx_m_st_ShinToTarsus_", &Digit_Controller::idx_m_st_ShinToTarsus_)
        .def_readwrite("idx_m_sw_hiproll_", &Digit_Controller::idx_m_sw_hiproll_)
        .def_readwrite("idx_m_sw_hipyaw_", &Digit_Controller::idx_m_sw_hipyaw_)
        .def_readwrite("idx_m_sw_hippitch_", &Digit_Controller::idx_m_sw_hippitch_)
        .def_readwrite("idx_m_sw_knee_", &Digit_Controller::idx_m_sw_knee_)
        .def_readwrite("idx_m_sw_ShinToTarsus_", &Digit_Controller::idx_m_sw_ShinToTarsus_)
        .def_readwrite("shift_lateral_", &Digit_Controller::shift_lateral_)
        .def_readwrite("vel_x_des_tuned_", &Digit_Controller::vel_x_des_tuned_)
        .def_readwrite("vel_y_des_tuned_", &Digit_Controller::vel_y_des_tuned_)
        .def_readwrite("turn_rps_tuned_", &Digit_Controller::turn_rps_tuned_)
        .def_readwrite("vel_x_des_filtered_", &Digit_Controller::vel_x_des_filtered_)
        .def_readwrite("vel_y_des_filtered_", &Digit_Controller::vel_y_des_filtered_)
        .def_readwrite("turn_rps_filtered_", &Digit_Controller::turn_rps_filtered_)
        .def_readwrite("x_offset_tuned_", &Digit_Controller::x_offset_tuned_)
        .def_readwrite("y_offset_tuned_", &Digit_Controller::y_offset_tuned_)
        .def_readwrite("z_offset_tuned_", &Digit_Controller::z_offset_tuned_)
        .def_readwrite("yaw_offset_tuned_", &Digit_Controller::yaw_offset_tuned_)
        .def_readwrite("pitch_offset_tuned_", &Digit_Controller::pitch_offset_tuned_)
        .def_readwrite("roll_offset_tuned_", &Digit_Controller::roll_offset_tuned_)
        .def_readwrite("kp_stand_", &Digit_Controller::kp_stand_)
        .def_readwrite("kp_right_st_", &Digit_Controller::kp_right_st_)
        .def_readwrite("kp_left_st_", &Digit_Controller::kp_left_st_)
        .def_readwrite("kp_hiproll_stand_tuned_", &Digit_Controller::kp_hiproll_stand_tuned_)
        .def_readwrite("kp_hipyaw_stand_tuned_", &Digit_Controller::kp_hipyaw_stand_tuned_)
        .def_readwrite("kp_hippitch_stand_tuned_", &Digit_Controller::kp_hippitch_stand_tuned_)
        .def_readwrite("kp_knee_stand_tuned_", &Digit_Controller::kp_knee_stand_tuned_)
        .def_readwrite("kp_toe_stand_tuned_", &Digit_Controller::kp_toe_stand_tuned_)
        .def_readwrite("kp_shoulderroll_stand_tuned_", &Digit_Controller::kp_shoulderroll_stand_tuned_)
        .def_readwrite("kp_shoulderpitch_stand_tuned_", &Digit_Controller::kp_shoulderpitch_stand_tuned_)
        .def_readwrite("kp_shoulderyaw_stand_tuned_", &Digit_Controller::kp_shoulderyaw_stand_tuned_)
        .def_readwrite("kp_elbow_stand_tuned_", &Digit_Controller::kp_elbow_stand_tuned_)
        .def_readwrite("kp_lateral_stand_tuned_", &Digit_Controller::kp_lateral_stand_tuned_)
        .def_readwrite("kd_lateral_stand_tuned_", &Digit_Controller::kd_lateral_stand_tuned_)
        .def_readwrite("kp_knee_comp_stand_tuned_", &Digit_Controller::kp_knee_comp_stand_tuned_)
        .def_readwrite("kd_knee_comp_stand_tuned_", &Digit_Controller::kd_knee_comp_stand_tuned_)
        .def_readwrite("kp_hiproll_sw_tuned_", &Digit_Controller::kp_hiproll_sw_tuned_)
        .def_readwrite("kp_hipyaw_sw_tuned_", &Digit_Controller::kp_hipyaw_sw_tuned_)
        .def_readwrite("kp_hippitch_sw_tuned_", &Digit_Controller::kp_hippitch_sw_tuned_)
        .def_readwrite("kp_knee_sw_tuned_", &Digit_Controller::kp_knee_sw_tuned_)
        .def_readwrite("kp_toe_sw_tuned_", &Digit_Controller::kp_toe_sw_tuned_)
        .def_readwrite("kp_hiproll_st_tuned_", &Digit_Controller::kp_hiproll_st_tuned_)
        .def_readwrite("kp_hipyaw_st_tuned_", &Digit_Controller::kp_hipyaw_st_tuned_)
        .def_readwrite("kp_hippitch_st_tuned_", &Digit_Controller::kp_hippitch_st_tuned_)
        .def_readwrite("kp_knee_st_tuned_", &Digit_Controller::kp_knee_st_tuned_)
        .def_readwrite("kp_toe_st_tuned_", &Digit_Controller::kp_toe_st_tuned_)
        .def_readwrite("kp_shoulderroll_tuned_", &Digit_Controller::kp_shoulderroll_tuned_)
        .def_readwrite("kp_shoulderpitch_tuned_", &Digit_Controller::kp_shoulderpitch_tuned_)
        .def_readwrite("kp_shoulderyaw_tuned_", &Digit_Controller::kp_shoulderyaw_tuned_)
        .def_readwrite("kp_elbow_tuned_", &Digit_Controller::kp_elbow_tuned_)
	    .def_readwrite("mass_", &Digit_Controller::mass_)
     	.def_readwrite("standing_roll_des_analytic_", &Digit_Controller::standing_roll_des_analytic_)
  	    .def_readwrite("standing_pitch_des_analytic_", &Digit_Controller::standing_pitch_des_analytic_)
	    .def_readwrite("LA_des_analytic_", &Digit_Controller::LA_des_analytic_)
	    .def_readwrite("LL_des_analytic_", &Digit_Controller::LL_des_analytic_)
	    .def_readwrite("standing_com_height_des_", &Digit_Controller::standing_com_height_des_)
	    .def_readwrite("step_width_standing_des_", &Digit_Controller::step_width_standing_des_)
	    .def_readwrite("time_step_period_", &Digit_Controller::time_step_period_)
	    .def_readwrite("kf_sample_time_", &Digit_Controller::kf_sample_time_)
	    .def_readwrite("zH_", &Digit_Controller::zH_)
	    .def_readwrite("z_clearance_", &Digit_Controller::z_clearance_)
	    .def_readwrite("s_clearance_", &Digit_Controller::s_clearance_)
	    .def_readwrite("step_width_", &Digit_Controller::step_width_)
	    .def_readwrite("u_knee_comp_", &Digit_Controller::u_knee_comp_)
	    .def_readwrite("u_hiproll_comp_", &Digit_Controller::u_hiproll_comp_)
	    .def_readwrite("fp_type_", &Digit_Controller::fp_type_)
	    .def_readwrite("mu_", &Digit_Controller::mu_)
	    .def_readwrite("kx_body_", &Digit_Controller::kx_body_)
	    .def_readwrite("ky_body_", &Digit_Controller::ky_body_)
	    .def_readwrite("dt_opt_mpc_", &Digit_Controller::dt_opt_mpc_)
	    .def_readwrite("N_step_horizon_mpc_", &Digit_Controller::N_step_horizon_mpc_)
	    .def_readwrite("ufp_x_max_", &Digit_Controller::ufp_x_max_)
	    .def_readwrite("ufp_y_min_", &Digit_Controller::ufp_y_min_)
	    .def_readwrite("ufp_y_max_", &Digit_Controller::ufp_y_max_)
	    .def_readwrite("scale_delta_ik_", &Digit_Controller::scale_delta_ik_)
	    .def_readwrite("max_iter_ik_", &Digit_Controller::max_iter_ik_)
	    .def_readwrite("tol_ik_", &Digit_Controller::tol_ik_)
	    .def_readwrite("ik_threshold_", &Digit_Controller::ik_threshold_)
	    .def_readwrite("target_type_", &Digit_Controller::target_type_)
	    .def_readwrite("x_offset_base_", &Digit_Controller::x_offset_base_)
	    .def_readwrite("y_offset_base_", &Digit_Controller::y_offset_base_)
	    .def_readwrite("z_offset_base_", &Digit_Controller::z_offset_base_)
	    .def_readwrite("yaw_offset_base_", &Digit_Controller::yaw_offset_base_)
	    .def_readwrite("pitch_offset_base_", &Digit_Controller::pitch_offset_base_)
	    .def_readwrite("roll_offset_base_", &Digit_Controller::roll_offset_base_)
	    .def_readwrite("x_offset_inc_", &Digit_Controller::x_offset_inc_)
	    .def_readwrite("y_offset_inc_", &Digit_Controller::y_offset_inc_)
	    .def_readwrite("z_offset_inc_", &Digit_Controller::z_offset_inc_)
	    .def_readwrite("yaw_offset_inc_", &Digit_Controller::yaw_offset_inc_)
	    .def_readwrite("pitch_offset_inc_", &Digit_Controller::pitch_offset_inc_)
	    .def_readwrite("roll_offset_inc_", &Digit_Controller::roll_offset_inc_)
	    .def_readwrite("x_offset_max_", &Digit_Controller::x_offset_max_)
	    .def_readwrite("x_offset_min_", &Digit_Controller::x_offset_min_)
	    .def_readwrite("y_offset_max_", &Digit_Controller::y_offset_max_)
	    .def_readwrite("y_offset_min_", &Digit_Controller::y_offset_min_)
	    .def_readwrite("z_offset_max_", &Digit_Controller::z_offset_max_)
	    .def_readwrite("z_offset_min_", &Digit_Controller::z_offset_min_)
	    .def_readwrite("yaw_offset_max_", &Digit_Controller::yaw_offset_max_)
	    .def_readwrite("yaw_offset_min_", &Digit_Controller::yaw_offset_min_)
	    .def_readwrite("pitch_offset_max_", &Digit_Controller::pitch_offset_max_)
	    .def_readwrite("pitch_offset_min_", &Digit_Controller::pitch_offset_min_)
	    .def_readwrite("roll_offset_max_", &Digit_Controller::roll_offset_max_)
	    .def_readwrite("roll_offset_min_", &Digit_Controller::roll_offset_min_)
	    .def_readwrite("standing_target_filter_param_", &Digit_Controller::standing_target_filter_param_)
	    .def_readwrite("time_sway_yaw_period_", &Digit_Controller::time_sway_yaw_period_)
	    .def_readwrite("time_sway_pitch_period_", &Digit_Controller::time_sway_pitch_period_)
	    .def_readwrite("time_sway_roll_period_", &Digit_Controller::time_sway_roll_period_)
	    .def_readwrite("time_sway_crouch_period_", &Digit_Controller::time_sway_crouch_period_)
	    .def_readwrite("sway_yaw_max_", &Digit_Controller::sway_yaw_max_)
	    .def_readwrite("sway_pitch_max_", &Digit_Controller::sway_pitch_max_)
	    .def_readwrite("sway_roll_max_", &Digit_Controller::sway_roll_max_)
	    .def_readwrite("sway_crouch_max_", &Digit_Controller::sway_crouch_max_)
	    .def_readwrite("sway_crouch_min_", &Digit_Controller::sway_crouch_min_)
	    .def_readwrite("vel_x_filter_param_", &Digit_Controller::vel_x_filter_param_)
	    .def_readwrite("vel_y_filter_param_", &Digit_Controller::vel_y_filter_param_)
	    .def_readwrite("turn_rps_filter_param_", &Digit_Controller::turn_rps_filter_param_)
	    .def_readwrite("vel_x_des_base_", &Digit_Controller::vel_x_des_base_)
	    .def_readwrite("vel_y_des_base_", &Digit_Controller::vel_y_des_base_)
	    .def_readwrite("turn_rps_base_", &Digit_Controller::turn_rps_base_)
	    .def_readwrite("vel_x_des_inc_", &Digit_Controller::vel_x_des_inc_)
	    .def_readwrite("vel_y_des_inc_", &Digit_Controller::vel_y_des_inc_)
	    .def_readwrite("turn_rps_inc_", &Digit_Controller::turn_rps_inc_)
	    .def_readwrite("vel_x_max_", &Digit_Controller::vel_x_max_)
	    .def_readwrite("vel_y_max_", &Digit_Controller::vel_y_max_)
	    .def_readwrite("turn_rps_max_", &Digit_Controller::turn_rps_max_)
	    .def_readwrite("path_to_log_", &Digit_Controller::path_to_log_)
	    .def_readwrite("path_to_gains_", &Digit_Controller::path_to_gains_)
	    .def_readwrite("kp_hiproll_inc_", &Digit_Controller::kp_hiproll_inc_)
	    .def_readwrite("kp_hipyaw_inc_", &Digit_Controller::kp_hipyaw_inc_)
	    .def_readwrite("kp_hippitch_inc_", &Digit_Controller::kp_hippitch_inc_)
	    .def_readwrite("kp_knee_inc_", &Digit_Controller::kp_knee_inc_)
	    .def_readwrite("kp_toe_inc_", &Digit_Controller::kp_toe_inc_)
	    .def_readwrite("kp_shoulderroll_inc_", &Digit_Controller::kp_shoulderroll_inc_)
	    .def_readwrite("kp_shoulderpitch_inc_", &Digit_Controller::kp_shoulderpitch_inc_)
	    .def_readwrite("kp_shoulderyaw_inc_", &Digit_Controller::kp_shoulderyaw_inc_)
	    .def_readwrite("kp_elbow_inc_", &Digit_Controller::kp_elbow_inc_)
	    .def_readwrite("kp_hiproll_stand_base_", &Digit_Controller::kp_hiproll_stand_base_)
	    .def_readwrite("kp_hipyaw_stand_base_", &Digit_Controller::kp_hipyaw_stand_base_)
	    .def_readwrite("kp_hippitch_stand_base_", &Digit_Controller::kp_hippitch_stand_base_)
	    .def_readwrite("kp_knee_stand_base_", &Digit_Controller::kp_knee_stand_base_)
	    .def_readwrite("kp_toe_stand_base_", &Digit_Controller::kp_toe_stand_base_)
	    .def_readwrite("kp_shoulderroll_stand_base_", &Digit_Controller::kp_shoulderroll_stand_base_)
	    .def_readwrite("kp_shoulderpitch_stand_base_", &Digit_Controller::kp_shoulderpitch_stand_base_)
	    .def_readwrite("kp_shoulderyaw_stand_base_", &Digit_Controller::kp_shoulderyaw_stand_base_)
	    .def_readwrite("kp_elbow_stand_base_", &Digit_Controller::kp_elbow_stand_base_)
	    .def_readwrite("kp_lateral_stand_base_", &Digit_Controller::kp_lateral_stand_base_)
	    .def_readwrite("kd_lateral_stand_base_", &Digit_Controller::kd_lateral_stand_base_)
	    .def_readwrite("kp_lateral_stand_inc_", &Digit_Controller::kp_lateral_stand_inc_)
	    .def_readwrite("kd_lateral_stand_inc_", &Digit_Controller::kd_lateral_stand_inc_)
	    .def_readwrite("kp_knee_comp_stand_base_", &Digit_Controller::kp_knee_comp_stand_base_)
	    .def_readwrite("kd_knee_comp_stand_base_", &Digit_Controller::kd_knee_comp_stand_base_)
	    .def_readwrite("kp_knee_comp_stand_inc_", &Digit_Controller::kp_knee_comp_stand_inc_)
	    .def_readwrite("kd_knee_comp_stand_inc_", &Digit_Controller::kd_knee_comp_stand_inc_)
	    .def_readwrite("kp_hiproll_sw_base_", &Digit_Controller::kp_hiproll_sw_base_)
	    .def_readwrite("kp_hipyaw_sw_base_", &Digit_Controller::kp_hipyaw_sw_base_)
	    .def_readwrite("kp_hippitch_sw_base_", &Digit_Controller::kp_hippitch_sw_base_)
	    .def_readwrite("kp_knee_sw_base_", &Digit_Controller::kp_knee_sw_base_)
	    .def_readwrite("kp_toe_sw_base_", &Digit_Controller::kp_toe_sw_base_)
	    .def_readwrite("kp_hiproll_st_base_", &Digit_Controller::kp_hiproll_st_base_)
	    .def_readwrite("kp_hipyaw_st_base_", &Digit_Controller::kp_hipyaw_st_base_)
	    .def_readwrite("kp_hippitch_st_base_", &Digit_Controller::kp_hippitch_st_base_)
	    .def_readwrite("kp_knee_st_base_", &Digit_Controller::kp_knee_st_base_)
	    .def_readwrite("kp_toe_st_base_", &Digit_Controller::kp_toe_st_base_)
	    .def_readwrite("kp_shoulderroll_base_", &Digit_Controller::kp_shoulderroll_base_)
	    .def_readwrite("kp_shoulderpitch_base_", &Digit_Controller::kp_shoulderpitch_base_)
	    .def_readwrite("kp_shoulderyaw_base_", &Digit_Controller::kp_shoulderyaw_base_)
	    .def_readwrite("kp_elbow_base_", &Digit_Controller::kp_elbow_base_)
	    .def_readwrite("q_motors_des_copy_", &Digit_Controller::q_motors_des_copy)
	    .def_readwrite("qdot_motors_des_copy_", &Digit_Controller::qdot_motors_des_copy)
	    .def_readwrite("q_all_des_copy_", &Digit_Controller::q_all_des_copy)
	    .def_readwrite("qdot_all_des_copy_", &Digit_Controller::qdot_all_des_copy);

    // agility llapi types
    py::class_<llapi_quaternion_t>(m, "Quaternion")
        .def(py::init<>())
        .def_readwrite("w", &llapi_quaternion_t::w)
        .def_readwrite("x", &llapi_quaternion_t::x)
        .def_readwrite("y", &llapi_quaternion_t::y)
        .def_readwrite("z", &llapi_quaternion_t::z);

    py::class_<llapi_observation_t>(m, "Observation")
        .def(py::init<>())
        .def_readwrite("time", &llapi_observation_t::time)
        .def_readwrite("error", &llapi_observation_t::error)
        .def_readwrite("base", &llapi_observation_t::base)
        .def_readwrite("imu", &llapi_observation_t::imu)
        .def_readwrite("motor", &llapi_observation_t::motor)
        .def_readwrite("joint", &llapi_observation_t::joint)
        .def_readwrite("battery_charge", &llapi_observation_t::battery_charge);

    // llapi_observation_t subtypes
    py::class_<llapi_observation_t::Base>(m, "ObsBase")
        .def(py::init<>())
        .def_readwrite("orientation", &llapi_observation_t::Base::orientation)
        .def_property("angular_velocity", [](const llapi_observation_t::Base &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {3}, {sizeof(double)});
            return py::array(dtype, {3}, {sizeof(double)}, self.angular_velocity, base);
        }, [](llapi_observation_t::Base &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.angular_velocity, arr.data(), 3 * sizeof(double));
        })
        .def_property("linear_velocity", [](const llapi_observation_t::Base &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {3}, {sizeof(double)});
            return py::array(dtype, {3}, {sizeof(double)}, self.linear_velocity, base);
        }, [](llapi_observation_t::Base &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.linear_velocity, arr.data(), 3 * sizeof(double));
        })
        .def_property("translation", [](const llapi_observation_t::Base &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {3}, {sizeof(double)});
            return py::array(dtype, {3}, {sizeof(double)}, self.translation, base);
        }, [](llapi_observation_t::Base &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.translation, arr.data(), 3 * sizeof(double));
        });

    py::class_<llapi_observation_t::IMU>(m, "ObsIMU")
        .def(py::init<>())
        .def_readwrite("orientation", &llapi_observation_t::IMU::orientation)
        .def_property("angular_velocity", [](const llapi_observation_t::IMU &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {3}, {sizeof(double)});
            return py::array(dtype, {3}, {sizeof(double)}, self.angular_velocity, base);
        }, [](llapi_observation_t::IMU &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.angular_velocity, arr.data(), 3 * sizeof(double));
        })
        .def_property("linear_acceleration", [](const llapi_observation_t::IMU &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {3}, {sizeof(double)});
            return py::array(dtype, {3}, {sizeof(double)}, self.linear_acceleration, base);
        }, [](llapi_observation_t::IMU &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.linear_acceleration, arr.data(), 3 * sizeof(double));
        })
        .def_property("magnetic_field", [](const llapi_observation_t::IMU &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {3}, {sizeof(double)});
            return py::array(dtype, {3}, {sizeof(double)}, self.magnetic_field, base);
        }, [](llapi_observation_t::IMU &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.magnetic_field, arr.data(), 3 * sizeof(double));
        });

    py::class_<llapi_observation_t::Motor>(m, "ObsMotor")
        .def(py::init<>())
        .def_property("position", [](const llapi_observation_t::Motor &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.position, base);
        }, [](llapi_observation_t::Motor &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.position, arr.data(), NUM_MOTORS * sizeof(double));
        })
        .def_property("velocity", [](const llapi_observation_t::Motor &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.velocity, base);
        }, [](llapi_observation_t::Motor &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.velocity, arr.data(), NUM_MOTORS * sizeof(double));
        })
        .def_property("torque", [](const llapi_observation_t::Motor &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.torque, base);
        }, [](llapi_observation_t::Motor &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.torque, arr.data(), NUM_MOTORS * sizeof(double));
        });

    py::class_<llapi_observation_t::Joint>(m, "ObsUnactJoints")
        .def(py::init<>())
        .def_property("position", [](const llapi_observation_t::Joint &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_UNACT_JOINTS}, {sizeof(double)});
            return py::array(dtype, {NUM_UNACT_JOINTS}, {sizeof(double)}, self.position, base);
        }, [](llapi_observation_t::Joint &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.position, arr.data(), NUM_UNACT_JOINTS * sizeof(double));
        })
        .def_property("velocity", [](const llapi_observation_t::Joint &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_UNACT_JOINTS}, {sizeof(double)});
            return py::array(dtype, {NUM_UNACT_JOINTS}, {sizeof(double)}, self.velocity, base);
        }, [](llapi_observation_t::Joint &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.velocity, arr.data(), NUM_UNACT_JOINTS * sizeof(double));
        });

    py::class_<llapi_limits_t>(m, "Limits")
        .def(py::init<>())
        .def_property("torque_limit", [](const llapi_limits_t &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.torque_limit, base);
        }, [](llapi_limits_t &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.torque_limit, arr.data(), NUM_MOTORS * sizeof(double));
        })
        .def_property("damping_limit", [](const llapi_limits_t &self) -> py::array {            
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.damping_limit, base);
        }, [](llapi_limits_t &self, py::array_t<double> arr) {
            std::memcpy(self.damping_limit, arr.data(), NUM_MOTORS * sizeof(double));
        })
        .def_property("velocity_limit", [](const llapi_limits_t &self) -> py::array {
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.velocity_limit, base);
        }, [](llapi_limits_t &self, py::array_t<double> arr) {
            std::memcpy(self.velocity_limit, arr.data(), NUM_MOTORS * sizeof(double));
        });

    py::class_<llapi_motor_t>(m, "Motor")
        .def(py::init<>())
        .def_readwrite("torque", &llapi_motor_t::torque)
        .def_readwrite("velocity", &llapi_motor_t::velocity)
        .def_readwrite("damping", &llapi_motor_t::damping);

    PYBIND11_NUMPY_DTYPE(llapi_motor_t, torque, velocity, damping);
    py::dtype dt = py::dtype::of<llapi_motor_t>();
    m.attr("llapi_motor_dtype") = dt;

    py::class_<llapi_command_t>(m, "Command")
        .def(py::init<>())
        .def_property("motors", [](const llapi_command_t &self) -> py::array_t<llapi_motor_t> {
            auto dtype = py::dtype::of<llapi_motor_t>();
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(llapi_motor_t)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(llapi_motor_t)}, self.motors, base);
        }, [](llapi_command_t &self, py::array_t<llapi_motor_t> arr) {
            std::memcpy(self.motors, arr.data(), NUM_MOTORS * sizeof(llapi_motor_t));
        })
        .def_readwrite("fallback_opmode", &llapi_command_t::fallback_opmode)
        .def_readwrite("apply_command", &llapi_command_t::apply_command);
}
