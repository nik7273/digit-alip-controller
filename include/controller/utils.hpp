#pragma once

#include <iostream>
#include <math.h>

#include "eigen340/Eigen/Dense"


#include "lowlevelapi.h"

using namespace Eigen;

namespace constants
{
    /* Size Constants */
    const int NUM_BASE = 6;
    const int NUM_PASSIVE_JOINTS = NUM_UNACT_JOINTS; //
    const int NUM_ACTUATOR_JOINTS = NUM_MOTORS;
    const int NUM_Q_LEG = 8; // includes knee_to_shin joint which has high stiffness
    const int NUM_Q_ARM = 4;
    const int NUM_Q_BODY = 2 * NUM_Q_LEG + 2 * NUM_Q_ARM;
    const int NUM_Q_ALL = NUM_BASE + NUM_Q_BODY;

    const int NUM_VIRT_CONSTR = 20; // 
    const int NUM_HOL_CONSTR = 10;
    const double PI = 3.141592653589793238;
}

/* Rotation & Transformation Functions */
namespace transform_utils
{
    void Quaternion_To_Euler_And_Rates(Ref<Vector3d> euler_ang,
                                       Ref<Vector3d> euler_angvel,
                                       const Ref<const Vector3d> &w,
                                       Quaterniond quat); // pass by value on purpose to normalize
    void Rotation_Matrix_To_Euler_Angles(Ref<Vector3d> euler_ang,
                                         const Ref<const Matrix3d> &R_mat);

    void Rotation_Matrix_And_Angular_Velocity_To_Euler_Rates(Ref<Vector3d> euler_rate,
                                                             const Ref<const Matrix3d> &R_mat,
                                                             const Ref<const Vector3d> &w_vect);

    void Euler_Yaw_Angle_To_Rotation_Matrix(Ref<Matrix3d> Rz,
                                            const double &yaw);
    void Wrap_To_Pi(double &wrapped_angle,
                    const double &raw_angle);
}
namespace bezier_utils
{
    /* Bezier function w.r.t phase variable s */
    double Bezier(const double &s,
                  Ref<VectorXd> coeff);

    /* Bezier partial derivative with respect to phase variable s */
    double Bezier_Partial_Derivative(const double &s,
                   Ref<VectorXd> coeff);

    /* Coefficients of bezier partial derivative w.r.t s */
    VectorXd Bezier_Partial_Derivative_Coeffs(Ref<VectorXd> coeff);

    /* General Factorial function */
    double Factorial(double n);

    /* Coefficients for smooth trajectory from given initial and final position */
    void Smooth_Bezier_Coefficients(const double &qinit,
                                    const double &qfinal,
                                    const int &bezdeg,
                                    Ref<VectorXd> bez_coeffs);
} // bezier
