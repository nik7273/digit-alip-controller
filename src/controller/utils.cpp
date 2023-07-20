#include "utils.hpp"

using namespace Eigen;
namespace transform_utils
{
    /* Rotation & Transformation Functions */
    void Quaternion_To_Euler_And_Rates(Ref<Vector3d> euler_ang,
                                       Ref<Vector3d> euler_angvel,
                                       const Ref<const Vector3d> &w,
                                       Quaterniond quat) // pass by value on purpose to normalize
    {
        quat.normalize();
        Matrix3d R = quat.toRotationMatrix(); // R.array()
        Rotation_Matrix_To_Euler_Angles(euler_ang, R);
        Rotation_Matrix_And_Angular_Velocity_To_Euler_Rates(euler_angvel, R, w);

        // test
        // double th = PI/4.0;
        // double wwt = cos(th);
        // double wxt = 0.5*sin(th);
        // double wyt = (sqrt(3)/2)*sin(th);
        // double wzt = 0;
        // Quaterniond qt(wwt,wxt,wyt,wzt);
        // qt.normalize();
        // Matrix3d Rt = qt.toRotationMatrix();
        // Vector3d wt(1.0,2.0,3.0);

        // Vector3d eulerang_t;
        // Vector3d eulerrate_t;
        // Rotation_To_EulerAng(eulerang_t,Rt);
        // RotationAngVel_To_EulerRate(eulerrate_t,Rt,wt);

        // std::cout << "q: \n" << qt.w() << "\n" << qt.vec() << std::endl;
        // std::cout << "w: \n" << wt << std::endl;
        // std::cout << "R: \n" << Rt.array() << std::endl;
        // std::cout << "eul: \n" << eulerang_t << std::endl;
        // std::cout << "EulRate: \n" << eulerrate_t << std:: endl;

        return;
    }

    void Rotation_Matrix_To_Euler_Angles(Ref<Vector3d> euler_ang,
                                         const Ref<const Matrix3d> &R_mat)
    {
        // Copied from generated matlab function in model directory
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        // Assign elements with column-major order
        double R1 = R_mat(0, 0);
        double R2 = R_mat(1, 0);
        double R3 = R_mat(2, 0);
        // double R4 = R_mat(0,1);
        // double R5 = R_mat(1,1);
        double R6 = R_mat(2, 1);
        // double R7 = R_mat(0,2);
        // double R8 = R_mat(1,2);
        double R9 = R_mat(2, 2);

        double yaw = atan2(R2, R1);
        double pitch = -asin(R3);
        double roll = atan2(R6, R9);
        euler_ang << yaw, pitch, roll;
        return;
    }

    void Rotation_Matrix_And_Angular_Velocity_To_Euler_Rates(Ref<Vector3d> euler_rate,
                                                             const Ref<const Matrix3d> &R_mat,
                                                             const Ref<const Vector3d> &w_vect)
    {
        // Copied from generated matlab function in model directory
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        // Assign elements with column-major order
        double R1 = R_mat(0, 0);
        double R2 = R_mat(1, 0);
        double R3 = R_mat(2, 0);
        double R4 = R_mat(0, 1);
        double R5 = R_mat(1, 1);
        double R6 = R_mat(2, 1);
        double R7 = R_mat(0, 2);
        double R8 = R_mat(1, 2);
        double R9 = R_mat(2, 2);

        // Angular velocities w.r.t IMU (body) frame
        double w1 = w_vect[0];
        double w2 = w_vect[1];
        double w3 = w_vect[2];

        // Compute
        double t2 = pow(R1, 2);
        double t3 = pow(R2, 2);
        double t4 = pow(R6, 2);
        double t5 = pow(R9, 2);
        double t6 = t2 + t3;
        double t7 = 1.0 / t6;
        double yaw_dot = -R2 * t7 * (R4 * w3 - R7 * w2) + R1 * t7 * (R5 * w3 - R8 * w2); //-R2*t7*(R4*w3-R7*w2)+R1*t7*(R5*w3-R8*w2);;
        double pitch_dot = -1.0 / sqrt(-pow(R3, 2) + 1.0) * (R6 * w3 - R9 * w2);
        double roll_dot = (t4 * w1 + t5 * w1 - R3 * R6 * w2 - R3 * R9 * w3) / (t4 + t5);
        euler_rate << yaw_dot, pitch_dot, roll_dot;
        return;
    }

    void Euler_Yaw_Angle_To_Rotation_Matrix(Ref<Matrix3d> Rz,
                                            const double &yaw)
    {
        Rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
        return;
    }

    void Wrap_To_Pi(double &wrapped_angle,
                    const double &raw_angle)
    {
        if (raw_angle > 0)
        {
            int num_rotations = static_cast<int>((raw_angle + M_PI) / (2 * M_PI));
            wrapped_angle = raw_angle - 2.0 * M_PI * num_rotations;
        }
        else
        {
            int num_rotations = static_cast<int>((raw_angle - M_PI) / (2.0 * M_PI));
            wrapped_angle = raw_angle - 2.0 * M_PI * num_rotations;
        }
        return;
    }
} // transformation_utils

namespace bezier_utils
{
    /* Bezier Functions */
    double Bezier(const double &s,
                  Ref<VectorXd> coeff)
    {
        double bez = 0.0;
        double M = coeff.size() - 1;
        double M_Fact = Factorial(M); // M! where M = NUM_BEZ_DEG
        for (int kint = 0; kint < M + 1; kint++)
        {
            double k = double(kint);
            bez = bez + coeff[kint] * (M_Fact / (Factorial(k) * Factorial(M - k))) * pow(s, k) * pow(1.0 - s, M - k);
        }
        return bez;
    }
    double Bezier_Partial_Derivative(const double &s,
                                     Ref<VectorXd> coeff)
    {
        // Partial derivative w.r.t phase variable s. To get time derivative need to still multiply by sdot
        double dbez = 0.0;
        double M = coeff.size() - 1;
        double M_Fact = Factorial(M);        // M! where M = NUM_BEZ_DEG
        for (int kint = 0; kint < M; kint++) // sum from k = 0 to M-1
        {
            double k = double(kint);
            dbez = dbez + (coeff[kint + 1] - coeff[kint]) * (M_Fact / (Factorial(k) * Factorial(M - k - 1.0))) * pow(s, k) * pow(1.0 - s, M - k - 1.0);
        }
        return dbez;
    }
    VectorXd Bezier_Partial_Derivative_Coeffs(Ref<VectorXd> coeff)
    {
        double M = coeff.size() - 1;
        // std::cout << "coeff:\n" << coeff << "\n";
        VectorXd dbez_coeffs;
        dbez_coeffs.setZero(M);
        // std::cout << "init dbez coeff: \n" << dbez_coeffs << "\n";
        for (int i = 0; i < M; i++)
        {
            // std::cout << coeff(i+1) << ", " << coeff(i) << "\n";
            dbez_coeffs(i) = M * (coeff(i + 1) - coeff(i));
        }
        return dbez_coeffs;
    }
    double Factorial(double n)
    {
        return (n == 1 || n == 0) ? 1 : n * Factorial(n - 1);
    }

    /* Coefficients for smooth trajectory from given initial and final position */
    void Smooth_Bezier_Coefficients(const double &qinit,
                                    const double &qfinal,
                                    const int &bezdeg,
                                    Ref<VectorXd> bez_coeffs)
    {
        // std::cout << "qinit: " << qinit << "\n";
        // std::cout << "qfinal: " << qfinal << "\n";
        VectorXd vect_init(bezdeg/2,1);
        VectorXd vect_final(bezdeg/2,1);
        vect_init.setZero();
        vect_final.setZero();

        for (int i = 0; i < bezdeg/2; i++)
        {
            vect_init(i) = qinit;
            vect_final(i) = qfinal;
        }
        // std::cout << "vinit: \n" << vect_init << "\n";
        // std::cout << "vfinal: \n" << vect_final << "\n";
        bez_coeffs.setZero();
        bez_coeffs << vect_init, 0.5*(qinit+qfinal), vect_final;
        std::cout << "\nbez_coeffs: \n" << bez_coeffs << "\n";

        return;
    }
} // bezier_utils
