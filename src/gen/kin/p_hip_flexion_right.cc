/*
 * Automatically Generated from Mathematica.
 * Wed 27 Oct 2021 22:59:45 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "gen/kin/p_hip_flexion_right.hh"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }
//INLINE double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

INLINE double Sec(double x) { return 1/cos(x); }
INLINE double Csc(double x) { return 1/sin(x); }

/*
 * Sub functions
 */
static void output1(Eigen::Ref<Eigen::VectorXd> p_output1,const Eigen::Ref<const Eigen::VectorXd> var1)
{
  double t636;
  double t3431;
  double t4948;
  double t3613;
  double t5406;
  double t1535;
  double t1536;
  double t1755;
  double t1773;
  double t2956;
  double t4614;
  double t6496;
  double t6503;
  double t6512;
  double t6546;
  double t6556;
  double t6558;
  double t1507;
  double t6606;
  double t6615;
  double t6618;
  double t6647;
  double t6648;
  double t6649;
  double t6667;
  double t6669;
  double t6670;
  double t6674;
  double t6677;
  double t6678;
  double t6679;
  double t6686;
  double t6696;
  double t6698;
  double t6711;
  double t6713;
  double t6714;
  double t6715;
  double t6717;
  double t6718;
  double t6724;
  double t6725;
  double t6726;
  double t6729;
  double t6730;
  double t6732;
  double t1769;
  double t1827;
  double t2332;
  double t6544;
  double t6545;
  double t6576;
  double t6587;
  double t6589;
  double t6597;
  double t6783;
  double t6785;
  double t6788;
  double t6793;
  double t6794;
  double t6795;
  double t6623;
  double t6626;
  double t6637;
  double t6641;
  double t6665;
  double t6673;
  double t6675;
  double t6676;
  double t6799;
  double t6800;
  double t6801;
  double t6803;
  double t6804;
  double t6805;
  double t6703;
  double t6704;
  double t6705;
  double t6710;
  double t6720;
  double t6721;
  double t6722;
  double t6807;
  double t6808;
  double t6810;
  double t6811;
  double t6814;
  double t6815;
  double t6816;
  double t6817;
  double t6743;
  double t6744;
  double t6819;
  double t6821;
  double t6822;
  double t6825;
  double t6750;
  double t6751;
  double t6769;
  double t6770;
  double t6853;
  double t6854;
  double t6855;
  double t6857;
  double t6858;
  double t6859;
  double t6861;
  double t6862;
  double t6864;
  double t6865;
  double t6869;
  double t6870;
  double t6871;
  double t6872;
  double t6874;
  double t6875;
  double t6876;
  double t6877;
  t636 = Cos(var1[3]);
  t3431 = Cos(var1[5]);
  t4948 = Sin(var1[3]);
  t3613 = Sin(var1[4]);
  t5406 = Sin(var1[5]);
  t1535 = Cos(var1[19]);
  t1536 = -1.*t1535;
  t1755 = 1. + t1536;
  t1773 = Sin(var1[19]);
  t2956 = Sin(var1[18]);
  t4614 = t636*t3431*t3613;
  t6496 = t4948*t5406;
  t6503 = t4614 + t6496;
  t6512 = Cos(var1[18]);
  t6546 = -1.*t3431*t4948;
  t6556 = t636*t3613*t5406;
  t6558 = t6546 + t6556;
  t1507 = Cos(var1[4]);
  t6606 = -1.*t2956*t6503;
  t6615 = t6512*t6558;
  t6618 = t6606 + t6615;
  t6647 = t6512*t6503;
  t6648 = t2956*t6558;
  t6649 = t6647 + t6648;
  t6667 = Cos(var1[20]);
  t6669 = -1.*t6667;
  t6670 = 1. + t6669;
  t6674 = Sin(var1[20]);
  t6677 = -0.366501*t636*t1507*t1773;
  t6678 = 0.340999127418*t1755*t6618;
  t6679 = -0.134322983001*t1755;
  t6686 = 1. + t6679;
  t6696 = t6686*t6649;
  t6698 = t6677 + t6678 + t6696;
  t6711 = 0.930418*t636*t1507*t1773;
  t6713 = -0.8656776547239999*t1755;
  t6714 = 1. + t6713;
  t6715 = t6714*t6618;
  t6717 = 0.340999127418*t1755*t6649;
  t6718 = t6711 + t6715 + t6717;
  t6724 = -1.000000637725*t1755;
  t6725 = 1. + t6724;
  t6726 = t6725*t636*t1507;
  t6729 = -0.930418*t1773*t6618;
  t6730 = 0.366501*t1773*t6649;
  t6732 = t6726 + t6729 + t6730;
  t1769 = -0.04500040093286238*t1755;
  t1827 = 0.0846680539949003*t1773;
  t2332 = t1769 + t1827;
  t6544 = -1.*t6512;
  t6545 = 1. + t6544;
  t6576 = 1.296332362046933e-7*var1[19];
  t6587 = -0.07877668146182712*t1755;
  t6589 = -0.04186915633414423*t1773;
  t6597 = t6576 + t6587 + t6589;
  t6783 = t3431*t4948*t3613;
  t6785 = -1.*t636*t5406;
  t6788 = t6783 + t6785;
  t6793 = t636*t3431;
  t6794 = t4948*t3613*t5406;
  t6795 = t6793 + t6794;
  t6623 = 3.2909349868922137e-7*var1[19];
  t6626 = 0.03103092645718495*t1755;
  t6637 = 0.016492681424499736*t1773;
  t6641 = t6623 + t6626 + t6637;
  t6665 = -1.296332362046933e-7*var1[20];
  t6673 = -0.14128592423750855*t6670;
  t6675 = 0.04186915633414423*t6674;
  t6676 = t6665 + t6673 + t6675;
  t6799 = -1.*t2956*t6788;
  t6800 = t6512*t6795;
  t6801 = t6799 + t6800;
  t6803 = t6512*t6788;
  t6804 = t2956*t6795;
  t6805 = t6803 + t6804;
  t6703 = 3.2909349868922137e-7*var1[20];
  t6704 = -0.055653945343889656*t6670;
  t6705 = 0.016492681424499736*t6674;
  t6710 = t6703 + t6704 + t6705;
  t6720 = -0.04500040093286238*t6670;
  t6721 = -0.15185209683981668*t6674;
  t6722 = t6720 + t6721;
  t6807 = -0.366501*t1507*t1773*t4948;
  t6808 = 0.340999127418*t1755*t6801;
  t6810 = t6686*t6805;
  t6811 = t6807 + t6808 + t6810;
  t6814 = 0.930418*t1507*t1773*t4948;
  t6815 = t6714*t6801;
  t6816 = 0.340999127418*t1755*t6805;
  t6817 = t6814 + t6815 + t6816;
  t6743 = -1.000000637725*t6670;
  t6744 = 1. + t6743;
  t6819 = t6725*t1507*t4948;
  t6821 = -0.930418*t1773*t6801;
  t6822 = 0.366501*t1773*t6805;
  t6825 = t6819 + t6821 + t6822;
  t6750 = -0.8656776547239999*t6670;
  t6751 = 1. + t6750;
  t6769 = -0.134322983001*t6670;
  t6770 = 1. + t6769;
  t6853 = -1.*t1507*t3431*t2956;
  t6854 = t6512*t1507*t5406;
  t6855 = t6853 + t6854;
  t6857 = t6512*t1507*t3431;
  t6858 = t1507*t2956*t5406;
  t6859 = t6857 + t6858;
  t6861 = 0.366501*t1773*t3613;
  t6862 = 0.340999127418*t1755*t6855;
  t6864 = t6686*t6859;
  t6865 = t6861 + t6862 + t6864;
  t6869 = -0.930418*t1773*t3613;
  t6870 = t6714*t6855;
  t6871 = 0.340999127418*t1755*t6859;
  t6872 = t6869 + t6870 + t6871;
  t6874 = -1.*t6725*t3613;
  t6875 = -0.930418*t1773*t6855;
  t6876 = 0.366501*t1773*t6859;
  t6877 = t6874 + t6875 + t6876;
  p_output1[0]=t1507*t2332*t636 - 0.091*t2956*t6503 - 0.091*t6545*t6558 + t6597*t6618 + t6641*t6649 + t6676*t6698 + t6710*t6718 + t6722*t6732 - 0.045*(0.930418*t6674*t6698 + 0.366501*t6674*t6718 + t6732*t6744) - 0.108789*(-0.340999127418*t6670*t6718 - 0.930418*t6674*t6732 + t6698*t6751) - 0.138152*(-0.340999127418*t6670*t6698 - 0.366501*t6674*t6732 + t6718*t6770) + var1[0];
  p_output1[1]=t1507*t2332*t4948 - 0.091*t2956*t6788 - 0.091*t6545*t6795 + t6597*t6801 + t6641*t6805 + t6676*t6811 + t6710*t6817 + t6722*t6825 - 0.108789*(t6751*t6811 - 0.340999127418*t6670*t6817 - 0.930418*t6674*t6825) - 0.138152*(-0.340999127418*t6670*t6811 + t6770*t6817 - 0.366501*t6674*t6825) - 0.045*(0.930418*t6674*t6811 + 0.366501*t6674*t6817 + t6744*t6825) + var1[1];
  p_output1[2]=-0.091*t1507*t2956*t3431 - 1.*t2332*t3613 - 0.091*t1507*t5406*t6545 + t6597*t6855 + t6641*t6859 + t6676*t6865 + t6710*t6872 + t6722*t6877 - 0.108789*(t6751*t6865 - 0.340999127418*t6670*t6872 - 0.930418*t6674*t6877) - 0.138152*(-0.340999127418*t6670*t6865 + t6770*t6872 - 0.366501*t6674*t6877) - 0.045*(0.930418*t6674*t6865 + 0.366501*t6674*t6872 + t6744*t6877) + var1[2];
}



void gen::kin::p_hip_flexion_right(Eigen::Ref<Eigen::VectorXd> p_output1, const Eigen::Ref<const Eigen::VectorXd> var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
