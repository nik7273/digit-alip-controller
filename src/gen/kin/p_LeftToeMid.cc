/*
 * Automatically Generated from Mathematica.
 * Wed 27 Oct 2021 23:05:28 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "gen/kin/p_LeftToeMid.hh"

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
  double t68;
  double t50;
  double t70;
  double t56;
  double t71;
  double t9;
  double t61;
  double t72;
  double t73;
  double t76;
  double t77;
  double t115;
  double t164;
  double t512;
  double t565;
  double t634;
  double t1118;
  double t1628;
  double t1811;
  double t1819;
  double t2118;
  double t2134;
  double t2211;
  double t346;
  double t2650;
  double t2696;
  double t2705;
  double t2744;
  double t3428;
  double t3534;
  double t3547;
  double t3551;
  double t3560;
  double t3565;
  double t2390;
  double t2402;
  double t2417;
  double t2422;
  double t2445;
  double t2558;
  double t2850;
  double t2867;
  double t2941;
  double t2942;
  double t3061;
  double t3089;
  double t3764;
  double t3772;
  double t3780;
  double t3860;
  double t4048;
  double t4065;
  double t4072;
  double t4077;
  double t4097;
  double t4099;
  double t4230;
  double t4233;
  double t4260;
  double t4285;
  double t4286;
  double t4287;
  double t4387;
  double t4388;
  double t4405;
  double t4407;
  double t4428;
  double t4446;
  double t4508;
  double t4510;
  double t4514;
  double t4535;
  double t4577;
  double t4597;
  double t4603;
  double t4639;
  double t4684;
  double t4685;
  double t4713;
  double t4714;
  double t4722;
  double t4725;
  double t4755;
  double t4781;
  double t4827;
  double t4837;
  double t4838;
  double t4845;
  double t4846;
  double t4855;
  double t4866;
  double t4910;
  double t4949;
  double t4964;
  double t5058;
  double t5060;
  double t5100;
  double t5108;
  double t5120;
  double t5122;
  double t5293;
  double t5298;
  double t5304;
  double t5318;
  double t5319;
  double t5363;
  double t5397;
  double t5399;
  double t5418;
  double t5419;
  double t5421;
  double t5440;
  double t5449;
  double t5456;
  double t5465;
  double t5474;
  double t5746;
  double t5755;
  double t5759;
  double t5795;
  double t5510;
  double t5511;
  double t5519;
  double t5548;
  double t5552;
  double t5561;
  double t5631;
  double t5634;
  double t5646;
  double t5651;
  double t5652;
  double t5668;
  double t5676;
  double t5702;
  double t5707;
  double t5709;
  double t5712;
  double t5725;
  double t5784;
  double t5825;
  double t5991;
  double t6103;
  double t5875;
  double t5910;
  double t5920;
  double t5944;
  double t5946;
  double t5947;
  double t6163;
  double t6047;
  double t6048;
  double t6060;
  double t6064;
  double t6071;
  double t6081;
  double t6196;
  double t6206;
  double t6208;
  double t6215;
  double t6216;
  double t6217;
  double t6031;
  double t6237;
  double t6001;
  double t6273;
  double t5809;
  double t6220;
  double t5845;
  double t40;
  double t48;
  double t635;
  double t1149;
  double t1367;
  double t6471;
  double t6475;
  double t6476;
  double t6509;
  double t6519;
  double t6520;
  double t1820;
  double t1865;
  double t1883;
  double t2071;
  double t2263;
  double t2291;
  double t2325;
  double t2343;
  double t6538;
  double t6547;
  double t6557;
  double t6560;
  double t6568;
  double t6582;
  double t2562;
  double t2719;
  double t2779;
  double t2786;
  double t3209;
  double t3251;
  double t3252;
  double t3317;
  double t3696;
  double t3697;
  double t3722;
  double t3809;
  double t3882;
  double t3975;
  double t6689;
  double t6698;
  double t6702;
  double t6707;
  double t6585;
  double t6598;
  double t6611;
  double t6628;
  double t6631;
  double t6632;
  double t6639;
  double t6671;
  double t4101;
  double t4138;
  double t4201;
  double t4224;
  double t4290;
  double t4323;
  double t4364;
  double t4384;
  double t4502;
  double t4533;
  double t4547;
  double t4558;
  double t6740;
  double t6748;
  double t6756;
  double t6759;
  double t6805;
  double t6806;
  double t6815;
  double t6817;
  double t6830;
  double t6833;
  double t6836;
  double t6856;
  double t4703;
  double t4705;
  double t4709;
  double t4710;
  double t4803;
  double t4816;
  double t4824;
  double t4962;
  double t5016;
  double t5044;
  double t6898;
  double t6920;
  double t6926;
  double t6934;
  double t6974;
  double t7027;
  double t7045;
  double t7062;
  double t7107;
  double t7120;
  double t7159;
  double t7163;
  double t5227;
  double t5250;
  double t5257;
  double t5282;
  double t5371;
  double t5374;
  double t5375;
  double t5390;
  double t5447;
  double t5466;
  double t5485;
  double t5500;
  double t7201;
  double t7203;
  double t7219;
  double t7230;
  double t7246;
  double t7253;
  double t7272;
  double t7285;
  double t7395;
  double t7396;
  double t7406;
  double t7459;
  double t5601;
  double t5622;
  double t5623;
  double t5624;
  double t5670;
  double t5672;
  double t5674;
  double t5745;
  double t5780;
  double t5812;
  double t5819;
  double t5860;
  double t5861;
  double t5866;
  double t7505;
  double t7507;
  double t7514;
  double t7532;
  double t7570;
  double t7598;
  double t7612;
  double t7614;
  double t7630;
  double t7643;
  double t7678;
  double t7682;
  double t5952;
  double t5972;
  double t6006;
  double t6022;
  double t6032;
  double t6035;
  double t6045;
  double t6086;
  double t6093;
  double t6133;
  double t6145;
  double t6188;
  double t6189;
  double t6191;
  double t6222;
  double t7702;
  double t7718;
  double t7730;
  double t7744;
  double t6238;
  double t7827;
  double t7838;
  double t7859;
  double t7883;
  double t6245;
  double t6246;
  double t7894;
  double t7915;
  double t7917;
  double t7958;
  double t6290;
  double t6294;
  double t6305;
  double t6339;
  double t6373;
  double t6380;
  double t6394;
  double t6402;
  double t8270;
  double t8279;
  double t8285;
  double t8307;
  double t8314;
  double t8315;
  double t8419;
  double t8427;
  double t8429;
  double t8431;
  double t8388;
  double t8395;
  double t8407;
  double t8409;
  double t8319;
  double t8323;
  double t8329;
  double t8331;
  double t8434;
  double t8437;
  double t8453;
  double t8454;
  double t8461;
  double t8469;
  double t8475;
  double t8482;
  double t8486;
  double t8502;
  double t8503;
  double t8512;
  double t8522;
  double t8524;
  double t8525;
  double t8528;
  double t8533;
  double t8550;
  double t8551;
  double t8553;
  double t8570;
  double t8573;
  double t8574;
  double t8576;
  double t8579;
  double t8583;
  double t8584;
  double t8585;
  double t8613;
  double t8614;
  double t8619;
  double t8620;
  double t8631;
  double t8633;
  double t8637;
  double t8663;
  double t8666;
  double t8668;
  double t8669;
  double t8674;
  double t8676;
  double t8677;
  double t8679;
  double t8680;
  double t8688;
  double t8710;
  double t8720;
  double t8724;
  double t8729;
  double t8737;
  double t8744;
  double t8751;
  double t8772;
  double t8783;
  double t8797;
  double t8822;
  double t8825;
  double t8829;
  double t8831;
  double t8832;
  t68 = Cos(var1[3]);
  t50 = Cos(var1[5]);
  t70 = Sin(var1[4]);
  t56 = Sin(var1[3]);
  t71 = Sin(var1[5]);
  t9 = Cos(var1[6]);
  t61 = -1.*t50*t56;
  t72 = t68*t70*t71;
  t73 = t61 + t72;
  t76 = t68*t50*t70;
  t77 = t56*t71;
  t115 = t76 + t77;
  t164 = Sin(var1[6]);
  t512 = Cos(var1[7]);
  t565 = -1.*t512;
  t634 = 1. + t565;
  t1118 = Sin(var1[7]);
  t1628 = t9*t73;
  t1811 = -1.*t115*t164;
  t1819 = t1628 + t1811;
  t2118 = t9*t115;
  t2134 = t73*t164;
  t2211 = t2118 + t2134;
  t346 = Cos(var1[4]);
  t2650 = Cos(var1[8]);
  t2696 = -1.*t2650;
  t2705 = 1. + t2696;
  t2744 = Sin(var1[8]);
  t3428 = -1.000000637725*t634;
  t3534 = 1. + t3428;
  t3547 = t68*t346*t3534;
  t3551 = -0.930418*t1819*t1118;
  t3560 = -0.366501*t2211*t1118;
  t3565 = t3547 + t3551 + t3560;
  t2390 = -0.340999127418*t634*t1819;
  t2402 = -0.134322983001*t634;
  t2417 = 1. + t2402;
  t2422 = t2417*t2211;
  t2445 = 0.366501*t68*t346*t1118;
  t2558 = t2390 + t2422 + t2445;
  t2850 = -0.8656776547239999*t634;
  t2867 = 1. + t2850;
  t2941 = t2867*t1819;
  t2942 = -0.340999127418*t634*t2211;
  t3061 = 0.930418*t68*t346*t1118;
  t3089 = t2941 + t2942 + t3061;
  t3764 = Cos(var1[9]);
  t3772 = -1.*t3764;
  t3780 = 1. + t3772;
  t3860 = Sin(var1[9]);
  t4048 = -1.000000637725*t2705;
  t4065 = 1. + t4048;
  t4072 = t4065*t3565;
  t4077 = -0.930418*t2558*t2744;
  t4097 = 0.366501*t3089*t2744;
  t4099 = t4072 + t4077 + t4097;
  t4230 = 0.340999127418*t2705*t2558;
  t4233 = -0.134322983001*t2705;
  t4260 = 1. + t4233;
  t4285 = t4260*t3089;
  t4286 = -0.366501*t3565*t2744;
  t4287 = t4230 + t4285 + t4286;
  t4387 = -0.8656776547239999*t2705;
  t4388 = 1. + t4387;
  t4405 = t4388*t2558;
  t4407 = 0.340999127418*t2705*t3089;
  t4428 = 0.930418*t3565*t2744;
  t4446 = t4405 + t4407 + t4428;
  t4508 = Cos(var1[10]);
  t4510 = -1.*t4508;
  t4514 = 1. + t4510;
  t4535 = Sin(var1[10]);
  t4577 = -0.930418*t3860*t4099;
  t4597 = 0.340999127418*t3780*t4287;
  t4603 = -0.8656776547239999*t3780;
  t4639 = 1. + t4603;
  t4684 = t4639*t4446;
  t4685 = t4577 + t4597 + t4684;
  t4713 = 0.366501*t3860*t4099;
  t4714 = -0.134322983001*t3780;
  t4722 = 1. + t4714;
  t4725 = t4722*t4287;
  t4755 = 0.340999127418*t3780*t4446;
  t4781 = t4713 + t4725 + t4755;
  t4827 = -1.000000637725*t3780;
  t4837 = 1. + t4827;
  t4838 = t4837*t4099;
  t4845 = -0.366501*t3860*t4287;
  t4846 = 0.930418*t3860*t4446;
  t4855 = t4838 + t4845 + t4846;
  t4866 = Cos(var1[11]);
  t4910 = -1.*t4866;
  t4949 = 1. + t4910;
  t4964 = Sin(var1[11]);
  t5058 = 0.930418*t4535*t4685;
  t5060 = -0.366501*t4535*t4781;
  t5100 = -1.000000637725*t4514;
  t5108 = 1. + t5100;
  t5120 = t5108*t4855;
  t5122 = t5058 + t5060 + t5120;
  t5293 = -0.8656776547239999*t4514;
  t5298 = 1. + t5293;
  t5304 = t5298*t4685;
  t5318 = 0.340999127418*t4514*t4781;
  t5319 = -0.930418*t4535*t4855;
  t5363 = t5304 + t5318 + t5319;
  t5397 = 0.340999127418*t4514*t4685;
  t5399 = -0.134322983001*t4514;
  t5418 = 1. + t5399;
  t5419 = t5418*t4781;
  t5421 = 0.366501*t4535*t4855;
  t5440 = t5397 + t5419 + t5421;
  t5449 = Cos(var1[12]);
  t5456 = -1.*t5449;
  t5465 = 1. + t5456;
  t5474 = Sin(var1[12]);
  t5746 = Cos(var1[13]);
  t5755 = -1.*t5746;
  t5759 = 1. + t5755;
  t5795 = Sin(var1[13]);
  t5510 = 0.366501*t4964*t5122;
  t5511 = 0.340999127418*t4949*t5363;
  t5519 = -0.134322983001*t4949;
  t5548 = 1. + t5519;
  t5552 = t5548*t5440;
  t5561 = t5510 + t5511 + t5552;
  t5631 = -0.930418*t4964*t5122;
  t5634 = -0.8656776547239999*t4949;
  t5646 = 1. + t5634;
  t5651 = t5646*t5363;
  t5652 = 0.340999127418*t4949*t5440;
  t5668 = t5631 + t5651 + t5652;
  t5676 = -1.000000637725*t4949;
  t5702 = 1. + t5676;
  t5707 = t5702*t5122;
  t5709 = 0.930418*t4964*t5363;
  t5712 = -0.366501*t4964*t5440;
  t5725 = t5707 + t5709 + t5712;
  t5784 = -0.444895486988*t5759;
  t5825 = 0.175248972904*t5759;
  t5991 = 0.120666640478*t5759;
  t6103 = -0.553471*t5795;
  t5875 = -0.366501*t5474*t5561;
  t5910 = 0.930418*t5474*t5668;
  t5920 = -1.000000637725*t5465;
  t5944 = 1. + t5920;
  t5946 = t5944*t5725;
  t5947 = t5875 + t5910 + t5946;
  t6163 = 0.803828*t5795;
  t6047 = 0.340999127418*t5465*t5561;
  t6048 = -0.8656776547239999*t5465;
  t6060 = 1. + t6048;
  t6064 = t6060*t5668;
  t6071 = -0.930418*t5474*t5725;
  t6081 = t6047 + t6064 + t6071;
  t6196 = -0.134322983001*t5465;
  t6206 = 1. + t6196;
  t6208 = t6206*t5561;
  t6215 = 0.340999127418*t5465*t5668;
  t6216 = 0.366501*t5474*t5725;
  t6217 = t6208 + t6215 + t6216;
  t6031 = -0.218018*t5795;
  t6237 = -0.120666640478*t5759;
  t6001 = -0.803828*t5795;
  t6273 = 0.444895486988*t5759;
  t5809 = 0.218018*t5795;
  t6220 = -0.175248972904*t5759;
  t5845 = 0.553471*t5795;
  t40 = -1.*t9;
  t48 = 1. + t40;
  t635 = -0.04500040093286238*t634;
  t1149 = -0.0846680539949003*t1118;
  t1367 = t635 + t1149;
  t6471 = t68*t50;
  t6475 = t56*t70*t71;
  t6476 = t6471 + t6475;
  t6509 = t50*t56*t70;
  t6519 = -1.*t68*t71;
  t6520 = t6509 + t6519;
  t1820 = 1.296332362046933e-7*var1[7];
  t1865 = 0.07877668146182712*t634;
  t1883 = -0.04186915633414423*t1118;
  t2071 = t1820 + t1865 + t1883;
  t2263 = -3.2909349868922137e-7*var1[7];
  t2291 = 0.03103092645718495*t634;
  t2325 = -0.016492681424499736*t1118;
  t2343 = t2263 + t2291 + t2325;
  t6538 = t9*t6476;
  t6547 = -1.*t6520*t164;
  t6557 = t6538 + t6547;
  t6560 = t9*t6520;
  t6568 = t6476*t164;
  t6582 = t6560 + t6568;
  t2562 = 1.296332362046933e-7*var1[8];
  t2719 = -0.14128592423750855*t2705;
  t2779 = -0.04186915633414423*t2744;
  t2786 = t2562 + t2719 + t2779;
  t3209 = 3.2909349868922137e-7*var1[8];
  t3251 = 0.055653945343889656*t2705;
  t3252 = 0.016492681424499736*t2744;
  t3317 = t3209 + t3251 + t3252;
  t3696 = -0.04500040093286238*t2705;
  t3697 = 0.15185209683981668*t2744;
  t3722 = t3696 + t3697;
  t3809 = 0.039853038461262744*t3780;
  t3882 = -0.23670515095269612*t3860;
  t3975 = t3809 + t3882;
  t6689 = t346*t3534*t56;
  t6698 = -0.930418*t6557*t1118;
  t6702 = -0.366501*t6582*t1118;
  t6707 = t6689 + t6698 + t6702;
  t6585 = -0.340999127418*t634*t6557;
  t6598 = t2417*t6582;
  t6611 = 0.366501*t346*t56*t1118;
  t6628 = t6585 + t6598 + t6611;
  t6631 = t2867*t6557;
  t6632 = -0.340999127418*t634*t6582;
  t6639 = 0.930418*t346*t56*t1118;
  t6671 = t6631 + t6632 + t6639;
  t4101 = -1.5981976069815686e-7*var1[9];
  t4138 = 0.08675267452931407*t3780;
  t4201 = 0.014606169134372047*t3860;
  t4224 = t4101 + t4138 + t4201;
  t4290 = -6.295460977284962e-8*var1[9];
  t4323 = -0.22023473313910558*t3780;
  t4364 = -0.03707996069223323*t3860;
  t4384 = t4290 + t4323 + t4364;
  t4502 = -1.6084556086870008e-7*var1[10];
  t4533 = -0.29135406957765553*t4514;
  t4547 = -0.02832985722118838*t4535;
  t4558 = t4502 + t4533 + t4547;
  t6740 = t4065*t6707;
  t6748 = -0.930418*t6628*t2744;
  t6756 = 0.366501*t6671*t2744;
  t6759 = t6740 + t6748 + t6756;
  t6805 = 0.340999127418*t2705*t6628;
  t6806 = t4260*t6671;
  t6815 = -0.366501*t6707*t2744;
  t6817 = t6805 + t6806 + t6815;
  t6830 = t4388*t6628;
  t6833 = 0.340999127418*t2705*t6671;
  t6836 = 0.930418*t6707*t2744;
  t6856 = t6830 + t6833 + t6836;
  t4703 = -4.0833068682577724e-7*var1[10];
  t4705 = 0.11476729583292707*t4514;
  t4709 = 0.0111594154470601*t4535;
  t4710 = t4703 + t4705 + t4709;
  t4803 = 0.03044854601678662*t4514;
  t4816 = -0.3131431996991197*t4535;
  t4824 = t4803 + t4816;
  t4962 = -0.26285954081199375*t4949;
  t5016 = -0.634735404786378*t4964;
  t5044 = t4962 + t5016;
  t6898 = -0.930418*t3860*t6759;
  t6920 = 0.340999127418*t3780*t6817;
  t6926 = t4639*t6856;
  t6934 = t6898 + t6920 + t6926;
  t6974 = 0.366501*t3860*t6759;
  t7027 = t4722*t6817;
  t7045 = 0.340999127418*t3780*t6856;
  t7062 = t6974 + t7027 + t7045;
  t7107 = t4837*t6759;
  t7120 = -0.366501*t3860*t6817;
  t7159 = 0.930418*t3860*t6856;
  t7163 = t7107 + t7120 + t7159;
  t5227 = 6.369237629068993e-8*var1[11];
  t5250 = -0.5905692458505322*t4949;
  t5257 = 0.24456909227538925*t4964;
  t5282 = t5227 + t5250 + t5257;
  t5371 = 1.6169269214444473e-7*var1[11];
  t5374 = 0.2326311605896123*t4949;
  t5375 = -0.09633822312984319*t4964;
  t5390 = t5371 + t5374 + t5375;
  t5447 = 1.7876586242383724e-7*var1[12];
  t5466 = 0.3243041141817093*t5465;
  t5485 = 0.02270383571304597*t5474;
  t5500 = t5447 + t5466 + t5485;
  t7201 = 0.930418*t4535*t6934;
  t7203 = -0.366501*t4535*t7062;
  t7219 = t5108*t7163;
  t7230 = t7201 + t7203 + t7219;
  t7246 = t5298*t6934;
  t7253 = 0.340999127418*t4514*t7062;
  t7272 = -0.930418*t4535*t7163;
  t7285 = t7246 + t7253 + t7272;
  t7395 = 0.340999127418*t4514*t6934;
  t7396 = t5418*t7062;
  t7406 = 0.366501*t4535*t7163;
  t7459 = t7395 + t7396 + t7406;
  t5601 = 7.041766963257243e-8*var1[12];
  t5622 = -0.8232948486053725*t5465;
  t5623 = -0.05763710717422546*t5474;
  t5624 = t5601 + t5622 + t5623;
  t5670 = 0.06194758047549556*t5465;
  t5672 = -0.8848655643005321*t5474;
  t5674 = t5670 + t5672;
  t5745 = -2.7989049814696287e-7*var1[13];
  t5780 = 0.15748067958019524*t5759;
  t5812 = t5784 + t5809;
  t5819 = -0.528674719304*t5812;
  t5860 = t5825 + t5845;
  t5861 = 0.29871295412*t5860;
  t5866 = t5745 + t5780 + t5819 + t5861;
  t7505 = 0.366501*t4964*t7230;
  t7507 = 0.340999127418*t4949*t7285;
  t7514 = t5548*t7459;
  t7532 = t7505 + t7507 + t7514;
  t7570 = -0.930418*t4964*t7230;
  t7598 = t5646*t7285;
  t7612 = 0.340999127418*t4949*t7459;
  t7614 = t7570 + t7598 + t7612;
  t7630 = t5702*t7230;
  t7643 = 0.930418*t4964*t7285;
  t7678 = -0.366501*t4964*t7459;
  t7682 = t7630 + t7643 + t7678;
  t5952 = -1.9271694180831932e-7*var1[13];
  t5972 = -0.3667264808254521*t5759;
  t6006 = t5991 + t6001;
  t6022 = 0.29871295412*t6006;
  t6032 = t5784 + t6031;
  t6035 = 0.445034169498*t6032;
  t6045 = t5952 + t5972 + t6022 + t6035;
  t6086 = 7.591321355439789e-8*var1[13];
  t6093 = 0.2845150083511607*t5759;
  t6133 = t5825 + t6103;
  t6145 = 0.445034169498*t6133;
  t6188 = t5991 + t6163;
  t6189 = -0.528674719304*t6188;
  t6191 = t6086 + t6093 + t6145 + t6189;
  t6222 = t6220 + t6103;
  t7702 = -0.366501*t5474*t7532;
  t7718 = 0.930418*t5474*t7614;
  t7730 = t5944*t7682;
  t7744 = t7702 + t7718 + t7730;
  t6238 = t6237 + t6163;
  t7827 = 0.340999127418*t5465*t7532;
  t7838 = t6060*t7614;
  t7859 = -0.930418*t5474*t7682;
  t7883 = t7827 + t7838 + t7859;
  t6245 = -0.952469601425*t5759;
  t6246 = 1. + t6245;
  t7894 = t6206*t7532;
  t7915 = 0.340999127418*t5465*t7614;
  t7917 = 0.366501*t5474*t7682;
  t7958 = t7894 + t7915 + t7917;
  t6290 = t6273 + t6031;
  t6294 = -0.693671301908*t5759;
  t6305 = 1. + t6294;
  t6339 = t6237 + t6001;
  t6373 = -0.353861996165*t5759;
  t6380 = 1. + t6373;
  t6394 = t6273 + t5809;
  t6402 = t6220 + t5845;
  t8270 = t346*t9*t71;
  t8279 = -1.*t346*t50*t164;
  t8285 = t8270 + t8279;
  t8307 = t346*t50*t9;
  t8314 = t346*t71*t164;
  t8315 = t8307 + t8314;
  t8419 = -1.*t3534*t70;
  t8427 = -0.930418*t8285*t1118;
  t8429 = -0.366501*t8315*t1118;
  t8431 = t8419 + t8427 + t8429;
  t8388 = t2867*t8285;
  t8395 = -0.340999127418*t634*t8315;
  t8407 = -0.930418*t70*t1118;
  t8409 = t8388 + t8395 + t8407;
  t8319 = -0.340999127418*t634*t8285;
  t8323 = t2417*t8315;
  t8329 = -0.366501*t70*t1118;
  t8331 = t8319 + t8323 + t8329;
  t8434 = t4065*t8431;
  t8437 = 0.366501*t8409*t2744;
  t8453 = -0.930418*t8331*t2744;
  t8454 = t8434 + t8437 + t8453;
  t8461 = t4260*t8409;
  t8469 = 0.340999127418*t2705*t8331;
  t8475 = -0.366501*t8431*t2744;
  t8482 = t8461 + t8469 + t8475;
  t8486 = 0.340999127418*t2705*t8409;
  t8502 = t4388*t8331;
  t8503 = 0.930418*t8431*t2744;
  t8512 = t8486 + t8502 + t8503;
  t8522 = -0.930418*t3860*t8454;
  t8524 = 0.340999127418*t3780*t8482;
  t8525 = t4639*t8512;
  t8528 = t8522 + t8524 + t8525;
  t8533 = 0.366501*t3860*t8454;
  t8550 = t4722*t8482;
  t8551 = 0.340999127418*t3780*t8512;
  t8553 = t8533 + t8550 + t8551;
  t8570 = t4837*t8454;
  t8573 = -0.366501*t3860*t8482;
  t8574 = 0.930418*t3860*t8512;
  t8576 = t8570 + t8573 + t8574;
  t8579 = 0.930418*t4535*t8528;
  t8583 = -0.366501*t4535*t8553;
  t8584 = t5108*t8576;
  t8585 = t8579 + t8583 + t8584;
  t8613 = t5298*t8528;
  t8614 = 0.340999127418*t4514*t8553;
  t8619 = -0.930418*t4535*t8576;
  t8620 = t8613 + t8614 + t8619;
  t8631 = 0.340999127418*t4514*t8528;
  t8633 = t5418*t8553;
  t8637 = 0.366501*t4535*t8576;
  t8663 = t8631 + t8633 + t8637;
  t8666 = 0.366501*t4964*t8585;
  t8668 = 0.340999127418*t4949*t8620;
  t8669 = t5548*t8663;
  t8674 = t8666 + t8668 + t8669;
  t8676 = -0.930418*t4964*t8585;
  t8677 = t5646*t8620;
  t8679 = 0.340999127418*t4949*t8663;
  t8680 = t8676 + t8677 + t8679;
  t8688 = t5702*t8585;
  t8710 = 0.930418*t4964*t8620;
  t8720 = -0.366501*t4964*t8663;
  t8724 = t8688 + t8710 + t8720;
  t8729 = -0.366501*t5474*t8674;
  t8737 = 0.930418*t5474*t8680;
  t8744 = t5944*t8724;
  t8751 = t8729 + t8737 + t8744;
  t8772 = 0.340999127418*t5465*t8674;
  t8783 = t6060*t8680;
  t8797 = -0.930418*t5474*t8724;
  t8822 = t8772 + t8783 + t8797;
  t8825 = t6206*t8674;
  t8829 = 0.340999127418*t5465*t8680;
  t8831 = 0.366501*t5474*t8724;
  t8832 = t8825 + t8829 + t8831;
  p_output1[0]=0.091*t115*t164 + t1819*t2071 + t2211*t2343 + t2558*t2786 + t3089*t3317 + t3565*t3722 + t3975*t4099 + t4224*t4287 + t4384*t4446 + t4558*t4685 + t4710*t4781 + t4824*t4855 + t5044*t5122 + t5282*t5363 + t5390*t5440 + t5500*t5561 + t5624*t5668 + t5674*t5725 + t5866*t5947 + t6045*t6081 + t6191*t6217 + 0.425556*(t5947*t6222 + t6081*t6238 + t6217*t6246) - 0.850685*(t5947*t6290 + t6081*t6305 + t6217*t6339) + 0.069082*(t5947*t6380 + t6081*t6394 + t6217*t6402) + t1367*t346*t68 + 0.091*t48*t73 + var1[0];
  p_output1[1]=t1367*t346*t56 + 0.091*t48*t6476 + 0.091*t164*t6520 + t2071*t6557 + t2343*t6582 + t2786*t6628 + t3317*t6671 + t3722*t6707 + t3975*t6759 + t4224*t6817 + t4384*t6856 + t4558*t6934 + t4710*t7062 + t4824*t7163 + t5044*t7230 + t5282*t7285 + t5390*t7459 + t5500*t7532 + t5624*t7614 + t5674*t7682 + t5866*t7744 + t6045*t7883 + t6191*t7958 + 0.425556*(t6222*t7744 + t6238*t7883 + t6246*t7958) - 0.850685*(t6290*t7744 + t6305*t7883 + t6339*t7958) + 0.069082*(t6380*t7744 + t6394*t7883 + t6402*t7958) + var1[1];
  p_output1[2]=0.091*t164*t346*t50 - 1.*t1367*t70 + 0.091*t346*t48*t71 + t2071*t8285 + t2343*t8315 + t2786*t8331 + t3317*t8409 + t3722*t8431 + t3975*t8454 + t4224*t8482 + t4384*t8512 + t4558*t8528 + t4710*t8553 + t4824*t8576 + t5044*t8585 + t5282*t8620 + t5390*t8663 + t5500*t8674 + t5624*t8680 + t5674*t8724 + t5866*t8751 + t6045*t8822 + t6191*t8832 + 0.425556*(t6222*t8751 + t6238*t8822 + t6246*t8832) - 0.850685*(t6290*t8751 + t6305*t8822 + t6339*t8832) + 0.069082*(t6380*t8751 + t6394*t8822 + t6402*t8832) + var1[2];
}



void gen::kin::p_LeftToeMid(Eigen::Ref<Eigen::VectorXd> p_output1, const Eigen::Ref<const Eigen::VectorXd> var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
