/*
 * Automatically Generated from Mathematica.
 * Wed 27 Oct 2021 23:00:11 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "gen/kin/p_toe_pitch_joint_right.hh"

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
  double t36;
  double t434;
  double t572;
  double t485;
  double t670;
  double t41;
  double t42;
  double t59;
  double t252;
  double t347;
  double t490;
  double t848;
  double t945;
  double t1134;
  double t1180;
  double t1184;
  double t1195;
  double t39;
  double t1305;
  double t1347;
  double t1354;
  double t1607;
  double t1840;
  double t1880;
  double t1929;
  double t1931;
  double t2023;
  double t2110;
  double t2276;
  double t2355;
  double t2369;
  double t2372;
  double t2509;
  double t2538;
  double t2922;
  double t2923;
  double t2925;
  double t2972;
  double t3000;
  double t3002;
  double t3213;
  double t3214;
  double t3244;
  double t3245;
  double t3307;
  double t3366;
  double t3369;
  double t3508;
  double t3567;
  double t3680;
  double t3702;
  double t3715;
  double t3738;
  double t3775;
  double t3825;
  double t3847;
  double t4158;
  double t4194;
  double t4217;
  double t4219;
  double t4310;
  double t4327;
  double t4499;
  double t4551;
  double t4553;
  double t4568;
  double t4576;
  double t4585;
  double t4882;
  double t4890;
  double t4926;
  double t5007;
  double t5080;
  double t5152;
  double t5153;
  double t5154;
  double t5238;
  double t5259;
  double t5653;
  double t5673;
  double t5675;
  double t5719;
  double t5721;
  double t5747;
  double t6353;
  double t6407;
  double t6450;
  double t6606;
  double t7023;
  double t7108;
  double t7369;
  double t7384;
  double t7401;
  double t7435;
  double t7471;
  double t7508;
  double t7517;
  double t7542;
  double t7547;
  double t7557;
  double t7632;
  double t7645;
  double t7661;
  double t7662;
  double t7664;
  double t7676;
  double t7700;
  double t7710;
  double t7720;
  double t7728;
  double t7743;
  double t7751;
  double t7766;
  double t7774;
  double t7803;
  double t7812;
  double t7830;
  double t7834;
  double t7835;
  double t7837;
  double t7839;
  double t7841;
  double t7856;
  double t7861;
  double t7862;
  double t7870;
  double t7874;
  double t7875;
  double t7891;
  double t7895;
  double t7898;
  double t7900;
  double t7904;
  double t7907;
  double t80;
  double t285;
  double t296;
  double t1143;
  double t1173;
  double t1221;
  double t1222;
  double t1230;
  double t1284;
  double t7995;
  double t7997;
  double t7999;
  double t8006;
  double t8010;
  double t8024;
  double t1389;
  double t1396;
  double t1553;
  double t1561;
  double t1928;
  double t2075;
  double t2111;
  double t2112;
  double t8030;
  double t8031;
  double t8032;
  double t8038;
  double t8042;
  double t8049;
  double t2618;
  double t2665;
  double t2743;
  double t2810;
  double t3071;
  double t3121;
  double t3159;
  double t3570;
  double t3682;
  double t3698;
  double t8052;
  double t8053;
  double t8054;
  double t8055;
  double t8060;
  double t8062;
  double t8063;
  double t8065;
  double t8074;
  double t8075;
  double t8079;
  double t8080;
  double t3893;
  double t3903;
  double t3947;
  double t3989;
  double t4351;
  double t4375;
  double t4380;
  double t4408;
  double t4633;
  double t4961;
  double t5070;
  double t5073;
  double t8085;
  double t8086;
  double t8090;
  double t8091;
  double t8093;
  double t8095;
  double t8099;
  double t8100;
  double t8102;
  double t8104;
  double t8105;
  double t8107;
  double t5401;
  double t5467;
  double t5545;
  double t5598;
  double t5928;
  double t5987;
  double t6236;
  double t7402;
  double t7444;
  double t7454;
  double t8120;
  double t8122;
  double t8123;
  double t8124;
  double t8128;
  double t8130;
  double t8132;
  double t8133;
  double t8137;
  double t8138;
  double t8140;
  double t8143;
  double t7579;
  double t7603;
  double t7621;
  double t7626;
  double t7682;
  double t7683;
  double t7689;
  double t7699;
  double t7755;
  double t7811;
  double t7823;
  double t7829;
  double t8147;
  double t8148;
  double t8150;
  double t8154;
  double t8156;
  double t8157;
  double t8159;
  double t8160;
  double t8169;
  double t8172;
  double t8173;
  double t8177;
  double t7845;
  double t7848;
  double t7849;
  double t7854;
  double t7880;
  double t7886;
  double t7890;
  double t8183;
  double t8186;
  double t8189;
  double t8190;
  double t8195;
  double t8198;
  double t8202;
  double t8203;
  double t7941;
  double t7945;
  double t8210;
  double t8211;
  double t8214;
  double t8220;
  double t7959;
  double t7965;
  double t7975;
  double t7977;
  double t8246;
  double t8250;
  double t8251;
  double t8253;
  double t8254;
  double t8255;
  double t8257;
  double t8258;
  double t8260;
  double t8261;
  double t8263;
  double t8264;
  double t8265;
  double t8269;
  double t8271;
  double t8273;
  double t8274;
  double t8276;
  double t8278;
  double t8282;
  double t8283;
  double t8284;
  double t8286;
  double t8287;
  double t8289;
  double t8290;
  double t8292;
  double t8295;
  double t8300;
  double t8307;
  double t8309;
  double t8310;
  double t8311;
  double t8312;
  double t8314;
  double t8315;
  double t8316;
  double t8317;
  double t8319;
  double t8320;
  double t8321;
  double t8322;
  double t8324;
  double t8328;
  double t8329;
  double t8330;
  double t8337;
  double t8338;
  double t8341;
  double t8342;
  double t8344;
  double t8345;
  double t8346;
  double t8352;
  double t8354;
  double t8355;
  double t8356;
  double t8357;
  double t8359;
  double t8360;
  double t8361;
  double t8362;
  double t8365;
  double t8366;
  double t8367;
  double t8372;
  t36 = Cos(var1[3]);
  t434 = Cos(var1[5]);
  t572 = Sin(var1[3]);
  t485 = Sin(var1[4]);
  t670 = Sin(var1[5]);
  t41 = Cos(var1[19]);
  t42 = -1.*t41;
  t59 = 1. + t42;
  t252 = Sin(var1[19]);
  t347 = Sin(var1[18]);
  t490 = t36*t434*t485;
  t848 = t572*t670;
  t945 = t490 + t848;
  t1134 = Cos(var1[18]);
  t1180 = -1.*t434*t572;
  t1184 = t36*t485*t670;
  t1195 = t1180 + t1184;
  t39 = Cos(var1[4]);
  t1305 = -1.*t347*t945;
  t1347 = t1134*t1195;
  t1354 = t1305 + t1347;
  t1607 = t1134*t945;
  t1840 = t347*t1195;
  t1880 = t1607 + t1840;
  t1929 = Cos(var1[20]);
  t1931 = -1.*t1929;
  t2023 = 1. + t1931;
  t2110 = Sin(var1[20]);
  t2276 = -0.366501*t36*t39*t252;
  t2355 = 0.340999127418*t59*t1354;
  t2369 = -0.134322983001*t59;
  t2372 = 1. + t2369;
  t2509 = t2372*t1880;
  t2538 = t2276 + t2355 + t2509;
  t2922 = 0.930418*t36*t39*t252;
  t2923 = -0.8656776547239999*t59;
  t2925 = 1. + t2923;
  t2972 = t2925*t1354;
  t3000 = 0.340999127418*t59*t1880;
  t3002 = t2922 + t2972 + t3000;
  t3213 = -1.000000637725*t59;
  t3214 = 1. + t3213;
  t3244 = t3214*t36*t39;
  t3245 = -0.930418*t252*t1354;
  t3307 = 0.366501*t252*t1880;
  t3366 = t3244 + t3245 + t3307;
  t3369 = Cos(var1[21]);
  t3508 = -1.*t3369;
  t3567 = 1. + t3508;
  t3680 = Sin(var1[21]);
  t3702 = 0.930418*t2110*t2538;
  t3715 = 0.366501*t2110*t3002;
  t3738 = -1.000000637725*t2023;
  t3775 = 1. + t3738;
  t3825 = t3775*t3366;
  t3847 = t3702 + t3715 + t3825;
  t4158 = -0.8656776547239999*t2023;
  t4194 = 1. + t4158;
  t4217 = t4194*t2538;
  t4219 = -0.340999127418*t2023*t3002;
  t4310 = -0.930418*t2110*t3366;
  t4327 = t4217 + t4219 + t4310;
  t4499 = -0.340999127418*t2023*t2538;
  t4551 = -0.134322983001*t2023;
  t4553 = 1. + t4551;
  t4568 = t4553*t3002;
  t4576 = -0.366501*t2110*t3366;
  t4585 = t4499 + t4568 + t4576;
  t4882 = Cos(var1[22]);
  t4890 = -1.*t4882;
  t4926 = 1. + t4890;
  t5007 = Sin(var1[22]);
  t5080 = 0.366501*t3680*t3847;
  t5152 = -0.340999127418*t3567*t4327;
  t5153 = -0.134322983001*t3567;
  t5154 = 1. + t5153;
  t5238 = t5154*t4585;
  t5259 = t5080 + t5152 + t5238;
  t5653 = 0.930418*t3680*t3847;
  t5673 = -0.8656776547239999*t3567;
  t5675 = 1. + t5673;
  t5719 = t5675*t4327;
  t5721 = -0.340999127418*t3567*t4585;
  t5747 = t5653 + t5719 + t5721;
  t6353 = -1.000000637725*t3567;
  t6407 = 1. + t6353;
  t6450 = t6407*t3847;
  t6606 = -0.930418*t3680*t4327;
  t7023 = -0.366501*t3680*t4585;
  t7108 = t6450 + t6606 + t7023;
  t7369 = Cos(var1[23]);
  t7384 = -1.*t7369;
  t7401 = 1. + t7384;
  t7435 = Sin(var1[23]);
  t7471 = -0.366501*t5007*t5259;
  t7508 = -0.930418*t5007*t5747;
  t7517 = -1.000000637725*t4926;
  t7542 = 1. + t7517;
  t7547 = t7542*t7108;
  t7557 = t7471 + t7508 + t7547;
  t7632 = -0.134322983001*t4926;
  t7645 = 1. + t7632;
  t7661 = t7645*t5259;
  t7662 = -0.340999127418*t4926*t5747;
  t7664 = 0.366501*t5007*t7108;
  t7676 = t7661 + t7662 + t7664;
  t7700 = -0.340999127418*t4926*t5259;
  t7710 = -0.8656776547239999*t4926;
  t7720 = 1. + t7710;
  t7728 = t7720*t5747;
  t7743 = 0.930418*t5007*t7108;
  t7751 = t7700 + t7728 + t7743;
  t7766 = Cos(var1[24]);
  t7774 = -1.*t7766;
  t7803 = 1. + t7774;
  t7812 = Sin(var1[24]);
  t7830 = 0.930418*t7435*t7557;
  t7834 = -0.340999127418*t7401*t7676;
  t7835 = -0.8656776547239999*t7401;
  t7837 = 1. + t7835;
  t7839 = t7837*t7751;
  t7841 = t7830 + t7834 + t7839;
  t7856 = 0.366501*t7435*t7557;
  t7861 = -0.134322983001*t7401;
  t7862 = 1. + t7861;
  t7870 = t7862*t7676;
  t7874 = -0.340999127418*t7401*t7751;
  t7875 = t7856 + t7870 + t7874;
  t7891 = -1.000000637725*t7401;
  t7895 = 1. + t7891;
  t7898 = t7895*t7557;
  t7900 = -0.366501*t7435*t7676;
  t7904 = -0.930418*t7435*t7751;
  t7907 = t7898 + t7900 + t7904;
  t80 = -0.04500040093286238*t59;
  t285 = 0.0846680539949003*t252;
  t296 = t80 + t285;
  t1143 = -1.*t1134;
  t1173 = 1. + t1143;
  t1221 = 1.296332362046933e-7*var1[19];
  t1222 = -0.07877668146182712*t59;
  t1230 = -0.04186915633414423*t252;
  t1284 = t1221 + t1222 + t1230;
  t7995 = t434*t572*t485;
  t7997 = -1.*t36*t670;
  t7999 = t7995 + t7997;
  t8006 = t36*t434;
  t8010 = t572*t485*t670;
  t8024 = t8006 + t8010;
  t1389 = 3.2909349868922137e-7*var1[19];
  t1396 = 0.03103092645718495*t59;
  t1553 = 0.016492681424499736*t252;
  t1561 = t1389 + t1396 + t1553;
  t1928 = -1.296332362046933e-7*var1[20];
  t2075 = -0.14128592423750855*t2023;
  t2111 = 0.04186915633414423*t2110;
  t2112 = t1928 + t2075 + t2111;
  t8030 = -1.*t347*t7999;
  t8031 = t1134*t8024;
  t8032 = t8030 + t8031;
  t8038 = t1134*t7999;
  t8042 = t347*t8024;
  t8049 = t8038 + t8042;
  t2618 = 3.2909349868922137e-7*var1[20];
  t2665 = -0.055653945343889656*t2023;
  t2743 = 0.016492681424499736*t2110;
  t2810 = t2618 + t2665 + t2743;
  t3071 = -0.04500040093286238*t2023;
  t3121 = -0.15185209683981668*t2110;
  t3159 = t3071 + t3121;
  t3570 = 0.039853038461262744*t3567;
  t3682 = 0.23670515095269612*t3680;
  t3698 = t3570 + t3682;
  t8052 = -0.366501*t39*t252*t572;
  t8053 = 0.340999127418*t59*t8032;
  t8054 = t2372*t8049;
  t8055 = t8052 + t8053 + t8054;
  t8060 = 0.930418*t39*t252*t572;
  t8062 = t2925*t8032;
  t8063 = 0.340999127418*t59*t8049;
  t8065 = t8060 + t8062 + t8063;
  t8074 = t3214*t39*t572;
  t8075 = -0.930418*t252*t8032;
  t8079 = 0.366501*t252*t8049;
  t8080 = t8074 + t8075 + t8079;
  t3893 = 6.295460977284962e-8*var1[21];
  t3903 = -0.22023473313910558*t3567;
  t3947 = 0.03707996069223323*t3680;
  t3989 = t3893 + t3903 + t3947;
  t4351 = -1.5981976069815686e-7*var1[21];
  t4375 = -0.08675267452931407*t3567;
  t4380 = 0.014606169134372047*t3680;
  t4408 = t4351 + t4375 + t4380;
  t4633 = -4.0833068682577724e-7*var1[22];
  t4961 = -0.11476729583292707*t4926;
  t5070 = 0.0111594154470601*t5007;
  t5073 = t4633 + t4961 + t5070;
  t8085 = 0.930418*t2110*t8055;
  t8086 = 0.366501*t2110*t8065;
  t8090 = t3775*t8080;
  t8091 = t8085 + t8086 + t8090;
  t8093 = t4194*t8055;
  t8095 = -0.340999127418*t2023*t8065;
  t8099 = -0.930418*t2110*t8080;
  t8100 = t8093 + t8095 + t8099;
  t8102 = -0.340999127418*t2023*t8055;
  t8104 = t4553*t8065;
  t8105 = -0.366501*t2110*t8080;
  t8107 = t8102 + t8104 + t8105;
  t5401 = 1.6084556086870008e-7*var1[22];
  t5467 = -0.29135406957765553*t4926;
  t5545 = 0.02832985722118838*t5007;
  t5598 = t5401 + t5467 + t5545;
  t5928 = 0.03044854601678662*t4926;
  t5987 = 0.3131431996991197*t5007;
  t6236 = t5928 + t5987;
  t7402 = -0.26285954081199375*t7401;
  t7444 = 0.634735404786378*t7435;
  t7454 = t7402 + t7444;
  t8120 = 0.366501*t3680*t8091;
  t8122 = -0.340999127418*t3567*t8100;
  t8123 = t5154*t8107;
  t8124 = t8120 + t8122 + t8123;
  t8128 = 0.930418*t3680*t8091;
  t8130 = t5675*t8100;
  t8132 = -0.340999127418*t3567*t8107;
  t8133 = t8128 + t8130 + t8132;
  t8137 = t6407*t8091;
  t8138 = -0.930418*t3680*t8100;
  t8140 = -0.366501*t3680*t8107;
  t8143 = t8137 + t8138 + t8140;
  t7579 = 1.6169269214444473e-7*var1[23];
  t7603 = -0.2326311605896123*t7401;
  t7621 = -0.09633822312984319*t7435;
  t7626 = t7579 + t7603 + t7621;
  t7682 = -6.369237629068993e-8*var1[23];
  t7683 = -0.5905692458505322*t7401;
  t7689 = -0.24456909227538925*t7435;
  t7699 = t7682 + t7683 + t7689;
  t7755 = -7.041766963257243e-8*var1[24];
  t7811 = -0.8232948486053725*t7803;
  t7823 = 0.05763710717422546*t7812;
  t7829 = t7755 + t7811 + t7823;
  t8147 = -0.366501*t5007*t8124;
  t8148 = -0.930418*t5007*t8133;
  t8150 = t7542*t8143;
  t8154 = t8147 + t8148 + t8150;
  t8156 = t7645*t8124;
  t8157 = -0.340999127418*t4926*t8133;
  t8159 = 0.366501*t5007*t8143;
  t8160 = t8156 + t8157 + t8159;
  t8169 = -0.340999127418*t4926*t8124;
  t8172 = t7720*t8133;
  t8173 = 0.930418*t5007*t8143;
  t8177 = t8169 + t8172 + t8173;
  t7845 = 1.7876586242383724e-7*var1[24];
  t7848 = -0.3243041141817093*t7803;
  t7849 = 0.02270383571304597*t7812;
  t7854 = t7845 + t7848 + t7849;
  t7880 = 0.06194758047549556*t7803;
  t7886 = 0.8848655643005321*t7812;
  t7890 = t7880 + t7886;
  t8183 = 0.930418*t7435*t8154;
  t8186 = -0.340999127418*t7401*t8160;
  t8189 = t7837*t8177;
  t8190 = t8183 + t8186 + t8189;
  t8195 = 0.366501*t7435*t8154;
  t8198 = t7862*t8160;
  t8202 = -0.340999127418*t7401*t8177;
  t8203 = t8195 + t8198 + t8202;
  t7941 = -1.000000637725*t7803;
  t7945 = 1. + t7941;
  t8210 = t7895*t8154;
  t8211 = -0.366501*t7435*t8160;
  t8214 = -0.930418*t7435*t8177;
  t8220 = t8210 + t8211 + t8214;
  t7959 = -0.134322983001*t7803;
  t7965 = 1. + t7959;
  t7975 = -0.8656776547239999*t7803;
  t7977 = 1. + t7975;
  t8246 = -1.*t39*t434*t347;
  t8250 = t1134*t39*t670;
  t8251 = t8246 + t8250;
  t8253 = t1134*t39*t434;
  t8254 = t39*t347*t670;
  t8255 = t8253 + t8254;
  t8257 = 0.366501*t252*t485;
  t8258 = 0.340999127418*t59*t8251;
  t8260 = t2372*t8255;
  t8261 = t8257 + t8258 + t8260;
  t8263 = -0.930418*t252*t485;
  t8264 = t2925*t8251;
  t8265 = 0.340999127418*t59*t8255;
  t8269 = t8263 + t8264 + t8265;
  t8271 = -1.*t3214*t485;
  t8273 = -0.930418*t252*t8251;
  t8274 = 0.366501*t252*t8255;
  t8276 = t8271 + t8273 + t8274;
  t8278 = 0.930418*t2110*t8261;
  t8282 = 0.366501*t2110*t8269;
  t8283 = t3775*t8276;
  t8284 = t8278 + t8282 + t8283;
  t8286 = t4194*t8261;
  t8287 = -0.340999127418*t2023*t8269;
  t8289 = -0.930418*t2110*t8276;
  t8290 = t8286 + t8287 + t8289;
  t8292 = -0.340999127418*t2023*t8261;
  t8295 = t4553*t8269;
  t8300 = -0.366501*t2110*t8276;
  t8307 = t8292 + t8295 + t8300;
  t8309 = 0.366501*t3680*t8284;
  t8310 = -0.340999127418*t3567*t8290;
  t8311 = t5154*t8307;
  t8312 = t8309 + t8310 + t8311;
  t8314 = 0.930418*t3680*t8284;
  t8315 = t5675*t8290;
  t8316 = -0.340999127418*t3567*t8307;
  t8317 = t8314 + t8315 + t8316;
  t8319 = t6407*t8284;
  t8320 = -0.930418*t3680*t8290;
  t8321 = -0.366501*t3680*t8307;
  t8322 = t8319 + t8320 + t8321;
  t8324 = -0.366501*t5007*t8312;
  t8328 = -0.930418*t5007*t8317;
  t8329 = t7542*t8322;
  t8330 = t8324 + t8328 + t8329;
  t8337 = t7645*t8312;
  t8338 = -0.340999127418*t4926*t8317;
  t8341 = 0.366501*t5007*t8322;
  t8342 = t8337 + t8338 + t8341;
  t8344 = -0.340999127418*t4926*t8312;
  t8345 = t7720*t8317;
  t8346 = 0.930418*t5007*t8322;
  t8352 = t8344 + t8345 + t8346;
  t8354 = 0.930418*t7435*t8330;
  t8355 = -0.340999127418*t7401*t8342;
  t8356 = t7837*t8352;
  t8357 = t8354 + t8355 + t8356;
  t8359 = 0.366501*t7435*t8330;
  t8360 = t7862*t8342;
  t8361 = -0.340999127418*t7401*t8352;
  t8362 = t8359 + t8360 + t8361;
  t8365 = t7895*t8330;
  t8366 = -0.366501*t7435*t8342;
  t8367 = -0.930418*t7435*t8352;
  t8372 = t8365 + t8366 + t8367;
  p_output1[0]=-0.091*t1173*t1195 + t1284*t1354 + t1561*t1880 + t2112*t2538 + t2810*t3002 + t3159*t3366 + t3698*t3847 + t296*t36*t39 + t3989*t4327 + t4408*t4585 + t5073*t5259 + t5598*t5747 + t6236*t7108 + t7454*t7557 + t7626*t7676 + t7699*t7751 + t7829*t7841 + t7854*t7875 + t7890*t7907 + 0.061947*(-0.930418*t7812*t7841 - 0.366501*t7812*t7875 + t7907*t7945) - 0.402615*(-0.340999127418*t7803*t7841 + 0.366501*t7812*t7907 + t7875*t7965) - 0.792446*(-0.340999127418*t7803*t7875 + 0.930418*t7812*t7907 + t7841*t7977) - 0.091*t347*t945 + var1[0];
  p_output1[1]=t296*t39*t572 - 0.091*t347*t7999 - 0.091*t1173*t8024 + t1284*t8032 + t1561*t8049 + t2112*t8055 + t2810*t8065 + t3159*t8080 + t3698*t8091 + t3989*t8100 + t4408*t8107 + t5073*t8124 + t5598*t8133 + t6236*t8143 + t7454*t8154 + t7626*t8160 + t7699*t8177 + t7829*t8190 + t7854*t8203 + t7890*t8220 - 0.402615*(-0.340999127418*t7803*t8190 + t7965*t8203 + 0.366501*t7812*t8220) - 0.792446*(t7977*t8190 - 0.340999127418*t7803*t8203 + 0.930418*t7812*t8220) + 0.061947*(-0.930418*t7812*t8190 - 0.366501*t7812*t8203 + t7945*t8220) + var1[1];
  p_output1[2]=-0.091*t347*t39*t434 - 1.*t296*t485 - 0.091*t1173*t39*t670 + t1284*t8251 + t1561*t8255 + t2112*t8261 + t2810*t8269 + t3159*t8276 + t3698*t8284 + t3989*t8290 + t4408*t8307 + t5073*t8312 + t5598*t8317 + t6236*t8322 + t7454*t8330 + t7626*t8342 + t7699*t8352 + t7829*t8357 + t7854*t8362 + t7890*t8372 - 0.402615*(-0.340999127418*t7803*t8357 + t7965*t8362 + 0.366501*t7812*t8372) - 0.792446*(t7977*t8357 - 0.340999127418*t7803*t8362 + 0.930418*t7812*t8372) + 0.061947*(-0.930418*t7812*t8357 - 0.366501*t7812*t8362 + t7945*t8372) + var1[2];
}



void gen::kin::p_toe_pitch_joint_right(Eigen::Ref<Eigen::VectorXd> p_output1, const Eigen::Ref<const Eigen::VectorXd> var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
