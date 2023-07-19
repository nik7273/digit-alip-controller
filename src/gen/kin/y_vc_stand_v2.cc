/*
 * Automatically Generated from Mathematica.
 * Wed 12 Jan 2022 03:45:16 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "gen/kin/y_vc_stand_v2.hh"

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
static void output1(Eigen::Ref<Eigen::VectorXd> p_output1,const Eigen::Ref<const Eigen::VectorXd> var1,const Eigen::Ref<const Eigen::VectorXd> var2)
{
  double t27;
  double t102;
  double t118;
  double t105;
  double t139;
  double t195;
  double t205;
  double t243;
  double t56;
  double t57;
  double t67;
  double t69;
  double t91;
  double t106;
  double t140;
  double t170;
  double t173;
  double t40;
  double t307;
  double t343;
  double t397;
  double t450;
  double t494;
  double t531;
  double t569;
  double t574;
  double t577;
  double t595;
  double t630;
  double t634;
  double t645;
  double t646;
  double t683;
  double t698;
  double t733;
  double t838;
  double t840;
  double t842;
  double t871;
  double t880;
  double t941;
  double t958;
  double t991;
  double t1013;
  double t1017;
  double t1019;
  double t1022;
  double t1065;
  double t1081;
  double t1088;
  double t1097;
  double t1108;
  double t1109;
  double t1115;
  double t1116;
  double t1137;
  double t1172;
  double t1174;
  double t1179;
  double t1186;
  double t1208;
  double t1224;
  double t1349;
  double t1361;
  double t1366;
  double t1367;
  double t1369;
  double t1396;
  double t1448;
  double t1456;
  double t1467;
  double t1506;
  double t1531;
  double t1537;
  double t1556;
  double t1557;
  double t1577;
  double t1626;
  double t1700;
  double t1704;
  double t1720;
  double t1721;
  double t1726;
  double t1727;
  double t1750;
  double t1763;
  double t1799;
  double t1808;
  double t1821;
  double t1840;
  double t1855;
  double t1863;
  double t1873;
  double t1879;
  double t1911;
  double t1912;
  double t1918;
  double t1951;
  double t1958;
  double t1969;
  double t2085;
  double t2086;
  double t2087;
  double t2089;
  double t2109;
  double t2115;
  double t2178;
  double t2179;
  double t2182;
  double t2183;
  double t2191;
  double t2208;
  double t2263;
  double t2268;
  double t2271;
  double t2277;
  double t2297;
  double t2304;
  double t2321;
  double t2332;
  double t2334;
  double t2336;
  double t2406;
  double t2416;
  double t2422;
  double t2426;
  double t2455;
  double t2456;
  double t2517;
  double t2519;
  double t2529;
  double t2562;
  double t2565;
  double t2566;
  double t263;
  double t2746;
  double t2749;
  double t2752;
  double t2766;
  double t2775;
  double t2804;
  double t2808;
  double t2809;
  double t2850;
  double t2854;
  double t2873;
  double t2957;
  double t2968;
  double t2969;
  double t2990;
  double t3099;
  double t3102;
  double t3108;
  double t3123;
  double t3124;
  double t3127;
  double t2918;
  double t2926;
  double t2942;
  double t2943;
  double t2945;
  double t2950;
  double t3012;
  double t3013;
  double t3015;
  double t3030;
  double t3035;
  double t3047;
  double t3144;
  double t3145;
  double t3153;
  double t3156;
  double t3185;
  double t3203;
  double t3204;
  double t3207;
  double t3213;
  double t3214;
  double t3232;
  double t3242;
  double t3246;
  double t3249;
  double t3252;
  double t3255;
  double t3267;
  double t3268;
  double t3271;
  double t3272;
  double t3273;
  double t3274;
  double t3289;
  double t3292;
  double t3294;
  double t3303;
  double t3312;
  double t3313;
  double t3314;
  double t3317;
  double t3322;
  double t3325;
  double t3347;
  double t3349;
  double t3352;
  double t3353;
  double t3361;
  double t3363;
  double t3378;
  double t3379;
  double t3381;
  double t3393;
  double t3394;
  double t3397;
  double t3427;
  double t3428;
  double t3429;
  double t3434;
  double t3445;
  double t3456;
  double t3465;
  double t3473;
  double t3474;
  double t3484;
  double t3546;
  double t3553;
  double t3555;
  double t3559;
  double t3560;
  double t3567;
  double t3600;
  double t3611;
  double t3616;
  double t3619;
  double t3636;
  double t3639;
  double t3649;
  double t3650;
  double t3651;
  double t3657;
  double t3665;
  double t3674;
  double t3679;
  double t3683;
  double t3684;
  double t3685;
  double t3713;
  double t3722;
  double t3723;
  double t3726;
  double t3727;
  double t3734;
  double t3785;
  double t3786;
  double t3789;
  double t3791;
  double t3792;
  double t3793;
  double t68;
  double t70;
  double t85;
  double t179;
  double t189;
  double t268;
  double t282;
  double t3888;
  double t3892;
  double t3897;
  double t284;
  double t291;
  double t293;
  double t305;
  double t3883;
  double t3884;
  double t3885;
  double t419;
  double t437;
  double t438;
  double t448;
  double t561;
  double t580;
  double t598;
  double t600;
  double t3912;
  double t3916;
  double t3918;
  double t3920;
  double t3922;
  double t3929;
  double t720;
  double t721;
  double t722;
  double t732;
  double t911;
  double t939;
  double t940;
  double t1083;
  double t1090;
  double t1094;
  double t3931;
  double t3937;
  double t3942;
  double t3959;
  double t3990;
  double t3993;
  double t4006;
  double t4013;
  double t4036;
  double t4037;
  double t4038;
  double t4045;
  double t1140;
  double t1147;
  double t1162;
  double t1168;
  double t1263;
  double t1298;
  double t1304;
  double t1348;
  double t1447;
  double t1485;
  double t1519;
  double t1521;
  double t4047;
  double t4059;
  double t4062;
  double t4064;
  double t4067;
  double t4068;
  double t4069;
  double t4072;
  double t4085;
  double t4086;
  double t4089;
  double t4090;
  double t1645;
  double t1677;
  double t1689;
  double t1692;
  double t1733;
  double t1747;
  double t1749;
  double t1878;
  double t1884;
  double t1896;
  double t4096;
  double t4104;
  double t4107;
  double t4108;
  double t4121;
  double t4122;
  double t4136;
  double t4141;
  double t4158;
  double t4160;
  double t4167;
  double t4168;
  double t2038;
  double t2058;
  double t2059;
  double t2066;
  double t2119;
  double t2140;
  double t2141;
  double t2170;
  double t2262;
  double t2273;
  double t2285;
  double t2290;
  double t4181;
  double t4190;
  double t4191;
  double t4197;
  double t4203;
  double t4216;
  double t4218;
  double t4219;
  double t4224;
  double t4227;
  double t4228;
  double t4229;
  double t2356;
  double t2357;
  double t2384;
  double t2386;
  double t2494;
  double t2499;
  double t2510;
  double t4238;
  double t4239;
  double t4241;
  double t4243;
  double t4246;
  double t4254;
  double t4255;
  double t4258;
  double t2620;
  double t2626;
  double t4265;
  double t4268;
  double t4269;
  double t4272;
  double t2662;
  double t2665;
  double t2701;
  double t2711;
  double t2767;
  double t2777;
  double t2780;
  double t2818;
  double t2833;
  double t2834;
  double t2835;
  double t2878;
  double t2888;
  double t2902;
  double t2906;
  double t4348;
  double t4354;
  double t4355;
  double t4363;
  double t4364;
  double t4369;
  double t2955;
  double t2974;
  double t3000;
  double t3003;
  double t3058;
  double t3070;
  double t3071;
  double t3083;
  double t3132;
  double t3133;
  double t3136;
  double t3154;
  double t3162;
  double t3166;
  double t4397;
  double t4399;
  double t4400;
  double t4406;
  double t4371;
  double t4375;
  double t4379;
  double t4381;
  double t4389;
  double t4391;
  double t4393;
  double t4394;
  double t3216;
  double t3217;
  double t3222;
  double t3225;
  double t3261;
  double t3263;
  double t3265;
  double t3266;
  double t3287;
  double t3299;
  double t3304;
  double t3307;
  double t4411;
  double t4412;
  double t4414;
  double t4416;
  double t4418;
  double t4419;
  double t4420;
  double t4422;
  double t4424;
  double t4426;
  double t4428;
  double t4429;
  double t3328;
  double t3332;
  double t3333;
  double t3342;
  double t3372;
  double t3373;
  double t3377;
  double t3430;
  double t3436;
  double t3440;
  double t4431;
  double t4432;
  double t4434;
  double t4438;
  double t4441;
  double t4442;
  double t4443;
  double t4448;
  double t4456;
  double t4457;
  double t4459;
  double t4463;
  double t3497;
  double t3506;
  double t3511;
  double t3528;
  double t3587;
  double t3589;
  double t3595;
  double t3599;
  double t3643;
  double t3656;
  double t3658;
  double t3660;
  double t4471;
  double t4472;
  double t4475;
  double t4476;
  double t4482;
  double t4484;
  double t4493;
  double t4494;
  double t4500;
  double t4501;
  double t4505;
  double t4506;
  double t3691;
  double t3697;
  double t3707;
  double t3708;
  double t3737;
  double t3738;
  double t3784;
  double t4517;
  double t4518;
  double t4519;
  double t4524;
  double t4529;
  double t4534;
  double t4537;
  double t4538;
  double t3815;
  double t3816;
  double t4545;
  double t4559;
  double t4564;
  double t4567;
  double t3831;
  double t3840;
  double t3856;
  double t3858;
  double t3876;
  double t90;
  double t172;
  double t247;
  double t283;
  double t405;
  double t543;
  double t714;
  double t910;
  double t1020;
  double t1139;
  double t1235;
  double t1439;
  double t1638;
  double t1731;
  double t1847;
  double t2009;
  double t2118;
  double t2231;
  double t2340;
  double t2481;
  double t2595;
  double t2604;
  double t2605;
  double t2627;
  double t2629;
  double t2648;
  double t2655;
  double t2667;
  double t2672;
  double t2675;
  double t2697;
  double t2720;
  double t2721;
  double t2729;
  double t2730;
  double t2742;
  double t2748;
  double t2793;
  double t2849;
  double t2908;
  double t3010;
  double t3098;
  double t3141;
  double t3215;
  double t3256;
  double t3278;
  double t3326;
  double t3371;
  double t3421;
  double t3486;
  double t3582;
  double t3640;
  double t3690;
  double t3735;
  double t3806;
  double t3809;
  double t3814;
  double t3819;
  double t3825;
  double t3826;
  double t3830;
  double t3842;
  double t3843;
  double t3849;
  double t3850;
  double t3861;
  double t3863;
  double t3865;
  double t3866;
  double t3868;
  double t3872;
  double t16;
  double t3878;
  double t3887;
  double t3898;
  double t3901;
  double t3919;
  double t3930;
  double t3960;
  double t4021;
  double t4046;
  double t4065;
  double t4084;
  double t4091;
  double t4118;
  double t4148;
  double t4180;
  double t4201;
  double t4223;
  double t4235;
  double t4244;
  double t4259;
  double t4274;
  double t4278;
  double t4279;
  double t4287;
  double t4291;
  double t4293;
  double t4303;
  double t4304;
  double t4306;
  double t4312;
  double t4316;
  double t4317;
  double t4318;
  double t4329;
  double t4334;
  double t4336;
  double t4340;
  double t4341;
  double t4361;
  double t4370;
  double t4382;
  double t4396;
  double t4409;
  double t4417;
  double t4423;
  double t4430;
  double t4439;
  double t4450;
  double t4470;
  double t4477;
  double t4495;
  double t4513;
  double t4526;
  double t4543;
  double t4571;
  double t4572;
  double t4573;
  double t4581;
  double t4582;
  double t4587;
  double t4593;
  double t4596;
  double t4597;
  double t4598;
  double t4601;
  double t4602;
  double t4603;
  double t4606;
  double t4607;
  double t4608;
  double t4611;
  double t4631;
  double t4632;
  double t4633;
  double t4641;
  double t4642;
  double t4643;
  double t4626;
  double t4636;
  double t4644;
  double t4651;
  double t4652;
  double t4653;
  double t4654;
  double t4655;
  double t4657;
  double t4658;
  double t4659;
  double t4675;
  double t4679;
  double t4680;
  double t4682;
  double t4686;
  double t4692;
  double t4696;
  double t4699;
  double t4703;
  double t4711;
  double t4712;
  double t4716;
  double t4723;
  double t4740;
  double t4741;
  double t4746;
  double t4747;
  double t4749;
  double t4758;
  double t4761;
  double t4766;
  double t4773;
  double t4780;
  double t4781;
  double t4783;
  double t4786;
  double t4788;
  double t4790;
  double t4793;
  double t4796;
  double t4803;
  double t4807;
  double t4808;
  double t4812;
  double t4813;
  double t4823;
  double t4838;
  double t4855;
  double t4872;
  double t4875;
  double t4885;
  double t4895;
  double t4896;
  double t4900;
  double t4904;
  double t4919;
  double t4921;
  double t4926;
  double t4927;
  t27 = Cos(var1[3]);
  t102 = Cos(var1[5]);
  t118 = Sin(var1[3]);
  t105 = Sin(var1[4]);
  t139 = Sin(var1[5]);
  t195 = -1.*t102*t118;
  t205 = t27*t105*t139;
  t243 = t195 + t205;
  t56 = Cos(var1[19]);
  t57 = -1.*t56;
  t67 = 1. + t57;
  t69 = Sin(var1[19]);
  t91 = Sin(var1[18]);
  t106 = t27*t102*t105;
  t140 = t118*t139;
  t170 = t106 + t140;
  t173 = Cos(var1[18]);
  t40 = Cos(var1[4]);
  t307 = -1.*t91*t170;
  t343 = t173*t243;
  t397 = t307 + t343;
  t450 = t173*t170;
  t494 = t91*t243;
  t531 = t450 + t494;
  t569 = Cos(var1[20]);
  t574 = -1.*t569;
  t577 = 1. + t574;
  t595 = Sin(var1[20]);
  t630 = -0.366501*t27*t40*t69;
  t634 = 0.340999127418*t67*t397;
  t645 = -0.134322983001*t67;
  t646 = 1. + t645;
  t683 = t646*t531;
  t698 = t630 + t634 + t683;
  t733 = 0.930418*t27*t40*t69;
  t838 = -0.8656776547239999*t67;
  t840 = 1. + t838;
  t842 = t840*t397;
  t871 = 0.340999127418*t67*t531;
  t880 = t733 + t842 + t871;
  t941 = -1.000000637725*t67;
  t958 = 1. + t941;
  t991 = t958*t27*t40;
  t1013 = -0.930418*t69*t397;
  t1017 = 0.366501*t69*t531;
  t1019 = t991 + t1013 + t1017;
  t1022 = Cos(var1[21]);
  t1065 = -1.*t1022;
  t1081 = 1. + t1065;
  t1088 = Sin(var1[21]);
  t1097 = 0.930418*t595*t698;
  t1108 = 0.366501*t595*t880;
  t1109 = -1.000000637725*t577;
  t1115 = 1. + t1109;
  t1116 = t1115*t1019;
  t1137 = t1097 + t1108 + t1116;
  t1172 = -0.8656776547239999*t577;
  t1174 = 1. + t1172;
  t1179 = t1174*t698;
  t1186 = -0.340999127418*t577*t880;
  t1208 = -0.930418*t595*t1019;
  t1224 = t1179 + t1186 + t1208;
  t1349 = -0.340999127418*t577*t698;
  t1361 = -0.134322983001*t577;
  t1366 = 1. + t1361;
  t1367 = t1366*t880;
  t1369 = -0.366501*t595*t1019;
  t1396 = t1349 + t1367 + t1369;
  t1448 = Cos(var1[22]);
  t1456 = -1.*t1448;
  t1467 = 1. + t1456;
  t1506 = Sin(var1[22]);
  t1531 = 0.366501*t1088*t1137;
  t1537 = -0.340999127418*t1081*t1224;
  t1556 = -0.134322983001*t1081;
  t1557 = 1. + t1556;
  t1577 = t1557*t1396;
  t1626 = t1531 + t1537 + t1577;
  t1700 = 0.930418*t1088*t1137;
  t1704 = -0.8656776547239999*t1081;
  t1720 = 1. + t1704;
  t1721 = t1720*t1224;
  t1726 = -0.340999127418*t1081*t1396;
  t1727 = t1700 + t1721 + t1726;
  t1750 = -1.000000637725*t1081;
  t1763 = 1. + t1750;
  t1799 = t1763*t1137;
  t1808 = -0.930418*t1088*t1224;
  t1821 = -0.366501*t1088*t1396;
  t1840 = t1799 + t1808 + t1821;
  t1855 = Cos(var1[23]);
  t1863 = -1.*t1855;
  t1873 = 1. + t1863;
  t1879 = Sin(var1[23]);
  t1911 = -0.366501*t1506*t1626;
  t1912 = -0.930418*t1506*t1727;
  t1918 = -1.000000637725*t1467;
  t1951 = 1. + t1918;
  t1958 = t1951*t1840;
  t1969 = t1911 + t1912 + t1958;
  t2085 = -0.134322983001*t1467;
  t2086 = 1. + t2085;
  t2087 = t2086*t1626;
  t2089 = -0.340999127418*t1467*t1727;
  t2109 = 0.366501*t1506*t1840;
  t2115 = t2087 + t2089 + t2109;
  t2178 = -0.340999127418*t1467*t1626;
  t2179 = -0.8656776547239999*t1467;
  t2182 = 1. + t2179;
  t2183 = t2182*t1727;
  t2191 = 0.930418*t1506*t1840;
  t2208 = t2178 + t2183 + t2191;
  t2263 = Cos(var1[24]);
  t2268 = -1.*t2263;
  t2271 = 1. + t2268;
  t2277 = Sin(var1[24]);
  t2297 = 0.930418*t1879*t1969;
  t2304 = -0.340999127418*t1873*t2115;
  t2321 = -0.8656776547239999*t1873;
  t2332 = 1. + t2321;
  t2334 = t2332*t2208;
  t2336 = t2297 + t2304 + t2334;
  t2406 = 0.366501*t1879*t1969;
  t2416 = -0.134322983001*t1873;
  t2422 = 1. + t2416;
  t2426 = t2422*t2115;
  t2455 = -0.340999127418*t1873*t2208;
  t2456 = t2406 + t2426 + t2455;
  t2517 = -1.000000637725*t1873;
  t2519 = 1. + t2517;
  t2529 = t2519*t1969;
  t2562 = -0.366501*t1879*t2115;
  t2565 = -0.930418*t1879*t2208;
  t2566 = t2529 + t2562 + t2565;
  t263 = Cos(var1[6]);
  t2746 = Sin(var1[6]);
  t2749 = Cos(var1[7]);
  t2752 = -1.*t2749;
  t2766 = 1. + t2752;
  t2775 = Sin(var1[7]);
  t2804 = t263*t243;
  t2808 = -1.*t170*t2746;
  t2809 = t2804 + t2808;
  t2850 = t263*t170;
  t2854 = t243*t2746;
  t2873 = t2850 + t2854;
  t2957 = Cos(var1[8]);
  t2968 = -1.*t2957;
  t2969 = 1. + t2968;
  t2990 = Sin(var1[8]);
  t3099 = -1.000000637725*t2766;
  t3102 = 1. + t3099;
  t3108 = t27*t40*t3102;
  t3123 = -0.930418*t2809*t2775;
  t3124 = -0.366501*t2873*t2775;
  t3127 = t3108 + t3123 + t3124;
  t2918 = -0.340999127418*t2766*t2809;
  t2926 = -0.134322983001*t2766;
  t2942 = 1. + t2926;
  t2943 = t2942*t2873;
  t2945 = 0.366501*t27*t40*t2775;
  t2950 = t2918 + t2943 + t2945;
  t3012 = -0.8656776547239999*t2766;
  t3013 = 1. + t3012;
  t3015 = t3013*t2809;
  t3030 = -0.340999127418*t2766*t2873;
  t3035 = 0.930418*t27*t40*t2775;
  t3047 = t3015 + t3030 + t3035;
  t3144 = Cos(var1[9]);
  t3145 = -1.*t3144;
  t3153 = 1. + t3145;
  t3156 = Sin(var1[9]);
  t3185 = -1.000000637725*t2969;
  t3203 = 1. + t3185;
  t3204 = t3203*t3127;
  t3207 = -0.930418*t2950*t2990;
  t3213 = 0.366501*t3047*t2990;
  t3214 = t3204 + t3207 + t3213;
  t3232 = 0.340999127418*t2969*t2950;
  t3242 = -0.134322983001*t2969;
  t3246 = 1. + t3242;
  t3249 = t3246*t3047;
  t3252 = -0.366501*t3127*t2990;
  t3255 = t3232 + t3249 + t3252;
  t3267 = -0.8656776547239999*t2969;
  t3268 = 1. + t3267;
  t3271 = t3268*t2950;
  t3272 = 0.340999127418*t2969*t3047;
  t3273 = 0.930418*t3127*t2990;
  t3274 = t3271 + t3272 + t3273;
  t3289 = Cos(var1[10]);
  t3292 = -1.*t3289;
  t3294 = 1. + t3292;
  t3303 = Sin(var1[10]);
  t3312 = -0.930418*t3156*t3214;
  t3313 = 0.340999127418*t3153*t3255;
  t3314 = -0.8656776547239999*t3153;
  t3317 = 1. + t3314;
  t3322 = t3317*t3274;
  t3325 = t3312 + t3313 + t3322;
  t3347 = 0.366501*t3156*t3214;
  t3349 = -0.134322983001*t3153;
  t3352 = 1. + t3349;
  t3353 = t3352*t3255;
  t3361 = 0.340999127418*t3153*t3274;
  t3363 = t3347 + t3353 + t3361;
  t3378 = -1.000000637725*t3153;
  t3379 = 1. + t3378;
  t3381 = t3379*t3214;
  t3393 = -0.366501*t3156*t3255;
  t3394 = 0.930418*t3156*t3274;
  t3397 = t3381 + t3393 + t3394;
  t3427 = Cos(var1[11]);
  t3428 = -1.*t3427;
  t3429 = 1. + t3428;
  t3434 = Sin(var1[11]);
  t3445 = 0.930418*t3303*t3325;
  t3456 = -0.366501*t3303*t3363;
  t3465 = -1.000000637725*t3294;
  t3473 = 1. + t3465;
  t3474 = t3473*t3397;
  t3484 = t3445 + t3456 + t3474;
  t3546 = -0.8656776547239999*t3294;
  t3553 = 1. + t3546;
  t3555 = t3553*t3325;
  t3559 = 0.340999127418*t3294*t3363;
  t3560 = -0.930418*t3303*t3397;
  t3567 = t3555 + t3559 + t3560;
  t3600 = 0.340999127418*t3294*t3325;
  t3611 = -0.134322983001*t3294;
  t3616 = 1. + t3611;
  t3619 = t3616*t3363;
  t3636 = 0.366501*t3303*t3397;
  t3639 = t3600 + t3619 + t3636;
  t3649 = Cos(var1[12]);
  t3650 = -1.*t3649;
  t3651 = 1. + t3650;
  t3657 = Sin(var1[12]);
  t3665 = 0.366501*t3434*t3484;
  t3674 = 0.340999127418*t3429*t3567;
  t3679 = -0.134322983001*t3429;
  t3683 = 1. + t3679;
  t3684 = t3683*t3639;
  t3685 = t3665 + t3674 + t3684;
  t3713 = -0.930418*t3434*t3484;
  t3722 = -0.8656776547239999*t3429;
  t3723 = 1. + t3722;
  t3726 = t3723*t3567;
  t3727 = 0.340999127418*t3429*t3639;
  t3734 = t3713 + t3726 + t3727;
  t3785 = -1.000000637725*t3429;
  t3786 = 1. + t3785;
  t3789 = t3786*t3484;
  t3791 = 0.930418*t3434*t3567;
  t3792 = -0.366501*t3434*t3639;
  t3793 = t3789 + t3791 + t3792;
  t68 = -0.04500040093286238*t67;
  t70 = 0.0846680539949003*t69;
  t85 = t68 + t70;
  t179 = -1.*t173;
  t189 = 1. + t179;
  t268 = -1.*t263;
  t282 = 1. + t268;
  t3888 = t27*t102;
  t3892 = t118*t105*t139;
  t3897 = t3888 + t3892;
  t284 = 1.296332362046933e-7*var1[19];
  t291 = -0.07877668146182712*t67;
  t293 = -0.04186915633414423*t69;
  t305 = t284 + t291 + t293;
  t3883 = t102*t118*t105;
  t3884 = -1.*t27*t139;
  t3885 = t3883 + t3884;
  t419 = 3.2909349868922137e-7*var1[19];
  t437 = 0.03103092645718495*t67;
  t438 = 0.016492681424499736*t69;
  t448 = t419 + t437 + t438;
  t561 = -1.296332362046933e-7*var1[20];
  t580 = -0.14128592423750855*t577;
  t598 = 0.04186915633414423*t595;
  t600 = t561 + t580 + t598;
  t3912 = -1.*t91*t3885;
  t3916 = t173*t3897;
  t3918 = t3912 + t3916;
  t3920 = t173*t3885;
  t3922 = t91*t3897;
  t3929 = t3920 + t3922;
  t720 = 3.2909349868922137e-7*var1[20];
  t721 = -0.055653945343889656*t577;
  t722 = 0.016492681424499736*t595;
  t732 = t720 + t721 + t722;
  t911 = -0.04500040093286238*t577;
  t939 = -0.15185209683981668*t595;
  t940 = t911 + t939;
  t1083 = 0.039853038461262744*t1081;
  t1090 = 0.23670515095269612*t1088;
  t1094 = t1083 + t1090;
  t3931 = -0.366501*t40*t69*t118;
  t3937 = 0.340999127418*t67*t3918;
  t3942 = t646*t3929;
  t3959 = t3931 + t3937 + t3942;
  t3990 = 0.930418*t40*t69*t118;
  t3993 = t840*t3918;
  t4006 = 0.340999127418*t67*t3929;
  t4013 = t3990 + t3993 + t4006;
  t4036 = t958*t40*t118;
  t4037 = -0.930418*t69*t3918;
  t4038 = 0.366501*t69*t3929;
  t4045 = t4036 + t4037 + t4038;
  t1140 = 6.295460977284962e-8*var1[21];
  t1147 = -0.22023473313910558*t1081;
  t1162 = 0.03707996069223323*t1088;
  t1168 = t1140 + t1147 + t1162;
  t1263 = -1.5981976069815686e-7*var1[21];
  t1298 = -0.08675267452931407*t1081;
  t1304 = 0.014606169134372047*t1088;
  t1348 = t1263 + t1298 + t1304;
  t1447 = -4.0833068682577724e-7*var1[22];
  t1485 = -0.11476729583292707*t1467;
  t1519 = 0.0111594154470601*t1506;
  t1521 = t1447 + t1485 + t1519;
  t4047 = 0.930418*t595*t3959;
  t4059 = 0.366501*t595*t4013;
  t4062 = t1115*t4045;
  t4064 = t4047 + t4059 + t4062;
  t4067 = t1174*t3959;
  t4068 = -0.340999127418*t577*t4013;
  t4069 = -0.930418*t595*t4045;
  t4072 = t4067 + t4068 + t4069;
  t4085 = -0.340999127418*t577*t3959;
  t4086 = t1366*t4013;
  t4089 = -0.366501*t595*t4045;
  t4090 = t4085 + t4086 + t4089;
  t1645 = 1.6084556086870008e-7*var1[22];
  t1677 = -0.29135406957765553*t1467;
  t1689 = 0.02832985722118838*t1506;
  t1692 = t1645 + t1677 + t1689;
  t1733 = 0.03044854601678662*t1467;
  t1747 = 0.3131431996991197*t1506;
  t1749 = t1733 + t1747;
  t1878 = -0.26285954081199375*t1873;
  t1884 = 0.634735404786378*t1879;
  t1896 = t1878 + t1884;
  t4096 = 0.366501*t1088*t4064;
  t4104 = -0.340999127418*t1081*t4072;
  t4107 = t1557*t4090;
  t4108 = t4096 + t4104 + t4107;
  t4121 = 0.930418*t1088*t4064;
  t4122 = t1720*t4072;
  t4136 = -0.340999127418*t1081*t4090;
  t4141 = t4121 + t4122 + t4136;
  t4158 = t1763*t4064;
  t4160 = -0.930418*t1088*t4072;
  t4167 = -0.366501*t1088*t4090;
  t4168 = t4158 + t4160 + t4167;
  t2038 = 1.6169269214444473e-7*var1[23];
  t2058 = -0.2326311605896123*t1873;
  t2059 = -0.09633822312984319*t1879;
  t2066 = t2038 + t2058 + t2059;
  t2119 = -6.369237629068993e-8*var1[23];
  t2140 = -0.5905692458505322*t1873;
  t2141 = -0.24456909227538925*t1879;
  t2170 = t2119 + t2140 + t2141;
  t2262 = -7.041766963257243e-8*var1[24];
  t2273 = -0.8232948486053725*t2271;
  t2285 = 0.05763710717422546*t2277;
  t2290 = t2262 + t2273 + t2285;
  t4181 = -0.366501*t1506*t4108;
  t4190 = -0.930418*t1506*t4141;
  t4191 = t1951*t4168;
  t4197 = t4181 + t4190 + t4191;
  t4203 = t2086*t4108;
  t4216 = -0.340999127418*t1467*t4141;
  t4218 = 0.366501*t1506*t4168;
  t4219 = t4203 + t4216 + t4218;
  t4224 = -0.340999127418*t1467*t4108;
  t4227 = t2182*t4141;
  t4228 = 0.930418*t1506*t4168;
  t4229 = t4224 + t4227 + t4228;
  t2356 = 1.7876586242383724e-7*var1[24];
  t2357 = -0.3243041141817093*t2271;
  t2384 = 0.02270383571304597*t2277;
  t2386 = t2356 + t2357 + t2384;
  t2494 = 0.06194758047549556*t2271;
  t2499 = 0.8848655643005321*t2277;
  t2510 = t2494 + t2499;
  t4238 = 0.930418*t1879*t4197;
  t4239 = -0.340999127418*t1873*t4219;
  t4241 = t2332*t4229;
  t4243 = t4238 + t4239 + t4241;
  t4246 = 0.366501*t1879*t4197;
  t4254 = t2422*t4219;
  t4255 = -0.340999127418*t1873*t4229;
  t4258 = t4246 + t4254 + t4255;
  t2620 = -1.000000637725*t2271;
  t2626 = 1. + t2620;
  t4265 = t2519*t4197;
  t4268 = -0.366501*t1879*t4219;
  t4269 = -0.930418*t1879*t4229;
  t4272 = t4265 + t4268 + t4269;
  t2662 = -0.134322983001*t2271;
  t2665 = 1. + t2662;
  t2701 = -0.8656776547239999*t2271;
  t2711 = 1. + t2701;
  t2767 = -0.04500040093286238*t2766;
  t2777 = -0.0846680539949003*t2775;
  t2780 = t2767 + t2777;
  t2818 = 1.296332362046933e-7*var1[7];
  t2833 = 0.07877668146182712*t2766;
  t2834 = -0.04186915633414423*t2775;
  t2835 = t2818 + t2833 + t2834;
  t2878 = -3.2909349868922137e-7*var1[7];
  t2888 = 0.03103092645718495*t2766;
  t2902 = -0.016492681424499736*t2775;
  t2906 = t2878 + t2888 + t2902;
  t4348 = t263*t3897;
  t4354 = -1.*t3885*t2746;
  t4355 = t4348 + t4354;
  t4363 = t263*t3885;
  t4364 = t3897*t2746;
  t4369 = t4363 + t4364;
  t2955 = 1.296332362046933e-7*var1[8];
  t2974 = -0.14128592423750855*t2969;
  t3000 = -0.04186915633414423*t2990;
  t3003 = t2955 + t2974 + t3000;
  t3058 = 3.2909349868922137e-7*var1[8];
  t3070 = 0.055653945343889656*t2969;
  t3071 = 0.016492681424499736*t2990;
  t3083 = t3058 + t3070 + t3071;
  t3132 = -0.04500040093286238*t2969;
  t3133 = 0.15185209683981668*t2990;
  t3136 = t3132 + t3133;
  t3154 = 0.039853038461262744*t3153;
  t3162 = -0.23670515095269612*t3156;
  t3166 = t3154 + t3162;
  t4397 = t40*t3102*t118;
  t4399 = -0.930418*t4355*t2775;
  t4400 = -0.366501*t4369*t2775;
  t4406 = t4397 + t4399 + t4400;
  t4371 = -0.340999127418*t2766*t4355;
  t4375 = t2942*t4369;
  t4379 = 0.366501*t40*t118*t2775;
  t4381 = t4371 + t4375 + t4379;
  t4389 = t3013*t4355;
  t4391 = -0.340999127418*t2766*t4369;
  t4393 = 0.930418*t40*t118*t2775;
  t4394 = t4389 + t4391 + t4393;
  t3216 = -1.5981976069815686e-7*var1[9];
  t3217 = 0.08675267452931407*t3153;
  t3222 = 0.014606169134372047*t3156;
  t3225 = t3216 + t3217 + t3222;
  t3261 = -6.295460977284962e-8*var1[9];
  t3263 = -0.22023473313910558*t3153;
  t3265 = -0.03707996069223323*t3156;
  t3266 = t3261 + t3263 + t3265;
  t3287 = -1.6084556086870008e-7*var1[10];
  t3299 = -0.29135406957765553*t3294;
  t3304 = -0.02832985722118838*t3303;
  t3307 = t3287 + t3299 + t3304;
  t4411 = t3203*t4406;
  t4412 = -0.930418*t4381*t2990;
  t4414 = 0.366501*t4394*t2990;
  t4416 = t4411 + t4412 + t4414;
  t4418 = 0.340999127418*t2969*t4381;
  t4419 = t3246*t4394;
  t4420 = -0.366501*t4406*t2990;
  t4422 = t4418 + t4419 + t4420;
  t4424 = t3268*t4381;
  t4426 = 0.340999127418*t2969*t4394;
  t4428 = 0.930418*t4406*t2990;
  t4429 = t4424 + t4426 + t4428;
  t3328 = -4.0833068682577724e-7*var1[10];
  t3332 = 0.11476729583292707*t3294;
  t3333 = 0.0111594154470601*t3303;
  t3342 = t3328 + t3332 + t3333;
  t3372 = 0.03044854601678662*t3294;
  t3373 = -0.3131431996991197*t3303;
  t3377 = t3372 + t3373;
  t3430 = -0.26285954081199375*t3429;
  t3436 = -0.634735404786378*t3434;
  t3440 = t3430 + t3436;
  t4431 = -0.930418*t3156*t4416;
  t4432 = 0.340999127418*t3153*t4422;
  t4434 = t3317*t4429;
  t4438 = t4431 + t4432 + t4434;
  t4441 = 0.366501*t3156*t4416;
  t4442 = t3352*t4422;
  t4443 = 0.340999127418*t3153*t4429;
  t4448 = t4441 + t4442 + t4443;
  t4456 = t3379*t4416;
  t4457 = -0.366501*t3156*t4422;
  t4459 = 0.930418*t3156*t4429;
  t4463 = t4456 + t4457 + t4459;
  t3497 = 6.369237629068993e-8*var1[11];
  t3506 = -0.5905692458505322*t3429;
  t3511 = 0.24456909227538925*t3434;
  t3528 = t3497 + t3506 + t3511;
  t3587 = 1.6169269214444473e-7*var1[11];
  t3589 = 0.2326311605896123*t3429;
  t3595 = -0.09633822312984319*t3434;
  t3599 = t3587 + t3589 + t3595;
  t3643 = 1.7876586242383724e-7*var1[12];
  t3656 = 0.3243041141817093*t3651;
  t3658 = 0.02270383571304597*t3657;
  t3660 = t3643 + t3656 + t3658;
  t4471 = 0.930418*t3303*t4438;
  t4472 = -0.366501*t3303*t4448;
  t4475 = t3473*t4463;
  t4476 = t4471 + t4472 + t4475;
  t4482 = t3553*t4438;
  t4484 = 0.340999127418*t3294*t4448;
  t4493 = -0.930418*t3303*t4463;
  t4494 = t4482 + t4484 + t4493;
  t4500 = 0.340999127418*t3294*t4438;
  t4501 = t3616*t4448;
  t4505 = 0.366501*t3303*t4463;
  t4506 = t4500 + t4501 + t4505;
  t3691 = 7.041766963257243e-8*var1[12];
  t3697 = -0.8232948486053725*t3651;
  t3707 = -0.05763710717422546*t3657;
  t3708 = t3691 + t3697 + t3707;
  t3737 = 0.06194758047549556*t3651;
  t3738 = -0.8848655643005321*t3657;
  t3784 = t3737 + t3738;
  t4517 = 0.366501*t3434*t4476;
  t4518 = 0.340999127418*t3429*t4494;
  t4519 = t3683*t4506;
  t4524 = t4517 + t4518 + t4519;
  t4529 = -0.930418*t3434*t4476;
  t4534 = t3723*t4494;
  t4537 = 0.340999127418*t3429*t4506;
  t4538 = t4529 + t4534 + t4537;
  t3815 = -1.000000637725*t3651;
  t3816 = 1. + t3815;
  t4545 = t3786*t4476;
  t4559 = 0.930418*t3434*t4494;
  t4564 = -0.366501*t3434*t4506;
  t4567 = t4545 + t4559 + t4564;
  t3831 = -0.8656776547239999*t3651;
  t3840 = 1. + t3831;
  t3856 = -0.134322983001*t3651;
  t3858 = 1. + t3856;
  t3876 = Sin(var2[0]);
  t90 = t27*t40*t85;
  t172 = -0.091*t91*t170;
  t247 = -0.091*t189*t243;
  t283 = -0.091*t282*t243;
  t405 = t305*t397;
  t543 = t448*t531;
  t714 = t600*t698;
  t910 = t732*t880;
  t1020 = t940*t1019;
  t1139 = t1094*t1137;
  t1235 = t1168*t1224;
  t1439 = t1348*t1396;
  t1638 = t1521*t1626;
  t1731 = t1692*t1727;
  t1847 = t1749*t1840;
  t2009 = t1896*t1969;
  t2118 = t2066*t2115;
  t2231 = t2170*t2208;
  t2340 = t2290*t2336;
  t2481 = t2386*t2456;
  t2595 = t2510*t2566;
  t2604 = -0.930418*t2277*t2336;
  t2605 = -0.366501*t2277*t2456;
  t2627 = t2626*t2566;
  t2629 = t2604 + t2605 + t2627;
  t2648 = 0.061947*t2629;
  t2655 = -0.340999127418*t2271*t2336;
  t2667 = t2665*t2456;
  t2672 = 0.366501*t2277*t2566;
  t2675 = t2655 + t2667 + t2672;
  t2697 = -0.402615*t2675;
  t2720 = t2711*t2336;
  t2721 = -0.340999127418*t2271*t2456;
  t2729 = 0.930418*t2277*t2566;
  t2730 = t2720 + t2721 + t2729;
  t2742 = -0.792446*t2730;
  t2748 = -0.091*t170*t2746;
  t2793 = -1.*t27*t40*t2780;
  t2849 = -1.*t2809*t2835;
  t2908 = -1.*t2873*t2906;
  t3010 = -1.*t2950*t3003;
  t3098 = -1.*t3047*t3083;
  t3141 = -1.*t3127*t3136;
  t3215 = -1.*t3166*t3214;
  t3256 = -1.*t3225*t3255;
  t3278 = -1.*t3266*t3274;
  t3326 = -1.*t3307*t3325;
  t3371 = -1.*t3342*t3363;
  t3421 = -1.*t3377*t3397;
  t3486 = -1.*t3440*t3484;
  t3582 = -1.*t3528*t3567;
  t3640 = -1.*t3599*t3639;
  t3690 = -1.*t3660*t3685;
  t3735 = -1.*t3708*t3734;
  t3806 = -1.*t3784*t3793;
  t3809 = -0.366501*t3657*t3685;
  t3814 = 0.930418*t3657*t3734;
  t3819 = t3816*t3793;
  t3825 = t3809 + t3814 + t3819;
  t3826 = -0.061947*t3825;
  t3830 = 0.340999127418*t3651*t3685;
  t3842 = t3840*t3734;
  t3843 = -0.930418*t3657*t3793;
  t3849 = t3830 + t3842 + t3843;
  t3850 = 0.792446*t3849;
  t3861 = t3858*t3685;
  t3863 = 0.340999127418*t3651*t3734;
  t3865 = 0.366501*t3657*t3793;
  t3866 = t3861 + t3863 + t3865;
  t3868 = -0.402615*t3866;
  t3872 = t90 + t172 + t247 + t283 + t405 + t543 + t714 + t910 + t1020 + t1139 + t1235 + t1439 + t1638 + t1731 + t1847 + t2009 + t2118 + t2231 + t2340 + t2481 + t2595 + t2648 + t2697 + t2742 + t2748 + t2793 + t2849 + t2908 + t3010 + t3098 + t3141 + t3215 + t3256 + t3278 + t3326 + t3371 + t3421 + t3486 + t3582 + t3640 + t3690 + t3735 + t3806 + t3826 + t3850 + t3868;
  t16 = Cos(var2[0]);
  t3878 = t40*t85*t118;
  t3887 = -0.091*t91*t3885;
  t3898 = -0.091*t189*t3897;
  t3901 = -0.091*t282*t3897;
  t3919 = t305*t3918;
  t3930 = t448*t3929;
  t3960 = t600*t3959;
  t4021 = t732*t4013;
  t4046 = t940*t4045;
  t4065 = t1094*t4064;
  t4084 = t1168*t4072;
  t4091 = t1348*t4090;
  t4118 = t1521*t4108;
  t4148 = t1692*t4141;
  t4180 = t1749*t4168;
  t4201 = t1896*t4197;
  t4223 = t2066*t4219;
  t4235 = t2170*t4229;
  t4244 = t2290*t4243;
  t4259 = t2386*t4258;
  t4274 = t2510*t4272;
  t4278 = -0.930418*t2277*t4243;
  t4279 = -0.366501*t2277*t4258;
  t4287 = t2626*t4272;
  t4291 = t4278 + t4279 + t4287;
  t4293 = 0.061947*t4291;
  t4303 = -0.340999127418*t2271*t4243;
  t4304 = t2665*t4258;
  t4306 = 0.366501*t2277*t4272;
  t4312 = t4303 + t4304 + t4306;
  t4316 = -0.402615*t4312;
  t4317 = t2711*t4243;
  t4318 = -0.340999127418*t2271*t4258;
  t4329 = 0.930418*t2277*t4272;
  t4334 = t4317 + t4318 + t4329;
  t4336 = -0.792446*t4334;
  t4340 = -0.091*t3885*t2746;
  t4341 = -1.*t40*t118*t2780;
  t4361 = -1.*t4355*t2835;
  t4370 = -1.*t4369*t2906;
  t4382 = -1.*t4381*t3003;
  t4396 = -1.*t4394*t3083;
  t4409 = -1.*t4406*t3136;
  t4417 = -1.*t3166*t4416;
  t4423 = -1.*t3225*t4422;
  t4430 = -1.*t3266*t4429;
  t4439 = -1.*t3307*t4438;
  t4450 = -1.*t3342*t4448;
  t4470 = -1.*t3377*t4463;
  t4477 = -1.*t3440*t4476;
  t4495 = -1.*t3528*t4494;
  t4513 = -1.*t3599*t4506;
  t4526 = -1.*t3660*t4524;
  t4543 = -1.*t3708*t4538;
  t4571 = -1.*t3784*t4567;
  t4572 = -0.366501*t3657*t4524;
  t4573 = 0.930418*t3657*t4538;
  t4581 = t3816*t4567;
  t4582 = t4572 + t4573 + t4581;
  t4587 = -0.061947*t4582;
  t4593 = 0.340999127418*t3651*t4524;
  t4596 = t3840*t4538;
  t4597 = -0.930418*t3657*t4567;
  t4598 = t4593 + t4596 + t4597;
  t4601 = 0.792446*t4598;
  t4602 = t3858*t4524;
  t4603 = 0.340999127418*t3651*t4538;
  t4606 = 0.366501*t3657*t4567;
  t4607 = t4602 + t4603 + t4606;
  t4608 = -0.402615*t4607;
  t4611 = t3878 + t3887 + t3898 + t3901 + t3919 + t3930 + t3960 + t4021 + t4046 + t4065 + t4084 + t4091 + t4118 + t4148 + t4180 + t4201 + t4223 + t4235 + t4244 + t4259 + t4274 + t4293 + t4316 + t4336 + t4340 + t4341 + t4361 + t4370 + t4382 + t4396 + t4409 + t4417 + t4423 + t4430 + t4439 + t4450 + t4470 + t4477 + t4495 + t4513 + t4526 + t4543 + t4571 + t4587 + t4601 + t4608;
  t4631 = -1.*t40*t102*t91;
  t4632 = t173*t40*t139;
  t4633 = t4631 + t4632;
  t4641 = t173*t40*t102;
  t4642 = t40*t91*t139;
  t4643 = t4641 + t4642;
  t4626 = 0.366501*t69*t105;
  t4636 = 0.340999127418*t67*t4633;
  t4644 = t646*t4643;
  t4651 = t4626 + t4636 + t4644;
  t4652 = 0.930418*t595*t4651;
  t4653 = -0.930418*t69*t105;
  t4654 = t840*t4633;
  t4655 = 0.340999127418*t67*t4643;
  t4657 = t4653 + t4654 + t4655;
  t4658 = 0.366501*t595*t4657;
  t4659 = -1.*t958*t105;
  t4675 = -0.930418*t69*t4633;
  t4679 = 0.366501*t69*t4643;
  t4680 = t4659 + t4675 + t4679;
  t4682 = t1115*t4680;
  t4686 = t4652 + t4658 + t4682;
  t4692 = t1174*t4651;
  t4696 = -0.340999127418*t577*t4657;
  t4699 = -0.930418*t595*t4680;
  t4703 = t4692 + t4696 + t4699;
  t4711 = -0.340999127418*t577*t4651;
  t4712 = t1366*t4657;
  t4716 = -0.366501*t595*t4680;
  t4723 = t4711 + t4712 + t4716;
  t4740 = 0.366501*t1088*t4686;
  t4741 = -0.340999127418*t1081*t4703;
  t4746 = t1557*t4723;
  t4747 = t4740 + t4741 + t4746;
  t4749 = 0.930418*t1088*t4686;
  t4758 = t1720*t4703;
  t4761 = -0.340999127418*t1081*t4723;
  t4766 = t4749 + t4758 + t4761;
  t4773 = t1763*t4686;
  t4780 = -0.930418*t1088*t4703;
  t4781 = -0.366501*t1088*t4723;
  t4783 = t4773 + t4780 + t4781;
  t4786 = -0.366501*t1506*t4747;
  t4788 = -0.930418*t1506*t4766;
  t4790 = t1951*t4783;
  t4793 = t4786 + t4788 + t4790;
  t4796 = t2086*t4747;
  t4803 = -0.340999127418*t1467*t4766;
  t4807 = 0.366501*t1506*t4783;
  t4808 = t4796 + t4803 + t4807;
  t4812 = -0.340999127418*t1467*t4747;
  t4813 = t2182*t4766;
  t4823 = 0.930418*t1506*t4783;
  t4838 = t4812 + t4813 + t4823;
  t4855 = 0.930418*t1879*t4793;
  t4872 = -0.340999127418*t1873*t4808;
  t4875 = t2332*t4838;
  t4885 = t4855 + t4872 + t4875;
  t4895 = 0.366501*t1879*t4793;
  t4896 = t2422*t4808;
  t4900 = -0.340999127418*t1873*t4838;
  t4904 = t4895 + t4896 + t4900;
  t4919 = t2519*t4793;
  t4921 = -0.366501*t1879*t4808;
  t4926 = -0.930418*t1879*t4838;
  t4927 = t4919 + t4921 + t4926;
  p_output1[0]=t16*t3872 + t3876*t4611;
  p_output1[1]=-1.*t3872*t3876 + t16*t4611;
  p_output1[2]=Sqrt(Power(0. - 0.045*t1137 - 1.*t1094*t1137 - 0.108789*t1224 - 1.*t1168*t1224 - 0.138152*t1396 - 1.*t1348*t1396 - 1.*t1521*t1626 - 1.*t1692*t1727 - 1.*t1749*t1840 - 1.*t1896*t1969 - 1.*t2066*t2115 - 1.*t2170*t2208 - 1.*t2290*t2336 - 1.*t2386*t2456 - 1.*t2510*t2566 - 0.061947*t2629 + 0.402615*t2675 + 0.792446*t2730,2) + Power(0. - 0.045*t4064 - 1.*t1094*t4064 - 0.108789*t4072 - 1.*t1168*t4072 - 0.138152*t4090 - 1.*t1348*t4090 - 1.*t1521*t4108 - 1.*t1692*t4141 - 1.*t1749*t4168 - 1.*t1896*t4197 - 1.*t2066*t4219 - 1.*t2170*t4229 - 1.*t2290*t4243 - 1.*t2386*t4258 - 1.*t2510*t4272 - 0.061947*t4291 + 0.402615*t4312 + 0.792446*t4334,2) + Power(0. - 0.045*t4686 - 1.*t1094*t4686 - 0.108789*t4703 - 1.*t1168*t4703 - 0.138152*t4723 - 1.*t1348*t4723 - 1.*t1521*t4747 - 1.*t1692*t4766 - 1.*t1749*t4783 - 1.*t1896*t4793 - 1.*t2066*t4808 - 1.*t2170*t4838 - 1.*t2290*t4885 - 1.*t2386*t4904 - 1.*t2510*t4927 + 0.402615*(-0.340999127418*t2271*t4885 + t2665*t4904 + 0.366501*t2277*t4927) + 0.792446*(t2711*t4885 - 0.340999127418*t2271*t4904 + 0.930418*t2277*t4927) - 0.061947*(-0.930418*t2277*t4885 - 0.366501*t2277*t4904 + t2626*t4927),2));
}



void gen::kin::y_vc_stand_v2(Eigen::Ref<Eigen::VectorXd> p_output1, const Eigen::Ref<const Eigen::VectorXd> var1,const Eigen::Ref<const Eigen::VectorXd> var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}