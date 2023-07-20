/*
 * Automatically Generated from Mathematica.
 * Wed 27 Oct 2021 22:54:12 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "gen/kin/p_hip_flexion_left.hh"

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
  double t485;
  double t477;
  double t487;
  double t483;
  double t490;
  double t464;
  double t484;
  double t492;
  double t504;
  double t509;
  double t510;
  double t520;
  double t522;
  double t531;
  double t532;
  double t533;
  double t537;
  double t548;
  double t549;
  double t550;
  double t560;
  double t561;
  double t564;
  double t527;
  double t590;
  double t593;
  double t596;
  double t599;
  double t625;
  double t628;
  double t629;
  double t630;
  double t631;
  double t632;
  double t582;
  double t583;
  double t584;
  double t585;
  double t586;
  double t587;
  double t607;
  double t609;
  double t611;
  double t612;
  double t615;
  double t616;
  double t471;
  double t475;
  double t536;
  double t538;
  double t539;
  double t663;
  double t664;
  double t667;
  double t669;
  double t670;
  double t672;
  double t552;
  double t553;
  double t554;
  double t556;
  double t566;
  double t567;
  double t575;
  double t577;
  double t675;
  double t676;
  double t677;
  double t680;
  double t681;
  double t682;
  double t589;
  double t598;
  double t604;
  double t605;
  double t617;
  double t618;
  double t620;
  double t623;
  double t633;
  double t634;
  double t635;
  double t637;
  double t638;
  double t700;
  double t701;
  double t702;
  double t703;
  double t686;
  double t687;
  double t688;
  double t689;
  double t693;
  double t694;
  double t695;
  double t698;
  double t648;
  double t649;
  double t655;
  double t656;
  double t724;
  double t725;
  double t726;
  double t728;
  double t729;
  double t730;
  double t742;
  double t743;
  double t744;
  double t745;
  double t737;
  double t738;
  double t739;
  double t740;
  double t732;
  double t733;
  double t734;
  double t735;
  t485 = Cos(var1[3]);
  t477 = Cos(var1[5]);
  t487 = Sin(var1[4]);
  t483 = Sin(var1[3]);
  t490 = Sin(var1[5]);
  t464 = Cos(var1[6]);
  t484 = -1.*t477*t483;
  t492 = t485*t487*t490;
  t504 = t484 + t492;
  t509 = t485*t477*t487;
  t510 = t483*t490;
  t520 = t509 + t510;
  t522 = Sin(var1[6]);
  t531 = Cos(var1[7]);
  t532 = -1.*t531;
  t533 = 1. + t532;
  t537 = Sin(var1[7]);
  t548 = t464*t504;
  t549 = -1.*t520*t522;
  t550 = t548 + t549;
  t560 = t464*t520;
  t561 = t504*t522;
  t564 = t560 + t561;
  t527 = Cos(var1[4]);
  t590 = Cos(var1[8]);
  t593 = -1.*t590;
  t596 = 1. + t593;
  t599 = Sin(var1[8]);
  t625 = -1.000000637725*t533;
  t628 = 1. + t625;
  t629 = t485*t527*t628;
  t630 = -0.930418*t550*t537;
  t631 = -0.366501*t564*t537;
  t632 = t629 + t630 + t631;
  t582 = -0.340999127418*t533*t550;
  t583 = -0.134322983001*t533;
  t584 = 1. + t583;
  t585 = t584*t564;
  t586 = 0.366501*t485*t527*t537;
  t587 = t582 + t585 + t586;
  t607 = -0.8656776547239999*t533;
  t609 = 1. + t607;
  t611 = t609*t550;
  t612 = -0.340999127418*t533*t564;
  t615 = 0.930418*t485*t527*t537;
  t616 = t611 + t612 + t615;
  t471 = -1.*t464;
  t475 = 1. + t471;
  t536 = -0.04500040093286238*t533;
  t538 = -0.0846680539949003*t537;
  t539 = t536 + t538;
  t663 = t485*t477;
  t664 = t483*t487*t490;
  t667 = t663 + t664;
  t669 = t477*t483*t487;
  t670 = -1.*t485*t490;
  t672 = t669 + t670;
  t552 = 1.296332362046933e-7*var1[7];
  t553 = 0.07877668146182712*t533;
  t554 = -0.04186915633414423*t537;
  t556 = t552 + t553 + t554;
  t566 = -3.2909349868922137e-7*var1[7];
  t567 = 0.03103092645718495*t533;
  t575 = -0.016492681424499736*t537;
  t577 = t566 + t567 + t575;
  t675 = t464*t667;
  t676 = -1.*t672*t522;
  t677 = t675 + t676;
  t680 = t464*t672;
  t681 = t667*t522;
  t682 = t680 + t681;
  t589 = 1.296332362046933e-7*var1[8];
  t598 = -0.14128592423750855*t596;
  t604 = -0.04186915633414423*t599;
  t605 = t589 + t598 + t604;
  t617 = 3.2909349868922137e-7*var1[8];
  t618 = 0.055653945343889656*t596;
  t620 = 0.016492681424499736*t599;
  t623 = t617 + t618 + t620;
  t633 = -0.04500040093286238*t596;
  t634 = 0.15185209683981668*t599;
  t635 = t633 + t634;
  t637 = -1.000000637725*t596;
  t638 = 1. + t637;
  t700 = t527*t628*t483;
  t701 = -0.930418*t677*t537;
  t702 = -0.366501*t682*t537;
  t703 = t700 + t701 + t702;
  t686 = -0.340999127418*t533*t677;
  t687 = t584*t682;
  t688 = 0.366501*t527*t483*t537;
  t689 = t686 + t687 + t688;
  t693 = t609*t677;
  t694 = -0.340999127418*t533*t682;
  t695 = 0.930418*t527*t483*t537;
  t698 = t693 + t694 + t695;
  t648 = -0.134322983001*t596;
  t649 = 1. + t648;
  t655 = -0.8656776547239999*t596;
  t656 = 1. + t655;
  t724 = t527*t464*t490;
  t725 = -1.*t527*t477*t522;
  t726 = t724 + t725;
  t728 = t527*t477*t464;
  t729 = t527*t490*t522;
  t730 = t728 + t729;
  t742 = -1.*t628*t487;
  t743 = -0.930418*t726*t537;
  t744 = -0.366501*t730*t537;
  t745 = t742 + t743 + t744;
  t737 = t609*t726;
  t738 = -0.340999127418*t533*t730;
  t739 = -0.930418*t487*t537;
  t740 = t737 + t738 + t739;
  t732 = -0.340999127418*t533*t726;
  t733 = t584*t730;
  t734 = -0.366501*t487*t537;
  t735 = t732 + t733 + t734;
  p_output1[0]=0.091*t475*t504 + 0.091*t520*t522 + t485*t527*t539 + t550*t556 + t564*t577 + t587*t605 + t616*t623 + t632*t635 - 0.045*(-0.930418*t587*t599 + 0.366501*t599*t616 + t632*t638) + 0.138152*(0.340999127418*t587*t596 - 0.366501*t599*t632 + t616*t649) - 0.108789*(0.340999127418*t596*t616 + 0.930418*t599*t632 + t587*t656) + var1[0];
  p_output1[1]=t483*t527*t539 + 0.091*t475*t667 + 0.091*t522*t672 + t556*t677 + t577*t682 + t605*t689 + t623*t698 + t635*t703 + 0.138152*(0.340999127418*t596*t689 + t649*t698 - 0.366501*t599*t703) - 0.108789*(t656*t689 + 0.340999127418*t596*t698 + 0.930418*t599*t703) - 0.045*(-0.930418*t599*t689 + 0.366501*t599*t698 + t638*t703) + var1[1];
  p_output1[2]=0.091*t475*t490*t527 + 0.091*t477*t522*t527 - 1.*t487*t539 + t556*t726 + t577*t730 + t605*t735 + t623*t740 + t635*t745 + 0.138152*(0.340999127418*t596*t735 + t649*t740 - 0.366501*t599*t745) - 0.108789*(t656*t735 + 0.340999127418*t596*t740 + 0.930418*t599*t745) - 0.045*(-0.930418*t599*t735 + 0.366501*t599*t740 + t638*t745) + var1[2];
}



void gen::kin::p_hip_flexion_left(Eigen::Ref<Eigen::VectorXd> p_output1, const Eigen::Ref<const Eigen::VectorXd> var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
