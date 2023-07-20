/*
 * Automatically Generated from Mathematica.
 * Wed 27 Oct 2021 23:35:03 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "gen/dyn/InertiaMatrix_sum_26.hh"

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
  double _NotUsed;
  NULL;
  p_output1[0]=0;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=0;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=0;
  p_output1[9]=0;
  p_output1[10]=0;
  p_output1[11]=0;
  p_output1[12]=0;
  p_output1[13]=0;
  p_output1[14]=0;
  p_output1[15]=0;
  p_output1[16]=0;
  p_output1[17]=0;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=0;
  p_output1[31]=0;
  p_output1[32]=0;
  p_output1[33]=0;
  p_output1[34]=0;
  p_output1[35]=0;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
  p_output1[54]=0;
  p_output1[55]=0;
  p_output1[56]=0;
  p_output1[57]=0;
  p_output1[58]=0;
  p_output1[59]=0;
  p_output1[60]=0;
  p_output1[61]=0;
  p_output1[62]=0;
  p_output1[63]=0;
  p_output1[64]=0;
  p_output1[65]=0;
  p_output1[66]=0;
  p_output1[67]=0;
  p_output1[68]=0;
  p_output1[69]=0;
  p_output1[70]=0;
  p_output1[71]=0;
  p_output1[72]=0;
  p_output1[73]=0;
  p_output1[74]=0;
  p_output1[75]=0;
  p_output1[76]=0;
  p_output1[77]=0;
  p_output1[78]=0;
  p_output1[79]=0;
  p_output1[80]=0;
  p_output1[81]=0;
  p_output1[82]=0;
  p_output1[83]=0;
  p_output1[84]=0;
  p_output1[85]=0;
  p_output1[86]=0;
  p_output1[87]=0;
  p_output1[88]=0;
  p_output1[89]=0;
  p_output1[90]=0;
  p_output1[91]=0;
  p_output1[92]=0;
  p_output1[93]=0;
  p_output1[94]=0;
  p_output1[95]=0;
  p_output1[96]=0;
  p_output1[97]=0;
  p_output1[98]=0;
  p_output1[99]=0;
  p_output1[100]=0;
  p_output1[101]=0;
  p_output1[102]=0;
  p_output1[103]=0;
  p_output1[104]=0;
  p_output1[105]=0;
  p_output1[106]=0;
  p_output1[107]=0;
  p_output1[108]=0;
  p_output1[109]=0;
  p_output1[110]=0;
  p_output1[111]=0;
  p_output1[112]=0;
  p_output1[113]=0;
  p_output1[114]=0;
  p_output1[115]=0;
  p_output1[116]=0;
  p_output1[117]=0;
  p_output1[118]=0;
  p_output1[119]=0;
  p_output1[120]=0;
  p_output1[121]=0;
  p_output1[122]=0;
  p_output1[123]=0;
  p_output1[124]=0;
  p_output1[125]=0;
  p_output1[126]=0;
  p_output1[127]=0;
  p_output1[128]=0;
  p_output1[129]=0;
  p_output1[130]=0;
  p_output1[131]=0;
  p_output1[132]=0;
  p_output1[133]=0;
  p_output1[134]=0;
  p_output1[135]=0;
  p_output1[136]=0;
  p_output1[137]=0;
  p_output1[138]=0;
  p_output1[139]=0;
  p_output1[140]=0;
  p_output1[141]=0;
  p_output1[142]=0;
  p_output1[143]=0;
  p_output1[144]=0;
  p_output1[145]=0;
  p_output1[146]=0;
  p_output1[147]=0;
  p_output1[148]=0;
  p_output1[149]=0;
  p_output1[150]=0;
  p_output1[151]=0;
  p_output1[152]=0;
  p_output1[153]=0;
  p_output1[154]=0;
  p_output1[155]=0;
  p_output1[156]=0;
  p_output1[157]=0;
  p_output1[158]=0;
  p_output1[159]=0;
  p_output1[160]=0;
  p_output1[161]=0;
  p_output1[162]=0;
  p_output1[163]=0;
  p_output1[164]=0;
  p_output1[165]=0;
  p_output1[166]=0;
  p_output1[167]=0;
  p_output1[168]=0;
  p_output1[169]=0;
  p_output1[170]=0;
  p_output1[171]=0;
  p_output1[172]=0;
  p_output1[173]=0;
  p_output1[174]=0;
  p_output1[175]=0;
  p_output1[176]=0;
  p_output1[177]=0;
  p_output1[178]=0;
  p_output1[179]=0;
  p_output1[180]=0;
  p_output1[181]=0;
  p_output1[182]=0;
  p_output1[183]=0;
  p_output1[184]=0;
  p_output1[185]=0;
  p_output1[186]=0;
  p_output1[187]=0;
  p_output1[188]=0;
  p_output1[189]=0;
  p_output1[190]=0;
  p_output1[191]=0;
  p_output1[192]=0;
  p_output1[193]=0;
  p_output1[194]=0;
  p_output1[195]=0;
  p_output1[196]=0;
  p_output1[197]=0;
  p_output1[198]=0;
  p_output1[199]=0;
  p_output1[200]=0;
  p_output1[201]=0;
  p_output1[202]=0;
  p_output1[203]=0;
  p_output1[204]=0;
  p_output1[205]=0;
  p_output1[206]=0;
  p_output1[207]=0;
  p_output1[208]=0;
  p_output1[209]=0;
  p_output1[210]=0;
  p_output1[211]=0;
  p_output1[212]=0;
  p_output1[213]=0;
  p_output1[214]=0;
  p_output1[215]=0;
  p_output1[216]=0;
  p_output1[217]=0;
  p_output1[218]=0;
  p_output1[219]=0;
  p_output1[220]=0;
  p_output1[221]=0;
  p_output1[222]=0;
  p_output1[223]=0;
  p_output1[224]=0;
  p_output1[225]=0;
  p_output1[226]=0;
  p_output1[227]=0;
  p_output1[228]=0;
  p_output1[229]=0;
  p_output1[230]=0;
  p_output1[231]=0;
  p_output1[232]=0.128;
  p_output1[233]=0;
  p_output1[234]=0;
  p_output1[235]=0;
  p_output1[236]=0;
  p_output1[237]=0;
  p_output1[238]=0;
  p_output1[239]=0;
  p_output1[240]=0;
  p_output1[241]=0;
  p_output1[242]=0;
  p_output1[243]=0;
  p_output1[244]=0;
  p_output1[245]=0;
  p_output1[246]=0;
  p_output1[247]=0;
  p_output1[248]=0;
  p_output1[249]=0;
  p_output1[250]=0;
  p_output1[251]=0;
  p_output1[252]=0;
  p_output1[253]=0;
  p_output1[254]=0;
  p_output1[255]=0;
  p_output1[256]=0;
  p_output1[257]=0;
  p_output1[258]=0;
  p_output1[259]=0;
  p_output1[260]=0;
  p_output1[261]=0.128;
  p_output1[262]=0;
  p_output1[263]=0;
  p_output1[264]=0;
  p_output1[265]=0;
  p_output1[266]=0;
  p_output1[267]=0;
  p_output1[268]=0;
  p_output1[269]=0;
  p_output1[270]=0;
  p_output1[271]=0;
  p_output1[272]=0;
  p_output1[273]=0;
  p_output1[274]=0;
  p_output1[275]=0;
  p_output1[276]=0;
  p_output1[277]=0;
  p_output1[278]=0;
  p_output1[279]=0;
  p_output1[280]=0;
  p_output1[281]=0;
  p_output1[282]=0;
  p_output1[283]=0;
  p_output1[284]=0;
  p_output1[285]=0;
  p_output1[286]=0;
  p_output1[287]=0;
  p_output1[288]=0;
  p_output1[289]=0;
  p_output1[290]=0;
  p_output1[291]=0;
  p_output1[292]=0;
  p_output1[293]=0;
  p_output1[294]=0;
  p_output1[295]=0;
  p_output1[296]=0;
  p_output1[297]=0;
  p_output1[298]=0;
  p_output1[299]=0;
  p_output1[300]=0;
  p_output1[301]=0;
  p_output1[302]=0;
  p_output1[303]=0;
  p_output1[304]=0;
  p_output1[305]=0;
  p_output1[306]=0;
  p_output1[307]=0;
  p_output1[308]=0;
  p_output1[309]=0;
  p_output1[310]=0;
  p_output1[311]=0;
  p_output1[312]=0;
  p_output1[313]=0;
  p_output1[314]=0;
  p_output1[315]=0;
  p_output1[316]=0;
  p_output1[317]=0;
  p_output1[318]=0;
  p_output1[319]=0;
  p_output1[320]=0;
  p_output1[321]=0;
  p_output1[322]=0;
  p_output1[323]=0;
  p_output1[324]=0;
  p_output1[325]=0;
  p_output1[326]=0;
  p_output1[327]=0;
  p_output1[328]=0;
  p_output1[329]=0;
  p_output1[330]=0;
  p_output1[331]=0;
  p_output1[332]=0;
  p_output1[333]=0;
  p_output1[334]=0;
  p_output1[335]=0;
  p_output1[336]=0;
  p_output1[337]=0;
  p_output1[338]=0;
  p_output1[339]=0;
  p_output1[340]=0;
  p_output1[341]=0;
  p_output1[342]=0;
  p_output1[343]=0;
  p_output1[344]=0;
  p_output1[345]=0;
  p_output1[346]=0;
  p_output1[347]=0;
  p_output1[348]=0;
  p_output1[349]=0;
  p_output1[350]=0;
  p_output1[351]=0;
  p_output1[352]=0;
  p_output1[353]=0;
  p_output1[354]=0;
  p_output1[355]=0;
  p_output1[356]=0;
  p_output1[357]=0;
  p_output1[358]=0;
  p_output1[359]=0;
  p_output1[360]=0;
  p_output1[361]=0;
  p_output1[362]=0;
  p_output1[363]=0;
  p_output1[364]=0;
  p_output1[365]=0;
  p_output1[366]=0;
  p_output1[367]=0;
  p_output1[368]=0;
  p_output1[369]=0;
  p_output1[370]=0;
  p_output1[371]=0;
  p_output1[372]=0;
  p_output1[373]=0;
  p_output1[374]=0;
  p_output1[375]=0;
  p_output1[376]=0;
  p_output1[377]=0;
  p_output1[378]=0;
  p_output1[379]=0;
  p_output1[380]=0;
  p_output1[381]=0;
  p_output1[382]=0;
  p_output1[383]=0;
  p_output1[384]=0;
  p_output1[385]=0;
  p_output1[386]=0;
  p_output1[387]=0;
  p_output1[388]=0;
  p_output1[389]=0;
  p_output1[390]=0;
  p_output1[391]=0;
  p_output1[392]=0;
  p_output1[393]=0;
  p_output1[394]=0;
  p_output1[395]=0;
  p_output1[396]=0;
  p_output1[397]=0;
  p_output1[398]=0;
  p_output1[399]=0;
  p_output1[400]=0;
  p_output1[401]=0;
  p_output1[402]=0;
  p_output1[403]=0;
  p_output1[404]=0;
  p_output1[405]=0;
  p_output1[406]=0;
  p_output1[407]=0;
  p_output1[408]=0;
  p_output1[409]=0;
  p_output1[410]=0;
  p_output1[411]=0;
  p_output1[412]=0;
  p_output1[413]=0;
  p_output1[414]=0;
  p_output1[415]=0;
  p_output1[416]=0;
  p_output1[417]=0;
  p_output1[418]=0;
  p_output1[419]=0;
  p_output1[420]=0;
  p_output1[421]=0;
  p_output1[422]=0;
  p_output1[423]=0;
  p_output1[424]=0;
  p_output1[425]=0;
  p_output1[426]=0;
  p_output1[427]=0;
  p_output1[428]=0;
  p_output1[429]=0;
  p_output1[430]=0;
  p_output1[431]=0;
  p_output1[432]=0;
  p_output1[433]=0;
  p_output1[434]=0;
  p_output1[435]=0;
  p_output1[436]=0;
  p_output1[437]=0;
  p_output1[438]=0;
  p_output1[439]=0;
  p_output1[440]=0;
  p_output1[441]=0;
  p_output1[442]=0;
  p_output1[443]=0;
  p_output1[444]=0;
  p_output1[445]=0;
  p_output1[446]=0;
  p_output1[447]=0;
  p_output1[448]=0;
  p_output1[449]=0;
  p_output1[450]=0;
  p_output1[451]=0;
  p_output1[452]=0;
  p_output1[453]=0;
  p_output1[454]=0;
  p_output1[455]=0;
  p_output1[456]=0;
  p_output1[457]=0;
  p_output1[458]=0;
  p_output1[459]=0;
  p_output1[460]=0;
  p_output1[461]=0;
  p_output1[462]=0;
  p_output1[463]=0;
  p_output1[464]=0;
  p_output1[465]=0;
  p_output1[466]=0;
  p_output1[467]=0;
  p_output1[468]=0;
  p_output1[469]=0;
  p_output1[470]=0;
  p_output1[471]=0;
  p_output1[472]=0;
  p_output1[473]=0;
  p_output1[474]=0;
  p_output1[475]=0;
  p_output1[476]=0;
  p_output1[477]=0;
  p_output1[478]=0;
  p_output1[479]=0;
  p_output1[480]=0;
  p_output1[481]=0;
  p_output1[482]=0;
  p_output1[483]=0;
  p_output1[484]=0;
  p_output1[485]=0;
  p_output1[486]=0;
  p_output1[487]=0;
  p_output1[488]=0;
  p_output1[489]=0;
  p_output1[490]=0;
  p_output1[491]=0;
  p_output1[492]=0;
  p_output1[493]=0;
  p_output1[494]=0;
  p_output1[495]=0;
  p_output1[496]=0;
  p_output1[497]=0;
  p_output1[498]=0;
  p_output1[499]=0;
  p_output1[500]=0;
  p_output1[501]=0;
  p_output1[502]=0;
  p_output1[503]=0;
  p_output1[504]=0;
  p_output1[505]=0;
  p_output1[506]=0;
  p_output1[507]=0;
  p_output1[508]=0;
  p_output1[509]=0;
  p_output1[510]=0;
  p_output1[511]=0;
  p_output1[512]=0;
  p_output1[513]=0;
  p_output1[514]=0;
  p_output1[515]=0;
  p_output1[516]=0;
  p_output1[517]=0;
  p_output1[518]=0;
  p_output1[519]=0;
  p_output1[520]=0;
  p_output1[521]=0;
  p_output1[522]=0;
  p_output1[523]=0;
  p_output1[524]=0;
  p_output1[525]=0;
  p_output1[526]=0;
  p_output1[527]=0;
  p_output1[528]=0;
  p_output1[529]=0;
  p_output1[530]=0;
  p_output1[531]=0;
  p_output1[532]=0;
  p_output1[533]=0;
  p_output1[534]=0;
  p_output1[535]=0;
  p_output1[536]=0;
  p_output1[537]=0;
  p_output1[538]=0;
  p_output1[539]=0;
  p_output1[540]=0;
  p_output1[541]=0;
  p_output1[542]=0;
  p_output1[543]=0;
  p_output1[544]=0;
  p_output1[545]=0;
  p_output1[546]=0;
  p_output1[547]=0;
  p_output1[548]=0;
  p_output1[549]=0;
  p_output1[550]=0;
  p_output1[551]=0;
  p_output1[552]=0;
  p_output1[553]=0;
  p_output1[554]=0;
  p_output1[555]=0;
  p_output1[556]=0;
  p_output1[557]=0;
  p_output1[558]=0;
  p_output1[559]=0;
  p_output1[560]=0;
  p_output1[561]=0;
  p_output1[562]=0;
  p_output1[563]=0;
  p_output1[564]=0;
  p_output1[565]=0;
  p_output1[566]=0;
  p_output1[567]=0;
  p_output1[568]=0;
  p_output1[569]=0;
  p_output1[570]=0;
  p_output1[571]=0;
  p_output1[572]=0;
  p_output1[573]=0;
  p_output1[574]=0;
  p_output1[575]=0;
  p_output1[576]=0;
  p_output1[577]=0;
  p_output1[578]=0;
  p_output1[579]=0;
  p_output1[580]=0.128;
  p_output1[581]=0;
  p_output1[582]=0;
  p_output1[583]=0;
  p_output1[584]=0;
  p_output1[585]=0;
  p_output1[586]=0;
  p_output1[587]=0;
  p_output1[588]=0;
  p_output1[589]=0;
  p_output1[590]=0;
  p_output1[591]=0;
  p_output1[592]=0;
  p_output1[593]=0;
  p_output1[594]=0;
  p_output1[595]=0;
  p_output1[596]=0;
  p_output1[597]=0;
  p_output1[598]=0;
  p_output1[599]=0;
  p_output1[600]=0;
  p_output1[601]=0;
  p_output1[602]=0;
  p_output1[603]=0;
  p_output1[604]=0;
  p_output1[605]=0;
  p_output1[606]=0;
  p_output1[607]=0;
  p_output1[608]=0;
  p_output1[609]=0.128;
  p_output1[610]=0;
  p_output1[611]=0;
  p_output1[612]=0;
  p_output1[613]=0;
  p_output1[614]=0;
  p_output1[615]=0;
  p_output1[616]=0;
  p_output1[617]=0;
  p_output1[618]=0;
  p_output1[619]=0;
  p_output1[620]=0;
  p_output1[621]=0;
  p_output1[622]=0;
  p_output1[623]=0;
  p_output1[624]=0;
  p_output1[625]=0;
  p_output1[626]=0;
  p_output1[627]=0;
  p_output1[628]=0;
  p_output1[629]=0;
  p_output1[630]=0;
  p_output1[631]=0;
  p_output1[632]=0;
  p_output1[633]=0;
  p_output1[634]=0;
  p_output1[635]=0;
  p_output1[636]=0;
  p_output1[637]=0;
  p_output1[638]=0;
  p_output1[639]=0;
  p_output1[640]=0;
  p_output1[641]=0;
  p_output1[642]=0;
  p_output1[643]=0;
  p_output1[644]=0;
  p_output1[645]=0;
  p_output1[646]=0;
  p_output1[647]=0;
  p_output1[648]=0;
  p_output1[649]=0;
  p_output1[650]=0;
  p_output1[651]=0;
  p_output1[652]=0;
  p_output1[653]=0;
  p_output1[654]=0;
  p_output1[655]=0;
  p_output1[656]=0;
  p_output1[657]=0;
  p_output1[658]=0;
  p_output1[659]=0;
  p_output1[660]=0;
  p_output1[661]=0;
  p_output1[662]=0;
  p_output1[663]=0;
  p_output1[664]=0;
  p_output1[665]=0;
  p_output1[666]=0;
  p_output1[667]=0;
  p_output1[668]=0;
  p_output1[669]=0;
  p_output1[670]=0;
  p_output1[671]=0;
  p_output1[672]=0;
  p_output1[673]=0;
  p_output1[674]=0;
  p_output1[675]=0;
  p_output1[676]=0;
  p_output1[677]=0;
  p_output1[678]=0;
  p_output1[679]=0;
  p_output1[680]=0;
  p_output1[681]=0;
  p_output1[682]=0;
  p_output1[683]=0;
  p_output1[684]=0;
  p_output1[685]=0;
  p_output1[686]=0;
  p_output1[687]=0;
  p_output1[688]=0;
  p_output1[689]=0;
  p_output1[690]=0;
  p_output1[691]=0;
  p_output1[692]=0;
  p_output1[693]=0;
  p_output1[694]=0;
  p_output1[695]=0;
  p_output1[696]=0;
  p_output1[697]=0;
  p_output1[698]=0;
  p_output1[699]=0;
  p_output1[700]=0;
  p_output1[701]=0;
  p_output1[702]=0;
  p_output1[703]=0;
  p_output1[704]=0;
  p_output1[705]=0;
  p_output1[706]=0;
  p_output1[707]=0;
  p_output1[708]=0;
  p_output1[709]=0;
  p_output1[710]=0;
  p_output1[711]=0;
  p_output1[712]=0;
  p_output1[713]=0;
  p_output1[714]=0;
  p_output1[715]=0;
  p_output1[716]=0;
  p_output1[717]=0;
  p_output1[718]=0;
  p_output1[719]=0;
  p_output1[720]=0;
  p_output1[721]=0;
  p_output1[722]=0;
  p_output1[723]=0;
  p_output1[724]=0;
  p_output1[725]=0;
  p_output1[726]=0;
  p_output1[727]=0;
  p_output1[728]=0;
  p_output1[729]=0;
  p_output1[730]=0;
  p_output1[731]=0;
  p_output1[732]=0;
  p_output1[733]=0;
  p_output1[734]=0;
  p_output1[735]=0;
  p_output1[736]=0;
  p_output1[737]=0;
  p_output1[738]=0;
  p_output1[739]=0;
  p_output1[740]=0;
  p_output1[741]=0;
  p_output1[742]=0;
  p_output1[743]=0;
  p_output1[744]=0;
  p_output1[745]=0;
  p_output1[746]=0;
  p_output1[747]=0;
  p_output1[748]=0;
  p_output1[749]=0;
  p_output1[750]=0;
  p_output1[751]=0;
  p_output1[752]=0;
  p_output1[753]=0;
  p_output1[754]=0;
  p_output1[755]=0;
  p_output1[756]=0;
  p_output1[757]=0;
  p_output1[758]=0;
  p_output1[759]=0;
  p_output1[760]=0;
  p_output1[761]=0;
  p_output1[762]=0;
  p_output1[763]=0;
  p_output1[764]=0;
  p_output1[765]=0;
  p_output1[766]=0;
  p_output1[767]=0;
  p_output1[768]=0;
  p_output1[769]=0;
  p_output1[770]=0;
  p_output1[771]=0;
  p_output1[772]=0;
  p_output1[773]=0;
  p_output1[774]=0;
  p_output1[775]=0;
  p_output1[776]=0;
  p_output1[777]=0;
  p_output1[778]=0;
  p_output1[779]=0;
  p_output1[780]=0;
  p_output1[781]=0;
  p_output1[782]=0;
  p_output1[783]=0;
}



void gen::dyn::InertiaMatrix_sum_26(Eigen::Ref<Eigen::VectorXd> p_output1, const Eigen::Ref<const Eigen::VectorXd> var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
