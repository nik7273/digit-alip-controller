/*
 * Automatically Generated from Mathematica.
 * Thu 31 Mar 2022 15:52:43 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "gen/kin/p_right_hand_wrt_base.hh"

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
  double t1561;
  double t1666;
  double t1771;
  double t1773;
  double t1778;
  double t1787;
  double t1793;
  double t1805;
  double t1794;
  double t1810;
  double t1687;
  double t1698;
  double t1712;
  double t1733;
  double t1802;
  double t1814;
  double t1818;
  double t1844;
  double t1845;
  double t1846;
  double t1918;
  double t1920;
  double t1922;
  double t1928;
  double t1679;
  double t1870;
  double t1871;
  double t1873;
  double t1906;
  double t1907;
  double t1909;
  double t1924;
  double t1975;
  double t1941;
  double t2041;
  double t1961;
  double t1962;
  double t1963;
  double t1964;
  double t1965;
  double t1970;
  double t2034;
  double t2008;
  double t2009;
  double t2010;
  double t2021;
  double t2024;
  double t2029;
  double t2047;
  double t2049;
  double t2054;
  double t2055;
  double t2061;
  double t2062;
  double t2069;
  double t2073;
  double t2075;
  double t2077;
  double t1932;
  double t2080;
  double t1942;
  double t2115;
  double t1997;
  double t2086;
  double t1988;
  double t2084;
  double t2085;
  double t2087;
  double t2092;
  double t2093;
  double t2096;
  double t2097;
  double t2101;
  double t2109;
  double t2111;
  double t2112;
  double t2116;
  double t2119;
  double t2120;
  double t2121;
  double t2124;
  double t2150;
  double t2151;
  double t2152;
  double t2153;
  double t2154;
  double t2155;
  double t2156;
  double t2157;
  double t2159;
  double t2160;
  double t2162;
  double t2163;
  double t2164;
  double t2165;
  double t2168;
  double t2169;
  double t2171;
  double t2172;
  double t2176;
  double t2177;
  double t2179;
  double t2180;
  double t2182;
  double t2183;
  double t2184;
  double t2185;
  double t1729;
  double t1743;
  double t1748;
  double t1783;
  double t1788;
  double t1792;
  double t1833;
  double t1834;
  double t1836;
  double t1863;
  double t1864;
  double t1868;
  double t1869;
  double t2213;
  double t2215;
  double t2216;
  double t2219;
  double t2220;
  double t2221;
  double t1885;
  double t1892;
  double t1893;
  double t1905;
  double t1916;
  double t1923;
  double t1936;
  double t1940;
  double t1943;
  double t1947;
  double t1948;
  double t2224;
  double t2225;
  double t2226;
  double t2229;
  double t2230;
  double t2232;
  double t1973;
  double t1974;
  double t1994;
  double t1996;
  double t1998;
  double t1999;
  double t2006;
  double t2032;
  double t2033;
  double t2038;
  double t2039;
  double t2042;
  double t2044;
  double t2045;
  double t2076;
  double t2078;
  double t2079;
  double t2236;
  double t2237;
  double t2238;
  double t2239;
  double t2242;
  double t2244;
  double t2245;
  double t2246;
  double t2248;
  double t2250;
  double t2252;
  double t2253;
  double t2104;
  double t2106;
  double t2107;
  double t2108;
  double t2132;
  double t2144;
  double t2147;
  double t2148;
  double t2255;
  double t2256;
  double t2257;
  double t2258;
  double t2261;
  double t2262;
  double t2265;
  double t2267;
  double t2272;
  double t2275;
  double t2276;
  double t2277;
  double t2279;
  double t2280;
  double t2283;
  double t2284;
  double t2287;
  double t2288;
  double t2289;
  double t2290;
  double t2292;
  double t2294;
  double t2295;
  double t2296;
  double t2211;
  double t1749;
  double t1830;
  double t1850;
  double t1875;
  double t1911;
  double t1971;
  double t2031;
  double t2063;
  double t2102;
  double t2128;
  double t2158;
  double t2167;
  double t2178;
  double t2187;
  double t2189;
  double t2190;
  double t2195;
  double t2199;
  double t2201;
  double t2205;
  double t2206;
  double t2207;
  double t2208;
  double t2209;
  double t2212;
  double t2217;
  double t2223;
  double t2228;
  double t2234;
  double t2240;
  double t2247;
  double t2254;
  double t2260;
  double t2271;
  double t2278;
  double t2285;
  double t2291;
  double t2298;
  double t2299;
  double t2302;
  double t2303;
  double t2306;
  double t2307;
  double t2308;
  double t2309;
  double t2310;
  double t2311;
  double t2312;
  double t1662;
  double t2210;
  double t2313;
  double t2314;
  double t2317;
  double t2318;
  double t2319;
  double t2320;
  double t2336;
  double t2337;
  double t2338;
  double t2341;
  double t2342;
  double t2344;
  double t2346;
  double t2347;
  double t2348;
  double t2349;
  double t2352;
  double t2354;
  double t2355;
  double t2357;
  double t2359;
  double t2360;
  double t2361;
  double t2362;
  double t2367;
  double t2368;
  double t2369;
  double t2371;
  double t2375;
  double t2376;
  double t2382;
  double t2383;
  double t2385;
  double t2386;
  double t2388;
  double t2389;
  double t2391;
  double t2392;
  double t2393;
  double t2394;
  double t2397;
  double t2398;
  double t2399;
  double t2400;
  double t2402;
  double t2403;
  double t2404;
  double t2405;
  t1561 = Sin(var2[0]);
  t1666 = Cos(var1[3]);
  t1771 = Cos(var1[26]);
  t1773 = -1.*t1771;
  t1778 = 1. + t1773;
  t1787 = Sin(var1[26]);
  t1793 = Cos(var1[5]);
  t1805 = Sin(var1[3]);
  t1794 = Sin(var1[4]);
  t1810 = Sin(var1[5]);
  t1687 = Cos(var1[27]);
  t1698 = -1.*t1687;
  t1712 = 1. + t1698;
  t1733 = Sin(var1[27]);
  t1802 = t1666*t1793*t1794;
  t1814 = t1805*t1810;
  t1818 = t1802 + t1814;
  t1844 = -1.*t1793*t1805;
  t1845 = t1666*t1794*t1810;
  t1846 = t1844 + t1845;
  t1918 = Cos(var1[28]);
  t1920 = -1.*t1918;
  t1922 = 1. + t1920;
  t1928 = Sin(var1[28]);
  t1679 = Cos(var1[4]);
  t1870 = -1.*t1787*t1818;
  t1871 = t1771*t1846;
  t1873 = t1870 + t1871;
  t1906 = t1771*t1818;
  t1907 = t1787*t1846;
  t1909 = t1906 + t1907;
  t1924 = -0.051978134642000004*t1922;
  t1975 = 0.05226439969100001*t1922;
  t1941 = 0.49726168403800003*t1922;
  t2041 = 0.073913*t1928;
  t1961 = -0.994522*t1666*t1679*t1733;
  t1962 = -0.103955395616*t1712*t1873;
  t1963 = -0.9890740084840001*t1712;
  t1964 = 1. + t1963;
  t1965 = t1964*t1909;
  t1970 = t1961 + t1962 + t1965;
  t2034 = -0.703234*t1928;
  t2008 = -0.104528*t1666*t1679*t1733;
  t2009 = -0.010926102783999999*t1712;
  t2010 = 1. + t2009;
  t2021 = t2010*t1873;
  t2024 = -0.103955395616*t1712*t1909;
  t2029 = t2008 + t2021 + t2024;
  t2047 = -1.0000001112680001*t1712;
  t2049 = 1. + t2047;
  t2054 = t2049*t1666*t1679;
  t2055 = 0.104528*t1733*t1873;
  t2061 = 0.994522*t1733*t1909;
  t2062 = t2054 + t2055 + t2061;
  t2069 = Cos(var1[29]);
  t2073 = -1.*t2069;
  t2075 = 1. + t2073;
  t2077 = Sin(var1[29]);
  t1932 = -0.707107*t1928;
  t2080 = -0.49726168403800003*t1922;
  t1942 = -0.073913*t1928;
  t2115 = 0.051978134642000004*t1922;
  t1997 = 0.707107*t1928;
  t2086 = -0.05226439969100001*t1922;
  t1988 = 0.703234*t1928;
  t2084 = t2080 + t2041;
  t2085 = t2084*t1970;
  t2087 = t2086 + t2034;
  t2092 = t2087*t2029;
  t2093 = -0.500001190325*t1922;
  t2096 = 1. + t2093;
  t2097 = t2096*t2062;
  t2101 = t2085 + t2092 + t2097;
  t2109 = -0.5054634410180001*t1922;
  t2111 = 1. + t2109;
  t2112 = t2111*t1970;
  t2116 = t2115 + t1932;
  t2119 = t2116*t2029;
  t2120 = t2080 + t1942;
  t2121 = t2120*t2062;
  t2124 = t2112 + t2119 + t2121;
  t2150 = t2115 + t1997;
  t2151 = t2150*t1970;
  t2152 = -0.9945383682050002*t1922;
  t2153 = 1. + t2152;
  t2154 = t2153*t2029;
  t2155 = t2086 + t1988;
  t2156 = t2155*t2062;
  t2157 = t2151 + t2154 + t2156;
  t2159 = -0.104528*t2077*t2101;
  t2160 = -0.103955395616*t2075*t2124;
  t2162 = -0.010926102783999999*t2075;
  t2163 = 1. + t2162;
  t2164 = t2163*t2157;
  t2165 = t2159 + t2160 + t2164;
  t2168 = -0.994522*t2077*t2101;
  t2169 = -0.9890740084840001*t2075;
  t2171 = 1. + t2169;
  t2172 = t2171*t2124;
  t2176 = -0.103955395616*t2075*t2157;
  t2177 = t2168 + t2172 + t2176;
  t2179 = -1.0000001112680001*t2075;
  t2180 = 1. + t2179;
  t2182 = t2180*t2101;
  t2183 = 0.994522*t2077*t2124;
  t2184 = 0.104528*t2077*t2157;
  t2185 = t2182 + t2183 + t2184;
  t1729 = -0.056500534356700764*t1712;
  t1743 = 0.3852490428658858*t1733;
  t1748 = t1729 + t1743;
  t1783 = 0.4*t1778;
  t1788 = -0.12*t1787;
  t1792 = t1783 + t1788;
  t1833 = -0.12*t1778;
  t1834 = -0.4*t1787;
  t1836 = t1833 + t1834;
  t1863 = 1.1345904784751044e-7*var1[27];
  t1864 = 0.0402693119526853*t1712;
  t1868 = 0.0059058871981009595*t1733;
  t1869 = t1863 + t1864 + t1868;
  t2213 = t1793*t1805*t1794;
  t2215 = -1.*t1666*t1810;
  t2216 = t2213 + t2215;
  t2219 = t1666*t1793;
  t2220 = t1805*t1794*t1810;
  t2221 = t2219 + t2220;
  t1885 = -1.1924972351948546e-8*var1[27];
  t1892 = 0.3831386486090665*t1712;
  t1893 = 0.05619101817723254*t1733;
  t1905 = t1885 + t1892 + t1893;
  t1916 = 4.0332087336819504e-7*var1[28];
  t1923 = 0.0958179942122405*t1922;
  t1936 = t1924 + t1932;
  t1940 = -0.23105307644*t1936;
  t1943 = t1941 + t1942;
  t1947 = 0.164374659834*t1943;
  t1948 = t1916 + t1923 + t1940 + t1947;
  t2224 = -1.*t1787*t2216;
  t2225 = t1771*t2221;
  t2226 = t2224 + t2225;
  t2229 = t1771*t2216;
  t2230 = t1787*t2221;
  t2232 = t2229 + t2230;
  t1973 = 4.239080549754904e-8*var1[28];
  t1974 = -0.22979114961138278*t1922;
  t1994 = t1975 + t1988;
  t1996 = 0.164374659834*t1994;
  t1998 = t1924 + t1997;
  t1999 = 0.189564637987*t1998;
  t2006 = t1973 + t1974 + t1996 + t1999;
  t2032 = -4.05542127947119e-7*var1[28];
  t2033 = 0.08218752557626696*t1922;
  t2038 = t1975 + t2034;
  t2039 = -0.23105307644*t2038;
  t2042 = t1941 + t2041;
  t2044 = 0.189564637987*t2042;
  t2045 = t2032 + t2033 + t2039 + t2044;
  t2076 = 0.19098732144477495*t2075;
  t2078 = 0.13776101532839094*t2077;
  t2079 = t2076 + t2078;
  t2236 = -0.994522*t1679*t1733*t1805;
  t2237 = -0.103955395616*t1712*t2226;
  t2238 = t1964*t2232;
  t2239 = t2236 + t2237 + t2238;
  t2242 = -0.104528*t1679*t1733*t1805;
  t2244 = t2010*t2226;
  t2245 = -0.103955395616*t1712*t2232;
  t2246 = t2242 + t2244 + t2245;
  t2248 = t2049*t1679*t1805;
  t2250 = 0.104528*t1733*t2226;
  t2252 = 0.994522*t1733*t2232;
  t2253 = t2248 + t2250 + t2252;
  t2104 = 5.06291820800569e-8*var1[29];
  t2106 = 0.13700636048642204*t2075;
  t2107 = -0.18994107176353728*t2077;
  t2108 = t2104 + t2106 + t2107;
  t2132 = -4.817066759205414e-7*var1[29];
  t2144 = 0.014399883410246048*t2075;
  t2147 = -0.019963520514678434*t2077;
  t2148 = t2132 + t2144 + t2147;
  t2255 = t2084*t2239;
  t2256 = t2087*t2246;
  t2257 = t2096*t2253;
  t2258 = t2255 + t2256 + t2257;
  t2261 = t2111*t2239;
  t2262 = t2116*t2246;
  t2265 = t2120*t2253;
  t2267 = t2261 + t2262 + t2265;
  t2272 = t2150*t2239;
  t2275 = t2153*t2246;
  t2276 = t2155*t2253;
  t2277 = t2272 + t2275 + t2276;
  t2279 = -0.104528*t2077*t2258;
  t2280 = -0.103955395616*t2075*t2267;
  t2283 = t2163*t2277;
  t2284 = t2279 + t2280 + t2283;
  t2287 = -0.994522*t2077*t2258;
  t2288 = t2171*t2267;
  t2289 = -0.103955395616*t2075*t2277;
  t2290 = t2287 + t2288 + t2289;
  t2292 = t2180*t2258;
  t2294 = 0.994522*t2077*t2267;
  t2295 = 0.104528*t2077*t2277;
  t2296 = t2292 + t2294 + t2295;
  t2211 = Cos(var2[0]);
  t1749 = t1666*t1679*t1748;
  t1830 = t1792*t1818;
  t1850 = t1836*t1846;
  t1875 = t1869*t1873;
  t1911 = t1905*t1909;
  t1971 = t1948*t1970;
  t2031 = t2006*t2029;
  t2063 = t2045*t2062;
  t2102 = t2079*t2101;
  t2128 = t2108*t2124;
  t2158 = t2148*t2157;
  t2167 = -0.272124*t2165;
  t2178 = 0.167122*t2177;
  t2187 = 0.190987*t2185;
  t2189 = -0.994522*t2165;
  t2190 = 0.104528*t2177;
  t2195 = t2189 + t2190;
  t2199 = -0.07912*t2195;
  t2201 = 0.040001*t2165;
  t2205 = 0.380588*t2177;
  t2206 = 0.92388*t2185;
  t2207 = t2201 + t2205 + t2206;
  t2208 = 0.369*t2207;
  t2209 = var1[0] + t1749 + t1830 + t1850 + t1875 + t1911 + t1971 + t2031 + t2063 + t2102 + t2128 + t2158 + t2167 + t2178 + t2187 + t2199 + t2208;
  t2212 = t1679*t1748*t1805;
  t2217 = t1792*t2216;
  t2223 = t1836*t2221;
  t2228 = t1869*t2226;
  t2234 = t1905*t2232;
  t2240 = t1948*t2239;
  t2247 = t2006*t2246;
  t2254 = t2045*t2253;
  t2260 = t2079*t2258;
  t2271 = t2108*t2267;
  t2278 = t2148*t2277;
  t2285 = -0.272124*t2284;
  t2291 = 0.167122*t2290;
  t2298 = 0.190987*t2296;
  t2299 = -0.994522*t2284;
  t2302 = 0.104528*t2290;
  t2303 = t2299 + t2302;
  t2306 = -0.07912*t2303;
  t2307 = 0.040001*t2284;
  t2308 = 0.380588*t2290;
  t2309 = 0.92388*t2296;
  t2310 = t2307 + t2308 + t2309;
  t2311 = 0.369*t2310;
  t2312 = var1[1] + t2212 + t2217 + t2223 + t2228 + t2234 + t2240 + t2247 + t2254 + t2260 + t2271 + t2278 + t2285 + t2291 + t2298 + t2306 + t2311;
  t1662 = -1.*var1[1];
  t2210 = -1.*t1561*t2209;
  t2313 = t2211*t2312;
  t2314 = t1662 + t2210 + t2313;
  t2317 = -1.*var1[0];
  t2318 = t2211*t2209;
  t2319 = t1561*t2312;
  t2320 = t2317 + t2318 + t2319;
  t2336 = -1.*t1679*t1793*t1787;
  t2337 = t1771*t1679*t1810;
  t2338 = t2336 + t2337;
  t2341 = t1771*t1679*t1793;
  t2342 = t1679*t1787*t1810;
  t2344 = t2341 + t2342;
  t2346 = 0.994522*t1733*t1794;
  t2347 = -0.103955395616*t1712*t2338;
  t2348 = t1964*t2344;
  t2349 = t2346 + t2347 + t2348;
  t2352 = 0.104528*t1733*t1794;
  t2354 = t2010*t2338;
  t2355 = -0.103955395616*t1712*t2344;
  t2357 = t2352 + t2354 + t2355;
  t2359 = -1.*t2049*t1794;
  t2360 = 0.104528*t1733*t2338;
  t2361 = 0.994522*t1733*t2344;
  t2362 = t2359 + t2360 + t2361;
  t2367 = t2084*t2349;
  t2368 = t2087*t2357;
  t2369 = t2096*t2362;
  t2371 = t2367 + t2368 + t2369;
  t2375 = t2111*t2349;
  t2376 = t2116*t2357;
  t2382 = t2120*t2362;
  t2383 = t2375 + t2376 + t2382;
  t2385 = t2150*t2349;
  t2386 = t2153*t2357;
  t2388 = t2155*t2362;
  t2389 = t2385 + t2386 + t2388;
  t2391 = -0.104528*t2077*t2371;
  t2392 = -0.103955395616*t2075*t2383;
  t2393 = t2163*t2389;
  t2394 = t2391 + t2392 + t2393;
  t2397 = -0.994522*t2077*t2371;
  t2398 = t2171*t2383;
  t2399 = -0.103955395616*t2075*t2389;
  t2400 = t2397 + t2398 + t2399;
  t2402 = t2180*t2371;
  t2403 = 0.994522*t2077*t2383;
  t2404 = 0.104528*t2077*t2389;
  t2405 = t2402 + t2403 + t2404;
  p_output1[0]=t1561*t2314 + t2211*t2320;
  p_output1[1]=t2211*t2314 - 1.*t1561*t2320;
  p_output1[2]=t1679*t1792*t1793 - 1.*t1748*t1794 + t1679*t1810*t1836 + t1869*t2338 + t1905*t2344 + t1948*t2349 + t2006*t2357 + t2045*t2362 + t2079*t2371 + t2108*t2383 + t2148*t2389 - 0.272124*t2394 - 0.07912*(-0.994522*t2394 + 0.104528*t2400) + 0.167122*t2400 + 0.369*(0.040001*t2394 + 0.380588*t2400 + 0.92388*t2405) + 0.190987*t2405;
}



void gen::kin::p_right_hand_wrt_base(Eigen::Ref<Eigen::VectorXd> p_output1, const Eigen::Ref<const Eigen::VectorXd> var1,const Eigen::Ref<const Eigen::VectorXd> var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
