/*
 * Automatically Generated from Mathematica.
 * Thu 31 Mar 2022 15:52:43 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "gen/kin/p_right_hand.hh"

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
  double t1489;
  double t1568;
  double t1599;
  double t1601;
  double t1624;
  double t1633;
  double t1656;
  double t1639;
  double t1661;
  double t1511;
  double t1526;
  double t1529;
  double t1532;
  double t1653;
  double t1662;
  double t1664;
  double t1688;
  double t1689;
  double t1690;
  double t1765;
  double t1766;
  double t1770;
  double t1773;
  double t1499;
  double t1722;
  double t1726;
  double t1727;
  double t1743;
  double t1748;
  double t1749;
  double t1772;
  double t1834;
  double t1788;
  double t1887;
  double t1802;
  double t1805;
  double t1806;
  double t1807;
  double t1810;
  double t1814;
  double t1873;
  double t1850;
  double t1858;
  double t1862;
  double t1863;
  double t1864;
  double t1868;
  double t1894;
  double t1898;
  double t1905;
  double t1906;
  double t1907;
  double t1909;
  double t1912;
  double t1914;
  double t1915;
  double t1918;
  double t1778;
  double t1923;
  double t1790;
  double t1962;
  double t1843;
  double t1930;
  double t1836;
  double t1924;
  double t1928;
  double t1931;
  double t1932;
  double t1933;
  double t1934;
  double t1936;
  double t1940;
  double t1951;
  double t1953;
  double t1961;
  double t1963;
  double t1964;
  double t1965;
  double t1966;
  double t1969;
  double t1976;
  double t1977;
  double t1978;
  double t1980;
  double t1984;
  double t1988;
  double t1994;
  double t1996;
  double t1998;
  double t1999;
  double t2002;
  double t2005;
  double t2006;
  double t2008;
  double t2010;
  double t2012;
  double t2014;
  double t2015;
  double t2019;
  double t2021;
  double t2025;
  double t2028;
  double t2029;
  double t2031;
  double t2032;
  double t2033;
  double t1530;
  double t1561;
  double t1562;
  double t1615;
  double t1631;
  double t1632;
  double t1679;
  double t1685;
  double t1687;
  double t1698;
  double t1709;
  double t1712;
  double t1717;
  double t2055;
  double t2056;
  double t2058;
  double t2062;
  double t2063;
  double t2069;
  double t1730;
  double t1732;
  double t1733;
  double t1742;
  double t1762;
  double t1771;
  double t1783;
  double t1787;
  double t1792;
  double t1793;
  double t1794;
  double t2071;
  double t2072;
  double t2073;
  double t2076;
  double t2077;
  double t2078;
  double t1830;
  double t1833;
  double t1837;
  double t1840;
  double t1844;
  double t1845;
  double t1846;
  double t1870;
  double t1871;
  double t1875;
  double t1885;
  double t1891;
  double t1892;
  double t1893;
  double t1916;
  double t1920;
  double t1922;
  double t2080;
  double t2081;
  double t2082;
  double t2084;
  double t2086;
  double t2087;
  double t2088;
  double t2090;
  double t2093;
  double t2096;
  double t2097;
  double t2098;
  double t1942;
  double t1943;
  double t1947;
  double t1948;
  double t1971;
  double t1973;
  double t1974;
  double t1975;
  double t2101;
  double t2102;
  double t2104;
  double t2106;
  double t2108;
  double t2109;
  double t2111;
  double t2112;
  double t2116;
  double t2119;
  double t2120;
  double t2121;
  double t2124;
  double t2125;
  double t2126;
  double t2128;
  double t2132;
  double t2137;
  double t2142;
  double t2143;
  double t2146;
  double t2147;
  double t2148;
  double t2150;
  double t2050;
  double t1564;
  double t1666;
  double t1696;
  double t1729;
  double t1750;
  double t1818;
  double t1869;
  double t1911;
  double t1941;
  double t1970;
  double t1997;
  double t2009;
  double t2024;
  double t2034;
  double t2035;
  double t2036;
  double t2038;
  double t2039;
  double t2041;
  double t2042;
  double t2044;
  double t2045;
  double t2047;
  double t2048;
  double t1439;
  double t2054;
  double t2061;
  double t2070;
  double t2075;
  double t2079;
  double t2085;
  double t2092;
  double t2099;
  double t2107;
  double t2115;
  double t2122;
  double t2129;
  double t2144;
  double t2151;
  double t2152;
  double t2153;
  double t2154;
  double t2155;
  double t2156;
  double t2157;
  double t2158;
  double t2159;
  double t2160;
  double t2161;
  double t2170;
  double t2171;
  double t2172;
  double t2177;
  double t2178;
  double t2179;
  double t2182;
  double t2183;
  double t2184;
  double t2185;
  double t2189;
  double t2190;
  double t2195;
  double t2199;
  double t2203;
  double t2205;
  double t2206;
  double t2207;
  double t2209;
  double t2210;
  double t2211;
  double t2212;
  double t2215;
  double t2216;
  double t2217;
  double t2219;
  double t2221;
  double t2223;
  double t2224;
  double t2225;
  double t2228;
  double t2229;
  double t2230;
  double t2232;
  double t2234;
  double t2236;
  double t2237;
  double t2238;
  double t2240;
  double t2241;
  double t2242;
  double t2243;
  t1489 = Cos(var1[3]);
  t1568 = Cos(var1[26]);
  t1599 = -1.*t1568;
  t1601 = 1. + t1599;
  t1624 = Sin(var1[26]);
  t1633 = Cos(var1[5]);
  t1656 = Sin(var1[3]);
  t1639 = Sin(var1[4]);
  t1661 = Sin(var1[5]);
  t1511 = Cos(var1[27]);
  t1526 = -1.*t1511;
  t1529 = 1. + t1526;
  t1532 = Sin(var1[27]);
  t1653 = t1489*t1633*t1639;
  t1662 = t1656*t1661;
  t1664 = t1653 + t1662;
  t1688 = -1.*t1633*t1656;
  t1689 = t1489*t1639*t1661;
  t1690 = t1688 + t1689;
  t1765 = Cos(var1[28]);
  t1766 = -1.*t1765;
  t1770 = 1. + t1766;
  t1773 = Sin(var1[28]);
  t1499 = Cos(var1[4]);
  t1722 = -1.*t1624*t1664;
  t1726 = t1568*t1690;
  t1727 = t1722 + t1726;
  t1743 = t1568*t1664;
  t1748 = t1624*t1690;
  t1749 = t1743 + t1748;
  t1772 = -0.051978134642000004*t1770;
  t1834 = 0.05226439969100001*t1770;
  t1788 = 0.49726168403800003*t1770;
  t1887 = 0.073913*t1773;
  t1802 = -0.994522*t1489*t1499*t1532;
  t1805 = -0.103955395616*t1529*t1727;
  t1806 = -0.9890740084840001*t1529;
  t1807 = 1. + t1806;
  t1810 = t1807*t1749;
  t1814 = t1802 + t1805 + t1810;
  t1873 = -0.703234*t1773;
  t1850 = -0.104528*t1489*t1499*t1532;
  t1858 = -0.010926102783999999*t1529;
  t1862 = 1. + t1858;
  t1863 = t1862*t1727;
  t1864 = -0.103955395616*t1529*t1749;
  t1868 = t1850 + t1863 + t1864;
  t1894 = -1.0000001112680001*t1529;
  t1898 = 1. + t1894;
  t1905 = t1898*t1489*t1499;
  t1906 = 0.104528*t1532*t1727;
  t1907 = 0.994522*t1532*t1749;
  t1909 = t1905 + t1906 + t1907;
  t1912 = Cos(var1[29]);
  t1914 = -1.*t1912;
  t1915 = 1. + t1914;
  t1918 = Sin(var1[29]);
  t1778 = -0.707107*t1773;
  t1923 = -0.49726168403800003*t1770;
  t1790 = -0.073913*t1773;
  t1962 = 0.051978134642000004*t1770;
  t1843 = 0.707107*t1773;
  t1930 = -0.05226439969100001*t1770;
  t1836 = 0.703234*t1773;
  t1924 = t1923 + t1887;
  t1928 = t1924*t1814;
  t1931 = t1930 + t1873;
  t1932 = t1931*t1868;
  t1933 = -0.500001190325*t1770;
  t1934 = 1. + t1933;
  t1936 = t1934*t1909;
  t1940 = t1928 + t1932 + t1936;
  t1951 = -0.5054634410180001*t1770;
  t1953 = 1. + t1951;
  t1961 = t1953*t1814;
  t1963 = t1962 + t1778;
  t1964 = t1963*t1868;
  t1965 = t1923 + t1790;
  t1966 = t1965*t1909;
  t1969 = t1961 + t1964 + t1966;
  t1976 = t1962 + t1843;
  t1977 = t1976*t1814;
  t1978 = -0.9945383682050002*t1770;
  t1980 = 1. + t1978;
  t1984 = t1980*t1868;
  t1988 = t1930 + t1836;
  t1994 = t1988*t1909;
  t1996 = t1977 + t1984 + t1994;
  t1998 = -0.104528*t1918*t1940;
  t1999 = -0.103955395616*t1915*t1969;
  t2002 = -0.010926102783999999*t1915;
  t2005 = 1. + t2002;
  t2006 = t2005*t1996;
  t2008 = t1998 + t1999 + t2006;
  t2010 = -0.994522*t1918*t1940;
  t2012 = -0.9890740084840001*t1915;
  t2014 = 1. + t2012;
  t2015 = t2014*t1969;
  t2019 = -0.103955395616*t1915*t1996;
  t2021 = t2010 + t2015 + t2019;
  t2025 = -1.0000001112680001*t1915;
  t2028 = 1. + t2025;
  t2029 = t2028*t1940;
  t2031 = 0.994522*t1918*t1969;
  t2032 = 0.104528*t1918*t1996;
  t2033 = t2029 + t2031 + t2032;
  t1530 = -0.056500534356700764*t1529;
  t1561 = 0.3852490428658858*t1532;
  t1562 = t1530 + t1561;
  t1615 = 0.4*t1601;
  t1631 = -0.12*t1624;
  t1632 = t1615 + t1631;
  t1679 = -0.12*t1601;
  t1685 = -0.4*t1624;
  t1687 = t1679 + t1685;
  t1698 = 1.1345904784751044e-7*var1[27];
  t1709 = 0.0402693119526853*t1529;
  t1712 = 0.0059058871981009595*t1532;
  t1717 = t1698 + t1709 + t1712;
  t2055 = t1633*t1656*t1639;
  t2056 = -1.*t1489*t1661;
  t2058 = t2055 + t2056;
  t2062 = t1489*t1633;
  t2063 = t1656*t1639*t1661;
  t2069 = t2062 + t2063;
  t1730 = -1.1924972351948546e-8*var1[27];
  t1732 = 0.3831386486090665*t1529;
  t1733 = 0.05619101817723254*t1532;
  t1742 = t1730 + t1732 + t1733;
  t1762 = 4.0332087336819504e-7*var1[28];
  t1771 = 0.0958179942122405*t1770;
  t1783 = t1772 + t1778;
  t1787 = -0.23105307644*t1783;
  t1792 = t1788 + t1790;
  t1793 = 0.164374659834*t1792;
  t1794 = t1762 + t1771 + t1787 + t1793;
  t2071 = -1.*t1624*t2058;
  t2072 = t1568*t2069;
  t2073 = t2071 + t2072;
  t2076 = t1568*t2058;
  t2077 = t1624*t2069;
  t2078 = t2076 + t2077;
  t1830 = 4.239080549754904e-8*var1[28];
  t1833 = -0.22979114961138278*t1770;
  t1837 = t1834 + t1836;
  t1840 = 0.164374659834*t1837;
  t1844 = t1772 + t1843;
  t1845 = 0.189564637987*t1844;
  t1846 = t1830 + t1833 + t1840 + t1845;
  t1870 = -4.05542127947119e-7*var1[28];
  t1871 = 0.08218752557626696*t1770;
  t1875 = t1834 + t1873;
  t1885 = -0.23105307644*t1875;
  t1891 = t1788 + t1887;
  t1892 = 0.189564637987*t1891;
  t1893 = t1870 + t1871 + t1885 + t1892;
  t1916 = 0.19098732144477495*t1915;
  t1920 = 0.13776101532839094*t1918;
  t1922 = t1916 + t1920;
  t2080 = -0.994522*t1499*t1532*t1656;
  t2081 = -0.103955395616*t1529*t2073;
  t2082 = t1807*t2078;
  t2084 = t2080 + t2081 + t2082;
  t2086 = -0.104528*t1499*t1532*t1656;
  t2087 = t1862*t2073;
  t2088 = -0.103955395616*t1529*t2078;
  t2090 = t2086 + t2087 + t2088;
  t2093 = t1898*t1499*t1656;
  t2096 = 0.104528*t1532*t2073;
  t2097 = 0.994522*t1532*t2078;
  t2098 = t2093 + t2096 + t2097;
  t1942 = 5.06291820800569e-8*var1[29];
  t1943 = 0.13700636048642204*t1915;
  t1947 = -0.18994107176353728*t1918;
  t1948 = t1942 + t1943 + t1947;
  t1971 = -4.817066759205414e-7*var1[29];
  t1973 = 0.014399883410246048*t1915;
  t1974 = -0.019963520514678434*t1918;
  t1975 = t1971 + t1973 + t1974;
  t2101 = t1924*t2084;
  t2102 = t1931*t2090;
  t2104 = t1934*t2098;
  t2106 = t2101 + t2102 + t2104;
  t2108 = t1953*t2084;
  t2109 = t1963*t2090;
  t2111 = t1965*t2098;
  t2112 = t2108 + t2109 + t2111;
  t2116 = t1976*t2084;
  t2119 = t1980*t2090;
  t2120 = t1988*t2098;
  t2121 = t2116 + t2119 + t2120;
  t2124 = -0.104528*t1918*t2106;
  t2125 = -0.103955395616*t1915*t2112;
  t2126 = t2005*t2121;
  t2128 = t2124 + t2125 + t2126;
  t2132 = -0.994522*t1918*t2106;
  t2137 = t2014*t2112;
  t2142 = -0.103955395616*t1915*t2121;
  t2143 = t2132 + t2137 + t2142;
  t2146 = t2028*t2106;
  t2147 = 0.994522*t1918*t2112;
  t2148 = 0.104528*t1918*t2121;
  t2150 = t2146 + t2147 + t2148;
  t2050 = Sin(var2[0]);
  t1564 = t1489*t1499*t1562;
  t1666 = t1632*t1664;
  t1696 = t1687*t1690;
  t1729 = t1717*t1727;
  t1750 = t1742*t1749;
  t1818 = t1794*t1814;
  t1869 = t1846*t1868;
  t1911 = t1893*t1909;
  t1941 = t1922*t1940;
  t1970 = t1948*t1969;
  t1997 = t1975*t1996;
  t2009 = -0.272124*t2008;
  t2024 = 0.167122*t2021;
  t2034 = 0.190987*t2033;
  t2035 = -0.994522*t2008;
  t2036 = 0.104528*t2021;
  t2038 = t2035 + t2036;
  t2039 = -0.07912*t2038;
  t2041 = 0.040001*t2008;
  t2042 = 0.380588*t2021;
  t2044 = 0.92388*t2033;
  t2045 = t2041 + t2042 + t2044;
  t2047 = 0.369*t2045;
  t2048 = var1[0] + t1564 + t1666 + t1696 + t1729 + t1750 + t1818 + t1869 + t1911 + t1941 + t1970 + t1997 + t2009 + t2024 + t2034 + t2039 + t2047;
  t1439 = Cos(var2[0]);
  t2054 = t1499*t1562*t1656;
  t2061 = t1632*t2058;
  t2070 = t1687*t2069;
  t2075 = t1717*t2073;
  t2079 = t1742*t2078;
  t2085 = t1794*t2084;
  t2092 = t1846*t2090;
  t2099 = t1893*t2098;
  t2107 = t1922*t2106;
  t2115 = t1948*t2112;
  t2122 = t1975*t2121;
  t2129 = -0.272124*t2128;
  t2144 = 0.167122*t2143;
  t2151 = 0.190987*t2150;
  t2152 = -0.994522*t2128;
  t2153 = 0.104528*t2143;
  t2154 = t2152 + t2153;
  t2155 = -0.07912*t2154;
  t2156 = 0.040001*t2128;
  t2157 = 0.380588*t2143;
  t2158 = 0.92388*t2150;
  t2159 = t2156 + t2157 + t2158;
  t2160 = 0.369*t2159;
  t2161 = var1[1] + t2054 + t2061 + t2070 + t2075 + t2079 + t2085 + t2092 + t2099 + t2107 + t2115 + t2122 + t2129 + t2144 + t2151 + t2155 + t2160;
  t2170 = -1.*t1499*t1633*t1624;
  t2171 = t1568*t1499*t1661;
  t2172 = t2170 + t2171;
  t2177 = t1568*t1499*t1633;
  t2178 = t1499*t1624*t1661;
  t2179 = t2177 + t2178;
  t2182 = 0.994522*t1532*t1639;
  t2183 = -0.103955395616*t1529*t2172;
  t2184 = t1807*t2179;
  t2185 = t2182 + t2183 + t2184;
  t2189 = 0.104528*t1532*t1639;
  t2190 = t1862*t2172;
  t2195 = -0.103955395616*t1529*t2179;
  t2199 = t2189 + t2190 + t2195;
  t2203 = -1.*t1898*t1639;
  t2205 = 0.104528*t1532*t2172;
  t2206 = 0.994522*t1532*t2179;
  t2207 = t2203 + t2205 + t2206;
  t2209 = t1924*t2185;
  t2210 = t1931*t2199;
  t2211 = t1934*t2207;
  t2212 = t2209 + t2210 + t2211;
  t2215 = t1953*t2185;
  t2216 = t1963*t2199;
  t2217 = t1965*t2207;
  t2219 = t2215 + t2216 + t2217;
  t2221 = t1976*t2185;
  t2223 = t1980*t2199;
  t2224 = t1988*t2207;
  t2225 = t2221 + t2223 + t2224;
  t2228 = -0.104528*t1918*t2212;
  t2229 = -0.103955395616*t1915*t2219;
  t2230 = t2005*t2225;
  t2232 = t2228 + t2229 + t2230;
  t2234 = -0.994522*t1918*t2212;
  t2236 = t2014*t2219;
  t2237 = -0.103955395616*t1915*t2225;
  t2238 = t2234 + t2236 + t2237;
  t2240 = t2028*t2212;
  t2241 = 0.994522*t1918*t2219;
  t2242 = 0.104528*t1918*t2225;
  t2243 = t2240 + t2241 + t2242;
  p_output1[0]=t1439*t2048 + t2050*t2161;
  p_output1[1]=-1.*t2048*t2050 + t1439*t2161;
  p_output1[2]=t1499*t1632*t1633 - 1.*t1562*t1639 + t1499*t1661*t1687 + t1717*t2172 + t1742*t2179 + t1794*t2185 + t1846*t2199 + t1893*t2207 + t1922*t2212 + t1948*t2219 + t1975*t2225 - 0.272124*t2232 - 0.07912*(-0.994522*t2232 + 0.104528*t2238) + 0.167122*t2238 + 0.369*(0.040001*t2232 + 0.380588*t2238 + 0.92388*t2243) + 0.190987*t2243 + var1[2];
}



void gen::kin::p_right_hand(Eigen::Ref<Eigen::VectorXd> p_output1, const Eigen::Ref<const Eigen::VectorXd> var1,const Eigen::Ref<const Eigen::VectorXd> var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
