/*
 * Automatically Generated from Mathematica.
 * Thu 31 Mar 2022 15:52:42 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include "gen/kin/p_left_hand.hh"

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
  double t1197;
  double t1272;
  double t1301;
  double t1303;
  double t1326;
  double t1336;
  double t1357;
  double t1342;
  double t1362;
  double t1218;
  double t1230;
  double t1232;
  double t1235;
  double t1354;
  double t1363;
  double t1365;
  double t1388;
  double t1389;
  double t1390;
  double t1462;
  double t1463;
  double t1467;
  double t1470;
  double t1208;
  double t1420;
  double t1423;
  double t1424;
  double t1439;
  double t1444;
  double t1445;
  double t1469;
  double t1530;
  double t1485;
  double t1581;
  double t1499;
  double t1502;
  double t1503;
  double t1504;
  double t1507;
  double t1511;
  double t1566;
  double t1546;
  double t1554;
  double t1555;
  double t1556;
  double t1557;
  double t1561;
  double t1588;
  double t1592;
  double t1599;
  double t1600;
  double t1601;
  double t1603;
  double t1606;
  double t1608;
  double t1609;
  double t1611;
  double t1475;
  double t1615;
  double t1487;
  double t1653;
  double t1539;
  double t1622;
  double t1532;
  double t1616;
  double t1620;
  double t1623;
  double t1624;
  double t1625;
  double t1626;
  double t1627;
  double t1631;
  double t1642;
  double t1644;
  double t1652;
  double t1654;
  double t1655;
  double t1656;
  double t1657;
  double t1660;
  double t1667;
  double t1668;
  double t1669;
  double t1671;
  double t1675;
  double t1679;
  double t1685;
  double t1687;
  double t1689;
  double t1690;
  double t1692;
  double t1695;
  double t1696;
  double t1698;
  double t1700;
  double t1702;
  double t1704;
  double t1705;
  double t1707;
  double t1709;
  double t1713;
  double t1716;
  double t1717;
  double t1719;
  double t1720;
  double t1721;
  double t1233;
  double t1265;
  double t1266;
  double t1317;
  double t1334;
  double t1335;
  double t1379;
  double t1385;
  double t1387;
  double t1398;
  double t1409;
  double t1411;
  double t1415;
  double t1743;
  double t1744;
  double t1746;
  double t1749;
  double t1750;
  double t1755;
  double t1427;
  double t1429;
  double t1430;
  double t1438;
  double t1459;
  double t1468;
  double t1480;
  double t1484;
  double t1489;
  double t1490;
  double t1491;
  double t1757;
  double t1758;
  double t1759;
  double t1762;
  double t1763;
  double t1764;
  double t1526;
  double t1529;
  double t1533;
  double t1536;
  double t1540;
  double t1541;
  double t1542;
  double t1563;
  double t1564;
  double t1568;
  double t1579;
  double t1585;
  double t1586;
  double t1587;
  double t1610;
  double t1613;
  double t1614;
  double t1766;
  double t1767;
  double t1768;
  double t1770;
  double t1772;
  double t1773;
  double t1774;
  double t1776;
  double t1779;
  double t1782;
  double t1783;
  double t1784;
  double t1633;
  double t1634;
  double t1638;
  double t1639;
  double t1662;
  double t1664;
  double t1665;
  double t1666;
  double t1787;
  double t1788;
  double t1790;
  double t1792;
  double t1794;
  double t1795;
  double t1797;
  double t1798;
  double t1802;
  double t1805;
  double t1806;
  double t1807;
  double t1810;
  double t1811;
  double t1812;
  double t1814;
  double t1818;
  double t1823;
  double t1828;
  double t1829;
  double t1832;
  double t1833;
  double t1834;
  double t1836;
  double t1738;
  double t1268;
  double t1367;
  double t1396;
  double t1426;
  double t1446;
  double t1515;
  double t1562;
  double t1605;
  double t1632;
  double t1661;
  double t1688;
  double t1699;
  double t1712;
  double t1722;
  double t1723;
  double t1724;
  double t1726;
  double t1727;
  double t1729;
  double t1730;
  double t1732;
  double t1733;
  double t1735;
  double t1736;
  double t1150;
  double t1742;
  double t1748;
  double t1756;
  double t1761;
  double t1765;
  double t1771;
  double t1778;
  double t1785;
  double t1793;
  double t1801;
  double t1808;
  double t1815;
  double t1830;
  double t1837;
  double t1838;
  double t1839;
  double t1840;
  double t1841;
  double t1842;
  double t1843;
  double t1844;
  double t1845;
  double t1846;
  double t1847;
  double t1856;
  double t1857;
  double t1858;
  double t1863;
  double t1864;
  double t1865;
  double t1868;
  double t1869;
  double t1870;
  double t1871;
  double t1875;
  double t1876;
  double t1881;
  double t1885;
  double t1889;
  double t1891;
  double t1892;
  double t1893;
  double t1895;
  double t1896;
  double t1897;
  double t1898;
  double t1901;
  double t1902;
  double t1903;
  double t1905;
  double t1907;
  double t1909;
  double t1910;
  double t1911;
  double t1914;
  double t1915;
  double t1916;
  double t1918;
  double t1920;
  double t1922;
  double t1923;
  double t1924;
  double t1926;
  double t1927;
  double t1928;
  double t1929;
  t1197 = Cos(var1[3]);
  t1272 = Cos(var1[14]);
  t1301 = -1.*t1272;
  t1303 = 1. + t1301;
  t1326 = Sin(var1[14]);
  t1336 = Cos(var1[5]);
  t1357 = Sin(var1[3]);
  t1342 = Sin(var1[4]);
  t1362 = Sin(var1[5]);
  t1218 = Cos(var1[15]);
  t1230 = -1.*t1218;
  t1232 = 1. + t1230;
  t1235 = Sin(var1[15]);
  t1354 = t1197*t1336*t1342;
  t1363 = t1357*t1362;
  t1365 = t1354 + t1363;
  t1388 = -1.*t1336*t1357;
  t1389 = t1197*t1342*t1362;
  t1390 = t1388 + t1389;
  t1462 = Cos(var1[16]);
  t1463 = -1.*t1462;
  t1467 = 1. + t1463;
  t1470 = Sin(var1[16]);
  t1208 = Cos(var1[4]);
  t1420 = -1.*t1326*t1365;
  t1423 = t1272*t1390;
  t1424 = t1420 + t1423;
  t1439 = t1272*t1365;
  t1444 = t1326*t1390;
  t1445 = t1439 + t1444;
  t1469 = 0.051978134642000004*t1467;
  t1530 = -0.05226439969100001*t1467;
  t1485 = 0.49726168403800003*t1467;
  t1581 = -0.073913*t1470;
  t1499 = 0.994522*t1197*t1208*t1235;
  t1502 = 0.103955395616*t1232*t1424;
  t1503 = -0.9890740084840001*t1232;
  t1504 = 1. + t1503;
  t1507 = t1504*t1445;
  t1511 = t1499 + t1502 + t1507;
  t1566 = -0.703234*t1470;
  t1546 = -0.104528*t1197*t1208*t1235;
  t1554 = -0.010926102783999999*t1232;
  t1555 = 1. + t1554;
  t1556 = t1555*t1424;
  t1557 = 0.103955395616*t1232*t1445;
  t1561 = t1546 + t1556 + t1557;
  t1588 = -1.0000001112680001*t1232;
  t1592 = 1. + t1588;
  t1599 = t1592*t1197*t1208;
  t1600 = 0.104528*t1235*t1424;
  t1601 = -0.994522*t1235*t1445;
  t1603 = t1599 + t1600 + t1601;
  t1606 = Cos(var1[17]);
  t1608 = -1.*t1606;
  t1609 = 1. + t1608;
  t1611 = Sin(var1[17]);
  t1475 = -0.707107*t1470;
  t1615 = -0.49726168403800003*t1467;
  t1487 = 0.073913*t1470;
  t1653 = -0.051978134642000004*t1467;
  t1539 = 0.707107*t1470;
  t1622 = 0.05226439969100001*t1467;
  t1532 = 0.703234*t1470;
  t1616 = t1615 + t1581;
  t1620 = t1616*t1511;
  t1623 = t1622 + t1566;
  t1624 = t1623*t1561;
  t1625 = -0.500001190325*t1467;
  t1626 = 1. + t1625;
  t1627 = t1626*t1603;
  t1631 = t1620 + t1624 + t1627;
  t1642 = -0.5054634410180001*t1467;
  t1644 = 1. + t1642;
  t1652 = t1644*t1511;
  t1654 = t1653 + t1475;
  t1655 = t1654*t1561;
  t1656 = t1615 + t1487;
  t1657 = t1656*t1603;
  t1660 = t1652 + t1655 + t1657;
  t1667 = t1653 + t1539;
  t1668 = t1667*t1511;
  t1669 = -0.9945383682050002*t1467;
  t1671 = 1. + t1669;
  t1675 = t1671*t1561;
  t1679 = t1622 + t1532;
  t1685 = t1679*t1603;
  t1687 = t1668 + t1675 + t1685;
  t1689 = -0.104528*t1611*t1631;
  t1690 = 0.103955395616*t1609*t1660;
  t1692 = -0.010926102783999999*t1609;
  t1695 = 1. + t1692;
  t1696 = t1695*t1687;
  t1698 = t1689 + t1690 + t1696;
  t1700 = 0.994522*t1611*t1631;
  t1702 = -0.9890740084840001*t1609;
  t1704 = 1. + t1702;
  t1705 = t1704*t1660;
  t1707 = 0.103955395616*t1609*t1687;
  t1709 = t1700 + t1705 + t1707;
  t1713 = -1.0000001112680001*t1609;
  t1716 = 1. + t1713;
  t1717 = t1716*t1631;
  t1719 = -0.994522*t1611*t1660;
  t1720 = 0.104528*t1611*t1687;
  t1721 = t1717 + t1719 + t1720;
  t1233 = -0.056500534356700764*t1232;
  t1265 = -0.3852490428658858*t1235;
  t1266 = t1233 + t1265;
  t1317 = 0.4*t1303;
  t1334 = 0.12*t1326;
  t1335 = t1317 + t1334;
  t1379 = 0.12*t1303;
  t1385 = -0.4*t1326;
  t1387 = t1379 + t1385;
  t1398 = 1.1345904784751044e-7*var1[15];
  t1409 = -0.0402693119526853*t1232;
  t1411 = 0.0059058871981009595*t1235;
  t1415 = t1398 + t1409 + t1411;
  t1743 = t1336*t1357*t1342;
  t1744 = -1.*t1197*t1362;
  t1746 = t1743 + t1744;
  t1749 = t1197*t1336;
  t1750 = t1357*t1342*t1362;
  t1755 = t1749 + t1750;
  t1427 = 1.1924972351948546e-8*var1[15];
  t1429 = 0.3831386486090665*t1232;
  t1430 = -0.05619101817723254*t1235;
  t1438 = t1427 + t1429 + t1430;
  t1459 = -4.0332087336819504e-7*var1[16];
  t1468 = 0.0958179942122405*t1467;
  t1480 = t1469 + t1475;
  t1484 = 0.23105307644*t1480;
  t1489 = t1485 + t1487;
  t1490 = 0.164374659834*t1489;
  t1491 = t1459 + t1468 + t1484 + t1490;
  t1757 = -1.*t1326*t1746;
  t1758 = t1272*t1755;
  t1759 = t1757 + t1758;
  t1762 = t1272*t1746;
  t1763 = t1326*t1755;
  t1764 = t1762 + t1763;
  t1526 = 4.239080549754904e-8*var1[16];
  t1529 = 0.22979114961138278*t1467;
  t1533 = t1530 + t1532;
  t1536 = 0.164374659834*t1533;
  t1540 = t1469 + t1539;
  t1541 = 0.189564637987*t1540;
  t1542 = t1526 + t1529 + t1536 + t1541;
  t1563 = 4.05542127947119e-7*var1[16];
  t1564 = 0.08218752557626696*t1467;
  t1568 = t1530 + t1566;
  t1579 = 0.23105307644*t1568;
  t1585 = t1485 + t1581;
  t1586 = 0.189564637987*t1585;
  t1587 = t1563 + t1564 + t1579 + t1586;
  t1610 = 0.19098732144477495*t1609;
  t1613 = -0.13776101532839094*t1611;
  t1614 = t1610 + t1613;
  t1766 = 0.994522*t1208*t1235*t1357;
  t1767 = 0.103955395616*t1232*t1759;
  t1768 = t1504*t1764;
  t1770 = t1766 + t1767 + t1768;
  t1772 = -0.104528*t1208*t1235*t1357;
  t1773 = t1555*t1759;
  t1774 = 0.103955395616*t1232*t1764;
  t1776 = t1772 + t1773 + t1774;
  t1779 = t1592*t1208*t1357;
  t1782 = 0.104528*t1235*t1759;
  t1783 = -0.994522*t1235*t1764;
  t1784 = t1779 + t1782 + t1783;
  t1633 = -5.06291820800569e-8*var1[17];
  t1634 = 0.13700636048642204*t1609;
  t1638 = 0.18994107176353728*t1611;
  t1639 = t1633 + t1634 + t1638;
  t1662 = -4.817066759205414e-7*var1[17];
  t1664 = -0.014399883410246048*t1609;
  t1665 = -0.019963520514678434*t1611;
  t1666 = t1662 + t1664 + t1665;
  t1787 = t1616*t1770;
  t1788 = t1623*t1776;
  t1790 = t1626*t1784;
  t1792 = t1787 + t1788 + t1790;
  t1794 = t1644*t1770;
  t1795 = t1654*t1776;
  t1797 = t1656*t1784;
  t1798 = t1794 + t1795 + t1797;
  t1802 = t1667*t1770;
  t1805 = t1671*t1776;
  t1806 = t1679*t1784;
  t1807 = t1802 + t1805 + t1806;
  t1810 = -0.104528*t1611*t1792;
  t1811 = 0.103955395616*t1609*t1798;
  t1812 = t1695*t1807;
  t1814 = t1810 + t1811 + t1812;
  t1818 = 0.994522*t1611*t1792;
  t1823 = t1704*t1798;
  t1828 = 0.103955395616*t1609*t1807;
  t1829 = t1818 + t1823 + t1828;
  t1832 = t1716*t1792;
  t1833 = -0.994522*t1611*t1798;
  t1834 = 0.104528*t1611*t1807;
  t1836 = t1832 + t1833 + t1834;
  t1738 = Sin(var2[0]);
  t1268 = t1197*t1208*t1266;
  t1367 = t1335*t1365;
  t1396 = t1387*t1390;
  t1426 = t1415*t1424;
  t1446 = t1438*t1445;
  t1515 = t1491*t1511;
  t1562 = t1542*t1561;
  t1605 = t1587*t1603;
  t1632 = t1614*t1631;
  t1661 = t1639*t1660;
  t1688 = t1666*t1687;
  t1699 = 0.272124*t1698;
  t1712 = 0.167122*t1709;
  t1722 = 0.190987*t1721;
  t1723 = 0.994522*t1698;
  t1724 = 0.104528*t1709;
  t1726 = t1723 + t1724;
  t1727 = -0.07912*t1726;
  t1729 = -0.040001*t1698;
  t1730 = 0.380588*t1709;
  t1732 = 0.92388*t1721;
  t1733 = t1729 + t1730 + t1732;
  t1735 = 0.369*t1733;
  t1736 = var1[0] + t1268 + t1367 + t1396 + t1426 + t1446 + t1515 + t1562 + t1605 + t1632 + t1661 + t1688 + t1699 + t1712 + t1722 + t1727 + t1735;
  t1150 = Cos(var2[0]);
  t1742 = t1208*t1266*t1357;
  t1748 = t1335*t1746;
  t1756 = t1387*t1755;
  t1761 = t1415*t1759;
  t1765 = t1438*t1764;
  t1771 = t1491*t1770;
  t1778 = t1542*t1776;
  t1785 = t1587*t1784;
  t1793 = t1614*t1792;
  t1801 = t1639*t1798;
  t1808 = t1666*t1807;
  t1815 = 0.272124*t1814;
  t1830 = 0.167122*t1829;
  t1837 = 0.190987*t1836;
  t1838 = 0.994522*t1814;
  t1839 = 0.104528*t1829;
  t1840 = t1838 + t1839;
  t1841 = -0.07912*t1840;
  t1842 = -0.040001*t1814;
  t1843 = 0.380588*t1829;
  t1844 = 0.92388*t1836;
  t1845 = t1842 + t1843 + t1844;
  t1846 = 0.369*t1845;
  t1847 = var1[1] + t1742 + t1748 + t1756 + t1761 + t1765 + t1771 + t1778 + t1785 + t1793 + t1801 + t1808 + t1815 + t1830 + t1837 + t1841 + t1846;
  t1856 = -1.*t1208*t1336*t1326;
  t1857 = t1272*t1208*t1362;
  t1858 = t1856 + t1857;
  t1863 = t1272*t1208*t1336;
  t1864 = t1208*t1326*t1362;
  t1865 = t1863 + t1864;
  t1868 = -0.994522*t1235*t1342;
  t1869 = 0.103955395616*t1232*t1858;
  t1870 = t1504*t1865;
  t1871 = t1868 + t1869 + t1870;
  t1875 = 0.104528*t1235*t1342;
  t1876 = t1555*t1858;
  t1881 = 0.103955395616*t1232*t1865;
  t1885 = t1875 + t1876 + t1881;
  t1889 = -1.*t1592*t1342;
  t1891 = 0.104528*t1235*t1858;
  t1892 = -0.994522*t1235*t1865;
  t1893 = t1889 + t1891 + t1892;
  t1895 = t1616*t1871;
  t1896 = t1623*t1885;
  t1897 = t1626*t1893;
  t1898 = t1895 + t1896 + t1897;
  t1901 = t1644*t1871;
  t1902 = t1654*t1885;
  t1903 = t1656*t1893;
  t1905 = t1901 + t1902 + t1903;
  t1907 = t1667*t1871;
  t1909 = t1671*t1885;
  t1910 = t1679*t1893;
  t1911 = t1907 + t1909 + t1910;
  t1914 = -0.104528*t1611*t1898;
  t1915 = 0.103955395616*t1609*t1905;
  t1916 = t1695*t1911;
  t1918 = t1914 + t1915 + t1916;
  t1920 = 0.994522*t1611*t1898;
  t1922 = t1704*t1905;
  t1923 = 0.103955395616*t1609*t1911;
  t1924 = t1920 + t1922 + t1923;
  t1926 = t1716*t1898;
  t1927 = -0.994522*t1611*t1905;
  t1928 = 0.104528*t1611*t1911;
  t1929 = t1926 + t1927 + t1928;
  p_output1[0]=t1150*t1736 + t1738*t1847;
  p_output1[1]=-1.*t1736*t1738 + t1150*t1847;
  p_output1[2]=t1208*t1335*t1336 - 1.*t1266*t1342 + t1208*t1362*t1387 + t1415*t1858 + t1438*t1865 + t1491*t1871 + t1542*t1885 + t1587*t1893 + t1614*t1898 + t1639*t1905 + t1666*t1911 + 0.272124*t1918 - 0.07912*(0.994522*t1918 + 0.104528*t1924) + 0.167122*t1924 + 0.369*(-0.040001*t1918 + 0.380588*t1924 + 0.92388*t1929) + 0.190987*t1929 + var1[2];
}



void gen::kin::p_left_hand(Eigen::Ref<Eigen::VectorXd> p_output1, const Eigen::Ref<const Eigen::VectorXd> var1,const Eigen::Ref<const Eigen::VectorXd> var2)
{
  // Call Subroutines
  output1(p_output1, var1, var2);

}
