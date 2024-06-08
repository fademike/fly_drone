/*
 * fast_atan.c
 *
 *  Created on: 10 ���� 2020 �.
 *      Author: NASA
 */


#include "fast_atan.h"
#include <math.h>

#define M_PI_4_P_0273	1.05839816339744830962 //M_PI/4 + 0.273

const double ATAN_LUT[256] = {0.0000000000,0.0039215485,0.0078429764,0.0117641631,0.0156849881,0.0196053309,
                              0.0235250710,0.0274440882,0.0313622624,0.0352794736,0.0391956019,0.0431105278,
                              0.0470241318,0.0509362949,0.0548468980,0.0587558227,0.0626629506,0.0665681638,
                              0.0704713446,0.0743723758,0.0782711405,0.0821675224,0.0860614053,0.0899526737,
                              0.0938412126,0.0977269074,0.1016096438,0.1054893085,0.1093657884,0.1132389710,
                              0.1171087446,0.1209749978,0.1248376255,0.1286965013,0.1325515323,0.1364026044,
                              0.1402496096,0.1440924408,0.1479309912,0.1517651553,0.1555948280,0.1594199049,
                              0.1632402828,0.1670558588,0.1708665312,0.1746721990,0.1784727620,0.1822681208,
                              0.1860581771,0.1898428334,0.1936219929,0.1973955598,0.2011634395,0.2049255380,
                              0.2086817623,0.2124320205,0.2161762215,0.2199142752,0.2236460927,0.2273715857,
                              0.2310906672,0.2348032511,0.2385092525,0.2422085871,0.2459011721,0.2495869254,
                              0.2532657662,0.2569376146,0.2606023917,0.2642600199,0.2679104224,0.2715535237,
                              0.2751892491,0.2788175253,0.2824382800,0.2860514417,0.2896569404,0.2932547070,
                              0.2968446734,0.3004267728,0.3040009393,0.3075671084,0.3111252164,0.3146752558,
                              0.3182170002,0.3217505544,0.3252758042,0.3287926915,0.3323011594,0.3358011520,
                              0.3392926145,0.3427754932,0.3462497357,0.3497152904,0.3531721069,0.3566201360,
                              0.3600593294,0.3634896400,0.3669110217,0.3703234297,0.3737268255,0.3771211497,
                              0.3805063771,0.3838824615,0.3872493632,0.3906070437,0.3939554653,0.3972945915,
                              0.4006243869,0.4039448169,0.4072558481,0.4105574480,0.4138495853,0.4171322295,
                              0.4204053512,0.4236689219,0.4269229141,0.4301673014,0.4334020581,0.4366271598,
                              0.4398425828,0.4430483044,0.4462443029,0.4494305575,0.4526070482,0.4557737560,
                              0.4589306629,0.4620777516,0.4652150058,0.4683424102,0.4714599501,0.4745676117,
                              0.4776653824,0.4807532499,0.4838312032,0.4868992318,0.4899573263,0.4930054778,
                              0.4960436784,0.4990719209,0.5020901990,0.5050985071,0.5080968402,0.5110851942,
                              0.5140635659,0.5170319525,0.5199903521,0.5229387636,0.5258771863,0.5288056206,
                              0.5317240673,0.5346325278,0.5375310045,0.5404195003,0.5432980185,0.5461665634,
                              0.5490251398,0.5518737530,0.5547124091,0.5575411147,0.5603598769,0.5631687036,
                              0.5659676030,0.5687565842,0.5715356566,0.5743048302,0.5770641155,0.5798135236,
                              0.5825530662,0.5852827553,0.5880026035,0.5907126240,0.5934128303,0.5961032364,
                              0.5987838570,0.6014547069,0.6041158015,0.6067671569,0.6094087892,0.6120407151,
                              0.6146629519,0.6172755171,0.6198784285,0.6224717045,0.6250553640,0.6276294258,
                              0.6301939095,0.6327488350,0.6352942223,0.6378300921,0.6403564651,0.6428733625,
                              0.6453808058,0.6478788169,0.6503674179,0.6528466311,0.6553164793,0.6577769856,
                              0.6602281731,0.6626700655,0.6651026865,0.6675260602,0.6699402110,0.6723451634,
                              0.6747409422,0.6771275725,0.6795050796,0.6818734889,0.6842328261,0.6865831172,
                              0.6889243882,0.6912566655,0.6935799756,0.6958943450,0.6981998008,0.7004963699,
                              0.7027840796,0.7050629571,0.7073330300,0.7095943260,0.7118468729,0.7140906986,
                              0.7163258312,0.7185522990,0.7207701302,0.7229793534,0.7251799971,0.7273720900,
                              0.7295556609,0.7317307387,0.7338973524,0.7360555311,0.7382053040,0.7403467003,
                              0.7424797493,0.7446044805,0.7467209234,0.7488291075,0.7509290624,0.7530208178,
                              0.7551044035,0.7571798492,0.7592471847,0.7613064400,0.7633576449,0.7654008294,
                              0.7674360235,0.7694632573,0.7714825607,0.7734939638,0.7754974968,0.7774931897,
                              0.7794810727,0.7814611759,0.7834335294,0.7853981634};
double atan2LUTif(double y,double x)
{
  double absx, absy, val;

  if (x == 0 && y == 0) {
      return 0;
    }
  absy = fabs(y);
  absx = fabs(x);
  if (absy - absx == absy) {
      return y < 0 ? -M_PI_2 : M_PI_2;
    }
  if (absx - absy == absx) {
      val = 0.0;
    }
  else
    {
      if (y>0) {
          if (absx > absy)
            val = ATAN_LUT[(int)(255*absy/absx)];//1st octant
          else
            val = M_PI_2 - ATAN_LUT[(int)(255*absx/absy)];//2nd octant
          val = x < 0 ? (M_PI - val) : val; //3-4th octants from 2-1
        }
      else {
          if (absx > absy)
            val = -ATAN_LUT[(int)(255*absy/absx)];//8th octant
          else
            val = -M_PI_2 + ATAN_LUT[(int)(255*absx/absy)];//7th octant
          val = x < 0 ? -M_PI - val : val; //5-6th octants from 8-7
        }
    }

  return val;

}
double atan2LUT(double y,double x)
{
  double absx, absy;
  absy = fabs(y);
  absx = fabs(x);
  short octant = ((x<0) << 2) + ((y<0) << 1 ) + (absx <= absy);
  switch (octant) {
    case 0: {
        if (x == 0 && y == 0)
          return 0;
        return ATAN_LUT[(int)(255*absy/absx)]; //1st octant
        break;
      }
    case 1:{
        if (x == 0 && y == 0)
          return 0.0;
        return M_PI_2 - ATAN_LUT[(int)(255*absx/absy)]; //2nd octant
        break;
      }
    case 2: {
        return -ATAN_LUT[(int)(255*absy/absx)]; //8th octant
        break;
      }
    case 3: {
        return -M_PI_2 + ATAN_LUT[(int)(255*absx/absy)];//7th octant
        break;
      }
    case 4: {
        return  M_PI - ATAN_LUT[(int)(255*absy/absx)];  //4th octant
      }
    case 5: {
        return  M_PI_2 + ATAN_LUT[(int)(255*absx/absy)];//3rd octant
        break;
      }
    case 6: {
        return -M_PI + ATAN_LUT[(int)(255*absy/absx)]; //5th octant
        break;
      }
    case 7: {
        return -M_PI_2 - ATAN_LUT[(int)(255*absx/absy)]; //6th octant
        break;
      }
    default:
      return 0.0;
    }
}
double atan2PI_4(double y,double x)
{
  double absx, absy;
  absy = fabs(y);
  absx = fabs(x);
  short octant = ((x<0) << 2) + ((y<0) << 1 ) + (absx <= absy);
  switch (octant) {
    case 0: {
        if (x == 0 && y == 0)
          return 0;
        return M_PI_4*absy/absx; //1st octant
        break;
      }
    case 1:{
        if (x == 0 && y == 0)
          return 0.0;
        return M_PI_2 - M_PI_4*absx/absy; //2nd octant
        break;
      }
    case 2: {
        return -M_PI_4*absy/absx; //8th octant
        break;
      }
    case 3: {
        return -M_PI_2 + M_PI_4*absx/absy;//7th octant
        break;
      }
    case 4: {
        return  M_PI - M_PI_4*absy/absx;  //4th octant
      }
    case 5: {
        return  M_PI_2 + M_PI_4*absx/absy;//3rd octant
        break;
      }
    case 6: {
        return -M_PI + M_PI_4*absy/absx; //5th octant
        break;
      }
    case 7: {
        return -M_PI_2 - M_PI_4*absx/absy; //6th octant
        break;
      }
    default:
      return 0.0;
    }
}

double atan2approx(double y,double x)
{
  double absx, absy;
  absy = fabs(y);
  absx = fabs(x);
  short octant = ((x<0) << 2) + ((y<0) << 1 ) + (absx <= absy);
  switch (octant) {
    case 0: {
        if (x == 0 && y == 0)
          return 0;
        double val = absy/absx;
        return (M_PI_4_P_0273 - 0.273*val)*val; //1st octant
        break;
      }
    case 1:{
        if (x == 0 && y == 0)
          return 0.0;
        double val = absx/absy;
        return M_PI_2 - (M_PI_4_P_0273 - 0.273*val)*val; //2nd octant
        break;
      }
    case 2: {
        double val =absy/absx;
        return -(M_PI_4_P_0273 - 0.273*val)*val; //8th octant
        break;
      }
    case 3: {
        double val =absx/absy;
        return -M_PI_2 + (M_PI_4_P_0273 - 0.273*val)*val;//7th octant
        break;
      }
    case 4: {
        double val =absy/absx;
        return  M_PI - (M_PI_4_P_0273 - 0.273*val)*val;  //4th octant
      }
    case 5: {
        double val =absx/absy;
        return  M_PI_2 + (M_PI_4_P_0273 - 0.273*val)*val;//3rd octant
        break;
      }
    case 6: {
        double val =absy/absx;
        return -M_PI + (M_PI_4_P_0273 - 0.273*val)*val; //5th octant
        break;
      }
    case 7: {
        double val =absx/absy;
        return -M_PI_2 - (M_PI_4_P_0273 - 0.273*val)*val; //6th octant
        break;
      }
    default:
      return 0.0;
    }
}





#define M_PIf       3.14159265358979323846f

#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
#define ABS(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })

// Initial implementation by Crashpilot1000 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
// Polynomial coefficients by Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
// Max absolute error 0,000027 degree
// atan2_approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
float atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}



