/*
 * fast_atan.h
 *
 *  Created on: 10 θώνÿ 2020 γ.
 *      Author: NASA
 */

#ifndef FAST_ATAN_H_
#define FAST_ATAN_H_


/* Approximated fucntions are taken from
 Rajan, S.; Wang, S.; Inkol, R. & Joyal, A.
 Efficient approximations for the arctangent function
 Signal Processing Magazine, IEEE, 2006, 23, 108-111*/

double atan2PI_4(double y,double x);	//425us
/*atan(x) ~ x*(pi/4) */

double atan2approx(double y,double x);
/*atan(x) ~ x*(pi/4+0.273-0.273*x) */


double atan2LUT(double y,double x);
double atan2LUTif(double y,double x);	//450us
/* switch/if versions, compiler dependent which is faster*/




#endif /* FAST_ATAN_H_ */
