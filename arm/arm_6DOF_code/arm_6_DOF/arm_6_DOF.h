#pragma once
#include "stdio.h"
#include <Eigen/LU>
#define pi 3.141592653589793
class MyArm
{
public:
	double theta1, d1 = 0,     a0 = 0,    alpha0 = 0,       offset1 = 0;
	double theta2, d2 = 0,     a1 = 0,    alpha1 = -pi / 2, offset2 = 0;
	double theta3, d3 = 0,     a2 = 180,  alpha2 = 0,       offset3 = 0;
	double theta4, d4 = 170.8, a3 = 0,    alpha3 = -pi / 2, offset4 = 0;
	double theta5, d5 = 0,     a4 = 0,    alpha4 = pi / 2,  offset5 = 0;
	double theta6, d6 = 0,     a5 = 0,    alpha5 = -pi / 2, offset6 = 0;

};
void arm_Fk(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6,int mode);
double* arm_Ik(double x, double y, double z, double roll, double pit, double yaw);
void multiply(double a[4][4], double b[4][4], double c[4][4]);