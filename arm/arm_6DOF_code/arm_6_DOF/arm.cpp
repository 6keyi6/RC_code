//#include <iostream>
//#include "arm.h"
//#include <complex.h>
//#include <math.h>
//#include <complex>
//#include <cmath>
//#define angle 180./pi
//#define radian pi/180.
//using namespace std;
//MyArm arm;
//double Target[3] = { 0 };
//double Target1[3] = { 0 };
//Eigen::Matrix<double, 3, 3> R03;
//Eigen::Matrix<double, 3, 3> R06;
///*****by 可抑
//******矩阵相乘
//******输入 a、b为计算矩阵  c储存矩阵
//******输出 无
//*/
//void multiply(double a[4][4], double b[4][4], double c[4][4]) {
//	double temp[4][4];
//	for (int i = 0; i < 4; i++) {
//		for (int j = 0; j < 4; j++) {
//			temp[i][j] = 0;
//			for (int k = 0; k < 4; k++) {
//				temp[i][j] += a[i][k] * b[k][j];
//			}
//		}
//	}
//	for (int i = 0; i < 4; i++) {
//		for (int j = 0; j < 4; j++) {
//			c[i][j] = temp[i][j];
//		}
//	}
//}
//
//double rrr(double num) {
//	return round(num * 1000) / 1000;
//}
////double x = -12.538, y = 0, z = -157.191, roll =180* radian, pit = -72 * radian, yaw = 0 * radian;//
////double x =-10.143,y = -7.369, z = -157.191, roll = -67 * radian, pit = -50.3* radian, yaw=118.9* radian;
////double x = 245.766, y = - 178.560, z = 54.412, roll = -73.8 * radian, pit = 12.5 * radian, yaw = -143.4 * radian;
//
//
//int main()
//{
//	arm_Ik(145.623, 105.801, -170.8, 0 * radian, 90 * radian, -180 * radian);
//	return 0;
//}
//
//
//
//
//
///*****Author 可抑
//******运动学逆解
//******输入 xyzrpy
//******输出 各个关节角度
//*/
//double result[6] = { 0 };
//double* arm_Ik(double x, double y, double z, double roll, double pit, double yaw)
//{
//	double A, B;
//	double a, b, c;
//	double u1 = 0, u2 = 0;//引入变量u1 u2  
//	double angle3_result1, angle3_result2, angle2_result1, angle2_result2, angle1_result1, angle1_result2;
//	double t;
//	
//	arm.theta1 = atan2(y, x) - atan2(0, 1);
//	a = x * cos(arm.theta1) + y * sin(arm.theta1);
//	b = 854 / 5;
//	t = pow(arm.a2,2) + pow(b, 2) - pow(a, 2) - pow(z, 2);
//	arm.theta3 = atan2(t, sqrt(1 - pow(t, 2)));
//
//
//    std::cout << round(arm.theta1 * angle) << endl;
//	std::cout << round(arm.theta2 * angle) << endl;
//
//
//	result[0] = arm.theta1;
//	result[1] = arm.theta2;
//	result[2] = arm.theta3;
//	result[3] = arm.theta4 = 0;
//	result[4] = arm.theta5 = 0;
//	result[5] = arm.theta6 = 0;
//
//	return result;
//}
//
//
//
///*****Author 可抑
//******运动学正解
//******输入 xyzrpy
//******输出 各个关节角度
//*/
//void arm_Fk(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6, int mode)
//{
//	//                      x轴转角（输入量） x轴相差距离 z轴相差距离   z轴 相差角
//	double	MDH[6][4] = { theta1,       arm.d1,      arm.a0,       arm.alpha0,
//							theta2,       arm.d2,      arm.a1,       arm.alpha1,
//							theta3,       arm.d3,      arm.a2,       arm.alpha2,
//							theta4,       arm.d4,      arm.a3,       arm.alpha3,
//							theta5,       arm.d5,      arm.a4,       arm.alpha4,
//							theta6,       arm.d6,      arm.a5,       arm.alpha5 };
//	double c1 = cos(MDH[0][0]), s1 = sin(MDH[0][0]);
//	double c2 = cos(MDH[1][0]), s2 = sin(MDH[1][0]);
//	double c3 = cos(MDH[2][0]), s3 = sin(MDH[2][0]);
//	double c4 = cos(MDH[3][0]), s4 = sin(MDH[3][0]);
//	double c5 = cos(MDH[4][0]), s5 = sin(MDH[4][0]);
//	double c6 = cos(MDH[5][0]), s6 = sin(MDH[5][0]);
//
//	double ca1 = cos(MDH[0][3]), sa1 = sin(MDH[0][3]);
//	double ca2 = cos(MDH[1][3]), sa2 = sin(MDH[1][3]);
//	double ca3 = cos(MDH[2][3]), sa3 = sin(MDH[2][3]);
//	double ca4 = cos(MDH[3][3]), sa4 = sin(MDH[3][3]);
//	double ca5 = cos(MDH[4][3]), sa5 = sin(MDH[4][3]);
//	double ca6 = cos(MDH[5][3]), sa6 = sin(MDH[5][3]);
//
//	double T01[4][4] = { c1       ,        -s1    ,          0       ,         arm.a0,
//						s1 * ca1  ,     c1 * ca1    ,       -sa1       ,     -sa1 * arm.d1,
//						s1 * sa1  ,     c1 * sa1    ,         ca1      ,      ca1 * arm.d1,
//						0        ,        0      ,          0       ,           1 };
//	double T12[4][4] = { c2       ,        -s2    ,          0       ,         arm.a1,
//						s2 * ca2  ,     c2 * ca2    ,       -sa2       ,     -sa2 * arm.d2,
//						s2 * sa2  ,     c2 * sa2    ,         ca2      ,      ca2 * arm.d2,
//						0        ,        0      ,          0       ,           1 };
//	double T23[4][4] = { c3       ,        -s3    ,          0       ,         arm.a2,
//						s3 * ca3  ,     c3 * ca3    ,       -sa3       ,     -sa3 * arm.d3,
//						s3 * sa3  ,     c3 * sa3    ,         ca3      ,      ca3 * arm.d3,
//						0        ,        0      ,          0       ,           1 };
//	double T34[4][4] = { c4       ,        -s4    ,          0       ,         arm.a3,
//						s4 * ca4  ,     c4 * ca4    ,       -sa4       ,     -sa4 * arm.d4,
//						s4 * sa4  ,     c4 * sa4    ,         ca4      ,      ca4 * arm.d4,
//						0        ,        0      ,          0       ,           1 };
//	double T45[4][4] = { c5       ,        -s5    ,          0       ,         arm.a4,
//						s5 * ca5  ,     c5 * ca5    ,       -sa5       ,     -sa5 * arm.d5,
//						s5 * sa5  ,     c5 * sa5    ,         ca5      ,      ca5 * arm.d5,
//						0        ,        0      ,          0       ,           1 };
//	double T56[4][4] = { c6       ,        -s6    ,          0       ,         arm.a5,
//						s6 * ca6  ,     c6 * ca6    ,       -sa6       ,     -sa6 * arm.d6,
//						s6 * sa6  ,     c6 * sa6   ,         ca6      ,      ca6 * arm.d6,
//						0        ,        0      ,          0       ,           1 };
//	double T06[4][4];
//
//
//	for (int i = 0; i < 4; i++) {
//		for (int j = 0; j < 4; j++) {
//			T06[i][j] = (i == j) ? 1 : 0;  // 单位矩阵
//		}
//	}
//	// 计算矩阵C = A1 * A2 * A3 * A4 * A5 * A6
//
//	multiply(T06, T01, T06);
//	multiply(T06, T12, T06);
//	multiply(T06, T23, T06);
//	multiply(T06, T34, T06);
//	multiply(T06, T45, T06);
//	multiply(T06, T56, T06);
//
//	// 输出矩阵C
//	for (int i = 0; i < 4; i++) {
//		for (int j = 0; j < 4; j++) {
//			if (fabs(T06[i][j]) < 0.0000001)
//			{
//				T06[i][j] = 0;
//			}
//			cout << T06[i][j] << "\t ";
//		}
//		cout << endl;
//	}
//	if (mode == 1)
//	{
//		for (int i = 0; i < 3; i++)
//		{
//			Target[i] = T06[i][3];
//		}
//	}
//	if (mode == 2)
//	{
//		for (int i = 0; i < 3; i++)
//		{
//			Target1[i] = T06[i][3];
//		}
//	}
//
//	//if (mode == 4)
//	//{
//	//	for (int i = 0; i < 3; ++i) {
//	//		for (int j = 0; j < 3; ++j) {
//	//			R06(i, j) = T06[i][j];
//	//		}
//	//	}
//	//}
//
//	return;
//}
