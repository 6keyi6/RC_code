#include <iostream>
#include "arm_6_DOF.h"
#include <complex.h>
#include <math.h>
#include <complex>
#include <cmath>
#define angle 180./pi
#define radian pi/180.
using namespace std;
MyArm arm;
double Target[3] = {0};
double Target1[3] = { 0 };
/*****by 可抑
******矩阵相乘
******输入 a、b为计算矩阵  c储存矩阵
******输出 无
*/
void multiply(double a[4][4], double b[4][4], double c[4][4]) {
	double temp[4][4];
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			temp[i][j] = 0;
			for (int k = 0; k < 4; k++) {
				temp[i][j] += a[i][k] * b[k][j];
			}
		}
	}
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			c[i][j] = temp[i][j];
		}
	}
}

double rrr(double num) {
	return round(num * 1000) / 1000;
}

int main()
{
	//arm_Ik(9.102, 0, 350.669, -180, 2, 0);
	//arm_Ik(173.915, 0, 176.992, 0, 89, -180);
	//arm_Ik(184.928, -115.556, 223.970, 116.3, 53.8, 58.5);
	arm_Ik(180, 0, -170.8, 0, 0, -180);
    return 0;
}





/*****Author 可抑
******运动学正解
******输入 xyzrpy
******输出 各个关节角度
*/
double result[6] = {0};
double* arm_Ik(double x, double y, double z, double roll, double pit, double yaw)
{
	double A, B;
	double a, b, c;
	double u1 = 0, u2 = 0;//引入变量u1 u2  
	double angle3_result1, angle3_result2, angle2_result1, angle2_result2, angle1_result1, angle1_result2;

	/*****************************************三轴角度计算*********************************************/
	A = pow(arm.a2, 2) + pow(arm.d4, 2) - pow(x, 2) - pow(y, 2) - pow(z, 2);
	B = 2 * arm.a2 * arm.d4;

	//输入范围[-1,1],输出 [-π/2, π/2]
	angle3_result1 = asin(A / B);

	//去除等于0时的精度问题
	if (fabs(angle3_result1) < 0.000001)angle3_result1 = 0;

	/*****************************************二轴角度计算*********************************************/
	a = -arm.d4 * cos(angle3_result1);
	b = (arm.d4 * sin(angle3_result1) - arm.a2);
	c = z;

	//第一种解法
	if (a + c != 0)
	{
		if (pow(a, 2) + pow(b, 2) >= pow(c, 2))
		{
			u1 = (b + sqrt(pow(a, 2) + pow(b, 2) - pow(c, 2))) / (a + c);
			u2 = (b - sqrt(pow(a, 2) + pow(b, 2) - pow(c, 2))) / (a + c);
		}
		else std::cout << "角度二在a+c!=0无解" << endl;
	}
	else if (a + c == 0)
	{
		if (b != 0)
		{
			u2 = u1 = (c - a) / (2 * b);
		}
		else std::cout << "角度二在a+c==0无解" << endl;
	}
	else std::cout << "角度二无解" << endl;

	angle2_result1 = atan((2 * u1) / (1 - pow(u1, 2)));//输 入: (-∞,+∞) 输出的时[-pi/2,pi/2]
	angle2_result2 = atan((2 * u2) / (1 - pow(u2, 2)));//输 入: (-∞,+∞) 输出的时[-pi/2,pi/2]

	if (fabs(angle2_result1) < 0.000001) angle2_result1 = 0;
	if (fabs(angle2_result2) < 0.000001) angle2_result2 = 0;

	/*****************************************一轴角度计算*********************************************/
	angle1_result1 = pi - acos(x / (arm.d4 * cos(angle2_result1) * sin(angle3_result1) - arm.a2 * cos(angle2_result1) + arm.d4 * cos(angle3_result1) * sin(angle2_result1)));
	angle1_result2 = pi - acos(x / (arm.d4 * cos(angle2_result2) * sin(angle3_result1) - arm.a2 * cos(angle2_result2) + arm.d4 * cos(angle3_result1) * sin(angle2_result2)));

	if (fabs(angle1_result1) < 0.000001)angle1_result1 = 0;
	if (fabs(angle1_result2) < 0.000001)angle1_result2 = 0;
	
	//进行验证
	angle1_result1 = -angle1_result1;
	
	arm_Fk(angle1_result1, angle2_result1, angle3_result1, 0, 0, 0, 1);

	arm_Fk(angle1_result2, angle2_result2, angle3_result1, 0, 0, 0, 2);

	
	if ((round(Target[1]) - round(y) == 0&& (round(Target[2]) - round(z) == 0)))//先判断角度二是否与目标值相等
	{
		//注意是弧度制
		std::cout << "************1***********" << endl;
		arm.theta1 = angle1_result1;
		arm.theta2 = angle2_result1;
		arm.theta3 = angle3_result1;
		std::cout << round(arm.theta1 * angle) << endl;
		std::cout << round(arm.theta2 * angle) << endl;
		std::cout << round(arm.theta3 * angle) << endl;
		result[0] = arm.theta1;
		result[1] = arm.theta2;
		result[2] = arm.theta3;
		/* arm_Fk(angle1_result1, angle2_result1, angle3_result1, 0, 0, 0, 4);*/
	}
	//else if(round(Target[1]) == -round(y))
	//{
	//	
	//}
	else if ((round(Target[1]) - round(y) == 0 || round(Target[1]) == round(-y)) && (round(Target[2]) - round(z) == 0 || round(Target[2]) == round(-z)))
	{
		std::cout << "************2***********" << endl;
		arm.theta1 = angle1_result2;
		arm.theta2 = angle2_result2;
		arm.theta3 = angle3_result1;
		std::cout << round(arm.theta1 * angle) << endl;
		std::cout << round(arm.theta2 * angle) << endl;
		std::cout << round(arm.theta3 * angle) << endl;
		result[0] = arm.theta1;
		result[1] = arm.theta2;
		result[2] = arm.theta3;
		/* arm_Fk(angle1_result2, angle2_result2, angle3_result1, 0, 0, 0, 4);*/
	}
	std::cout << "********************************************************************" << endl;
	

	Eigen::Matrix<double, 3, 3> R36;
    Eigen::Matrix<double, 3, 3> RRR;
    Eigen::Matrix<double, 3, 3> Rzyz;
    Eigen::Matrix<double, 3, 3> Rx;
    Eigen::Matrix<double, 3, 3> Ry;
    Eigen::Matrix<double, 3, 3> Rz;
	Eigen::Matrix<double, 3, 3> R03;
    Eigen::Matrix<double, 3, 3> R04;
	Eigen::Matrix<double, 3, 3> R06;
    double RR46[3][3] = {0};

   ////填充矩阵
    R03 << cos(arm.theta1) * cos(arm.theta2) * cos(arm.theta3) - cos(arm.theta1) * sin(arm.theta2) * sin(arm.theta3), -cos(arm.theta1) * cos(arm.theta2) * sin(arm.theta3) - cos(arm.theta1) * cos(arm.theta3) * sin(arm.theta2), -sin(arm.theta1),
   	 cos(arm.theta2)* cos(arm.theta3)* sin(arm.theta1) - sin(arm.theta1) * sin(arm.theta2) * sin(arm.theta3), -cos(arm.theta2) * sin(arm.theta1) * sin(arm.theta3) - cos(arm.theta3) * sin(arm.theta1) * sin(arm.theta2), cos(arm.theta1),
   	 -cos(arm.theta2) * sin(arm.theta3) - cos(arm.theta3) * sin(arm.theta2), sin(arm.theta2)* sin(arm.theta3) - cos(arm.theta2) * cos(arm.theta3), 0;

    Rx << 1, 0, 0,
   	 0, cos(yaw * pi / 180), -sin(yaw * pi / 180),
   	 0, sin(yaw * pi / 180), cos(yaw * pi / 180);
    Ry << cos(pit * pi / 180), 0, sin(pit* pi / 180),
   	 0, 1, 0,
   	 -sin(pit * pi / 180), 0, cos(pit* pi / 180);
    Rz << cos(roll * pi / 180), -sin(roll * pi / 180), 0,
   	 sin(roll* pi / 180), cos(roll* pi / 180), 0,
   	 0, 0, 1;
    R06 = Rx * Ry * Rz;
   	RRR << 1, 0, 0,
           0, 0, 1,
   	       0, -1,0; 
   R36 << R03.inverse()*  R06;
   //R46
   Rzyz = RRR.inverse()* R36;

   //将结果转换成数组
   for (int i = 0; i < 3; ++i) {
   	for (int j = 0; j < 3; ++j) {
   		RR46[i][j] = Rzyz(i, j);
   	}
   }

   // //打印数组
   //std::cout << "RR46:\n";
   //for (int i = 0; i < 3; ++i) {
   //	for (int j = 0; j < 3; ++j) {
   //		if (fabs(RR46[i][j]) < 0.000001)
   //			RR46[i][j] = 0;
   //		std::cout << RR46[i][j] << ' ';
   //	}
   //	std::cout << '\n';
   //}

   arm.theta5 = atan2(sqrt(pow(RR46[2][0], 2) + pow(RR46[2][1], 2)), RR46[2][2]);
   
   if (fabs(arm.theta5) < 0.000001)
   {
   	arm.theta5 = 0;
   }

   if (arm.theta5!=0 && fabs(arm.theta5- pi) >=0.0001)
   {
   	
   	arm.theta4 = atan2(RR46[1][2] / sin(arm.theta5), RR46[0][2] / sin(arm.theta5));
   	arm.theta6 = atan2(RR46[2][1] / sin(arm.theta5), -RR46[2][0] / sin(arm.theta5));
   }
   else if (arm.theta5 == 0)
   {
   	arm.theta4 = 0;
   	arm.theta6= atan2(-RR46[0][1], RR46[0][0]);
   }
   else if (abs(arm.theta5 - pi) <= 0.001)
   {
   	arm.theta4 = 0;
   	arm.theta6 = atan2(RR46[0][1], -RR46[0][0]);
   } 
    

   	if (fabs(arm.theta4) < 0.000001)arm.theta4 = 0;
   	if (fabs(arm.theta6) < 0.000001)arm.theta6 = 0;
   
   //arm_Fk(arm.theta1, arm.theta2, arm.theta3, arm.theta4 , arm.theta5, arm.theta6,3);
 
   std::cout << round((arm.theta4) * angle) << endl;
   std::cout << round(arm.theta5 * angle )<< endl;
   std::cout << round((arm.theta6 ) * angle) << endl;
   
	result[3] = arm.theta4 ;
	result[4] = arm.theta5 ;
	result[5] = arm.theta6 ;

	return result;
}



/*****Author 可抑
******运动学正解
******输入 xyzrpy
******输出 各个关节角度
*/
void arm_Fk(double theta1, double theta2, double theta3, double theta4, double theta5, double theta6,int mode)
{
	//                      x轴转角（输入量） x轴相差距离 z轴相差距离   z轴 相差角
	double	MDH[6][4] = {   theta1,       arm.d1,      arm.a0,       arm.alpha0,
							theta2,       arm.d2,      arm.a1,       arm.alpha1,
							theta3,       arm.d3,      arm.a2,       arm.alpha2,
							theta4,       arm.d4,      arm.a3,       arm.alpha3,
							theta5,       arm.d5,      arm.a4,       arm.alpha4,
							theta6,       arm.d6,      arm.a5,       arm.alpha5 };
	double c1 = cos(MDH[0][0]), s1 = sin(MDH[0][0]);
	double c2 = cos(MDH[1][0]), s2 = sin(MDH[1][0]);
	double c3 = cos(MDH[2][0]), s3 = sin(MDH[2][0]);
	double c4 = cos(MDH[3][0]), s4 = sin(MDH[3][0]);
	double c5 = cos(MDH[4][0]), s5 = sin(MDH[4][0]);
	double c6 = cos(MDH[5][0]), s6 = sin(MDH[5][0]);

	double ca1 = cos(MDH[0][3]), sa1 = sin(MDH[0][3]);
	double ca2 = cos(MDH[1][3]), sa2 = sin(MDH[1][3]);
	double ca3 = cos(MDH[2][3]), sa3 = sin(MDH[2][3]);
	double ca4 = cos(MDH[3][3]), sa4 = sin(MDH[3][3]);
	double ca5 = cos(MDH[4][3]), sa5 = sin(MDH[4][3]);
	double ca6 = cos(MDH[5][3]), sa6 = sin(MDH[5][3]);

	double T01[4][4] = { c1       ,        -s1    ,          0       ,         arm.a0,
						s1 * ca1  ,     c1 * ca1    ,       -sa1       ,     -sa1 * arm.d1,
						s1 * sa1  ,     c1 * sa1    ,         ca1      ,      ca1 * arm.d1,
						0        ,        0      ,          0       ,           1 };
	double T12[4][4] = { c2       ,        -s2    ,          0       ,         arm.a1,
						s2 * ca2  ,     c2 * ca2    ,       -sa2       ,     -sa2 * arm.d2,
						s2 * sa2  ,     c2 * sa2    ,         ca2      ,      ca2 * arm.d2,
						0        ,        0      ,          0       ,           1 };
	double T23[4][4] = { c3       ,        -s3    ,          0       ,         arm.a2,
						s3 * ca3  ,     c3 * ca3    ,       -sa3       ,     -sa3 * arm.d3,
						s3 * sa3  ,     c3 * sa3    ,         ca3      ,      ca3 * arm.d3,
						0        ,        0      ,          0       ,           1 };
	double T34[4][4] = { c4       ,        -s4    ,          0       ,         arm.a3,
						s4 * ca4  ,     c4 * ca4    ,       -sa4       ,     -sa4 * arm.d4,
						s4 * sa4  ,     c4 * sa4    ,         ca4      ,      ca4 * arm.d4,
						0        ,        0      ,          0       ,           1 };
	double T45[4][4] = { c5       ,        -s5    ,          0       ,         arm.a4,
						s5 * ca5  ,     c5 * ca5    ,       -sa5       ,     -sa5 * arm.d5,
						s5 * sa5  ,     c5 * sa5    ,         ca5      ,      ca5 * arm.d5,
						0        ,        0      ,          0       ,           1 };
	double T56[4][4] = { c6       ,        -s6    ,          0       ,         arm.a5,
						s6 * ca6  ,     c6 * ca6    ,       -sa6       ,     -sa6 * arm.d6,
						s6 * sa6  ,     c6 * sa6   ,         ca6      ,      ca6 * arm.d6,
						0        ,        0      ,          0       ,           1 };
	double T06[4][4];


	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			T06[i][j] = (i == j) ? 1 : 0;  // 单位矩阵
		}
	}
	// 计算矩阵C = A1 * A2 * A3 * A4 * A5 * A6

		multiply(T06, T01, T06);
		multiply(T06, T12, T06);
		multiply(T06, T23, T06);
		multiply(T06, T34, T06);
		multiply(T06, T45, T06);
		multiply(T06, T56, T06);

		// 输出矩阵C
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				if (fabs(T06[i][j])<0.0000001)
				{
					T06[i][j] = 0;
				}
				cout << T06[i][j] << "\t ";
			}
			cout << endl;
		}
		if (mode == 1)
		{
			for (int i=0;i<3;i++)
			{
				Target[i] = T06[i][3];
			}
		}
		if (mode == 2)
		{
			for (int i = 0; i < 3; i++)
			{
				Target1[i] = T06[i][3];
			}
		}

		//if (mode == 4)
		//{
		//	for (int i = 0; i < 3; ++i) {
		//		for (int j = 0; j < 3; ++j) {
		//			R06(i, j) = T06[i][j];
		//		}
		//	}
		//}

		return ;
}
