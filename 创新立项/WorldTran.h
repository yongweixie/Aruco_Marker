#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<math.h>
using namespace std;


class WorldT
{
public:
	cv::Mat RoteM, TransM;
	cv::Point3f Theta_W2C;
	cv::Point3f Theta_C2W;
	cv::Point3f Position_OwInC;
	cv::Point3f Position_OcInW;
	cv::Mat rvecs, tvecs;

public:	
	static void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
	{
		double x1 = x;//将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
		double y1 = y;
		double rz = thetaz * CV_PI / 180;
		outx = cos(rz) * x1 - sin(rz) * y1;
		outy = sin(rz) * x1 + cos(rz) * y1;
	}
	static void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz)
	{
		double x1 = x;
		double z1 = z;
		double ry = thetay * CV_PI / 180;
		outx = cos(ry) * x1 + sin(ry) * z1;
		outz = cos(ry) * z1 - sin(ry) * x1;
	}
	static void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)
	{
		double y1 = y;//将变量拷贝一次，保证&y == &y这种情况下也能计算正确
		double z1 = z;
		double rx = thetax * CV_PI / 180;
		outy = cos(rx) * y1 - sin(rx) * z1;
		outz = cos(rx) * z1 + sin(rx) * y1;
	}
	void proTran(cv::Mat* rvec, cv::Mat* tvec)
	{
		/******提取旋转矩阵******/
		double rm[9];
		RoteM = cv::Mat(3, 3, CV_64FC1, rm);
		Rodrigues(*rvec, RoteM);
		double r11 = RoteM.ptr<double>(0)[0];
		double r12 = RoteM.ptr<double>(0)[1];
		double r13 = RoteM.ptr<double>(0)[2];
		double r21 = RoteM.ptr<double>(1)[0];
		double r22 = RoteM.ptr<double>(1)[1];
		double r23 = RoteM.ptr<double>(1)[2];
		double r31 = RoteM.ptr<double>(2)[0];
		double r32 = RoteM.ptr<double>(2)[1];
		double r33 = RoteM.ptr<double>(2)[2];
		TransM = *tvec;

		//计算出相机坐标系的三轴旋转欧拉角，旋转后可以转出世界坐标系。
		//旋转顺序为z、y、x
		double thetaz = atan2(r21, r11) / CV_PI * 180;
		double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
		double thetax = atan2(r32, r33) / CV_PI * 180;
		//相机系到世界系的三轴旋转欧拉角，相机坐标系照此旋转后可以与世界坐标系完全平行。
		//旋转顺序为z、y、x
		//////THETAZ可以用来判断绕z轴旋转，角度变化，以y轴负方向为初始方向
		//////摄像头绕aruco码的z轴逆时针转动角度增加，顺时针转动角度减小
		Theta_C2W.z = thetaz;
		Theta_C2W.y = thetay;
		Theta_C2W.x = thetax;
		//计算出世界系到相机系的三轴旋转欧拉角，世界系照此旋转后可以转出相机坐标系。
		//旋转顺序为x、y、z
		Theta_W2C.x = -1 * thetax;
		Theta_W2C.y = -1 * thetay;
		Theta_W2C.z = -1 * thetaz;
		//提出平移矩阵，表示从相机坐标系原点，跟着向量(x,y,z)走，就到了世界坐标系原点
		double tx = (*tvec).ptr<double>(0)[0];
		double ty = (*tvec).ptr<double>(0)[1];
		double tz = (*tvec).ptr<double>(0)[2];
		//x y z 为唯一向量在相机原始坐标系下的向量值
		//也就是向量OcOw在相机坐标系下的值
		double x = tx, y = ty, z = tz;
		Position_OwInC.x = x;
		Position_OwInC.y = y;
		Position_OwInC.z = z;
		//进行三次反向旋转
		CodeRotateByZ(x, y, -1 * thetaz, x, y);
		CodeRotateByY(x, z, -1 * thetay, x, z);
		CodeRotateByX(y, z, -1 * thetax, y, z);

		Position_OcInW.x = x*-1;
		Position_OcInW.y = y*-1;
		Position_OcInW.z = z*-1;

	}
};