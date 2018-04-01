#include<opencv2/aruco.hpp>
#include"WorldTran.h"
#include"map.h"
#include"SerialPort.hpp"
#include<conio.h>

using namespace std;
#define devView(i) imshow(#i,i)

double mapx[10][10], mapy[10][10];

///////////////////////////cv::Mat whitei = cv::imread("imagewhite.PNG");/////路径画板

			SerialPort test1(4);
int main()
{
		MapBuild map1;
		WorldT wt1;

	void sendData(UINT16 speed, UINT16 rev, int mode);
		UINT16 Speed,revol;
	if (test1.isOpened()) cout << "usart： test1 is opened" << endl;
	else cout << "error    usart: test1 is not opened" << endl;
	cv::VideoCapture cam(0);
	if (!cam.isOpened()) return -1;
		cv::Mat frame; cam >> frame;
	////////////////////////////////////////////////相机内外参设置（图传小摄像头（鱼眼））///////////////////
	/*double camD[9] = { 2.110519307784827e+02, 0.571214297213275, 3.133118946762558e+02, 0.,
		2.935501499017382e+02, 2.380420750112324e+02, 0., 0., 1. };
	double distCoeffD[5] = { 0.155418404002410, -0.118786098296807,
		0, -0.002141275669646,
		0.001798532343113 };*/
	///////////////////////////////////////////////相机内外参设置（PC微型摄像头）////////////////////////////
	double camD[9] = { 8.185326563542064e+02, -0.720355107076837, 3.343624621743392e+02, 0.,
		8.181428694593365e+02, 2.485130982112906e+02, 0., 0., 1. };
	double distCoeffD[5] = { -0.321217652405000, 1.987975332885214,
		0, 0.002717379515449,
		0.001105867209477 };
	//////////////////////////////////////////////相机内外参设置（USBFHD01M摄像头）（70度无畸变）//////////////////////
	/*double camD[9] = { 4.607553331293777e+02, 0.222033095387230, 3.135711114758823e+02, 0.,
		4.583346386426276e+02, 2.276711646579179e+02, 0., 0., 1. };
	double distCoeffD[5] = { -0.024167696894212, 0.061533052979149,
		0, -0.002172634287795,
		0.002386705399233 };*/
	///////////////////////*************************************************************//////////////////
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64FC1, camD);
	cv::Mat distCoeffs = cv::Mat(5, 1, CV_64FC1, distCoeffD);
	auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	cv::aruco::DetectorParameters dpara;
	dpara.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

	int camk = 0;
	Sleep(1000);//////检查专用sleep
	for (cv::Mat f2; cv::waitKey(5) != 27; cam >> frame)
	{
		
		if (camk < 0)camk = 0;
		if (camk > 30)camk = 15;
		frame.copyTo(f2);
		/*if (camk > 10) camk = 0;*///////////帧率检测控制
		cv::aruco::detectMarkers(f2, dict, corners, ids);
		Speed = 2000;
		revol = 50;

		if (ids.size()==1)//ARUCO码数量为1时进行姿态测算
			{
			camk += 8;//检测稳定态控制量
			cv::aruco::drawDetectedMarkers(f2, corners, ids);			
			cv::aruco::estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs, wt1.rvecs, wt1.tvecs);
			cv::aruco::drawAxis(f2, cameraMatrix, distCoeffs, wt1.rvecs, wt1.tvecs, 0.05);
			wt1.proTran(&wt1.rvecs, &wt1.tvecs);
			//cout << " x "<<map1.idTranx(ids[0]) + 2.5*wt1.Position_OcInW.x;
				//cout << "         y "<<map1.idTrany(ids[0]) + 2.5*wt1.Position_OcInW.y << endl;
					//cout << "自旋角度" << wt1.Theta_C2W.z << endl;
								
					/*circle(whitei, Point(450-1000*(map1.idTranx(ids[0])+wt1.Position_OcInW.x), 
					450-1000* (map1.idTrany(ids[0]) + wt1.Position_OcInW.y)), 3, Scalar(0, 128, 255), -1, 8);*/
			sendData(Speed, revol, 0); 
			//if (wt1.Theta_C2W.z > 3) sendData(0, -revol, 1);
			//if (wt1.Theta_C2W.z < -3) sendData(0, revol, 1);

			cout << "数据已经发送（启动）" << endl;
			//sendData(0, revol, 1);
			}
		else 
		{ 			
			camk--;
			if (camk < 10)
			{
			Speed = 0;
			//revol = 0; 
			sendData(Speed, revol, 0);
			//sendData(0 , revol , 1); 
			cout << "数据已经发送（停止）" << endl;
			}
		}
						//camk++;////////帧率检测控制
		devView(f2);
		//devView(whitei);//路径轨迹
	}
}




void sendData(UINT16 speed,UINT16 rev, int mode)
{
	switch (mode)
	{
		//////////////////
		case 0:
		{
			union
			{
				unsigned char data[2];
				UINT16 value;
			} Speed;
			Speed.value = speed;
			unsigned char package[7]{ 0xFE,0xFE,0x05,0x01 };
			//unsigned char package[7]{ 0xFE,0xFE,0x05,0x01,0xD0,0x07,0xDD };//此行速度2000
			package[4] = Speed.data[0];
			package[5] = Speed.data[1];
			for (int i = 2; i < 6; i++)
			{
				package[6] += package[i];
			}
			test1.WriteData(package, 7);
			Sleep(10);
		}
		//////////////////
		case 1:
		{
			union
			{
				unsigned char data[2];
				UINT16 value;
			}revolve;
			revolve.value = rev;

			unsigned char package[7]{ 0xFE,0xFE,0x05,0x03 };
			package[4] = revolve.data[0];
			package[5] = revolve.data[1];
			for (int i = 2; i < 6; i++)
			{
				package[6] += package[i];
			}
			test1.WriteData(package, 7);
			Sleep(10);
		}
	}
}
