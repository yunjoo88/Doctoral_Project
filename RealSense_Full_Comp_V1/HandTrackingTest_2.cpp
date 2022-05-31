//#define WIN32_LEAN_AND_MEAN
//#define UNICODE
//#ifdef _MSC_VER
//#define _CRT_SECURE_NO_WARNINGS
//#endif
//
//#include "KinovaTypes.h"
//#include <Windows.h>
//#include <objbase.h>
//#include <WS2tcpip.h> //function lib for win socket networks
//#include "CommunicationLayerWindows.h"
//#include "CommandLayer.h"
//#include <conio.h>
//#include <cstring>
//#include <iostream>
//
//#include "NuitrackGLSample.h"
//#include <GL/glut.h>
//#include <iomanip>
//
//#include <royale.hpp>
//#include <mutex>
//#include <opencv2/opencv.hpp>
//#include <sample_utils/PlatformResources.hpp>
//
//// Dynamixel
//#include "dynamixel2.h"
//#include "FunctionDefine.h"
//
//
//#include <string.h>
//#include <stdio.h>
//
//#include<boost/thread.hpp>
//#include<boost/atomic.hpp>
//#pragma comment(lib, "dynamixel2_win32.lib")
//
//#ifdef _DEBUG
//#pragma comment (lib, "ws2_32.lib")
//#endif
//
//#define MAX_IN_CHAR 128
//
//// OpenCV 4.1.0
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/imgproc.hpp>
//
//#include <fstream>
//
//using namespace std;
//
//HINSTANCE  commandLayer_handle;
//NuitrackGLSample sample;
//
////Function pointers to the functions we need
//int(*MyInitAPI)();
//int(*MyCloseAPI)();
//int(*MySendBasicTrajectory)(TrajectoryPoint command);
//int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
//int(*MySetActiveDevice)(KinovaDevice device);
//int(*MyMoveHome)();
//int(*MyInitFingers)();
//int(*MyGetCartesianCommand)(CartesianPosition &);
//int(*MyGetAngularCurrent)(AngularPosition &);
//int(*MyEraseAllTrajectories)();
//int(*MyStartForceControl)();
//int(*MyStopForceControl)();
//
//
//// State machine
//enum G_Mode {
//	Wait,
//	Direct,
//	HandTracking,
//	Grasping
//};
//G_Mode G_Mode_S = Wait;
//
////global variables
//int NumRecord = 200;
//int programResult = 0;
//float* RHandPos;
//float* LHandPos;
//float* HeadPos;
//int NumofBodies = 0;
//int FirstDetect = 0;
//int state = 0;
//int Right_Hand_Grip = -1;
//int Left_Hand_Grip = -1;
//int Bodyflag = 0;
//float rob_pos[3];
//float Dtogoal = 1000;
//float GUrep_bnd[] = { 0,0,0 };
//float GUrep_obs[] = { 0,0,0 };
//float GUrep[] = { 0,0,0 };
//float GUatt[3];
//float gradient[3];
//float D;
//float DtoCenter;
//float Cons;
//float norm_gradient;
//int numloop = 0;
//float bnd[2][3] = { { -0.3,-0.7,-0.1 },{ 0.4,-0.3,0.7 } };	//boundary
//float bnd_center[] = { 0.5*(bnd[1][1] + bnd[2][1]),0.5*(bnd[1][2] + bnd[2][2]),0.5*(bnd[1][3] + bnd[2][3]) };
//
//double norm_momentum = 0.0;
//int imagetype = -1;
//int obj_id = 1000;
//float Down_Z = 1000;
//cv::Vec3f d_XYZt_Kinova = { 0,0,0 };
//atomic<bool> flag = true;
//
////test case 3 - w/ momentum, 2 balls
//float Kappa = 0.4f;
//float Nu = 1.0e-6f;
//float rate = 0.9f;
//float ObsTh = 0.05f;
//float temp[3] = { 0.0f,0.0f,0.0f };
//float goal[3] = { 0.02f,-0.6f,0.3f };
//float momentum[] = { 0.0f,0.0f,0.0f };
//const int obsnum = 2;
//float obs[obsnum][4] = { { 0.373f,-0.44f,0.190f,0.12f },{ 0.163f,-0.587f,0.100f,0.12f } }; // x,y,z position + R radius
//float stepsize = 0.03f;
//
//struct TimeData {
//	int state = 0;
//	float HumanHandPos[3] = { 0.0f };
//	float RobotHandPos[2] = { 0.0f };
//	float RobotHandCurrent[2] = { 0.0f };
//	float RobotArmPos[6] = { 0.0f };
//	float RobotArmTorque[7] = { 0.0f };
//	float TactileData[8] = { 0.0f }; // Finger 1: 13 14 15 16, Finger 2: 13 14 15 16
//};
//
//struct TactileData {
//	int state = 0;
//	float Tactile[32] = { 0.0f };
//};
//
//struct GestData {
//	int gest = 0;
//	float GestSensor[7] = { 0.0f };
//};
//TimeData LogData[200];
//TactileData tactdata;
//GestData gestdata;
//
//std::ofstream LogFile;
//
//#define PI 3.141592
//
//// UDP Threads
//void UDPthread_1(atomic<bool>& flag) {
//	// 1.3) UDP Communication
//	// Startup Winsock
//	WSADATA data;
//	WORD version = MAKEWORD(2, 2);
//	int wsOk = WSAStartup(version, &data);
//	if (wsOk != 0)
//	{
//		std::cout << "Can't start Winsock!" << wsOk;
//	}
//	std::cout << "start winsock" << std::endl;
//	SOCKET in = socket(AF_INET, SOCK_DGRAM, 0); // in 은 그냥 interger
//												// UDP는 packet이 도착하는게 보장되지 않음. 제어할 수 없음. 
//												// TCP_IP는 5개 도착할때까지 기다림. 안도착한 packet을 재요청 및 기다림.
//												/*struct timeval optVal = { 10, 0 };
//												int optLen = sizeof(optVal);
//												setsockopt(in, SOL_SOCKET, SO_RCVTIMEO, (char*)&optVal, optLen);*/
//	DWORD recvTO = 5000;
//	setsockopt(in, SOL_SOCKET, SO_RCVTIMEO, (char*)&recvTO, sizeof(recvTO));
//	sockaddr_in serverHint;
//	serverHint.sin_addr.S_un.S_addr = ADDR_ANY; // give me any address, whatever address give that to me
//	serverHint.sin_family = AF_INET;
//	serverHint.sin_port = htons(58430); // Conver from little to big endian //htons:host to network short
//
//	sockaddr_in client;
//	int clientLength = sizeof(client);
//	ZeroMemory(&client, clientLength);
//
//	// 2.3) UDP Communication
//	// Bind sokcet to ip address and port
//	if (bind(static_cast<SOCKET>(in), static_cast<const sockaddr*>((sockaddr*)&serverHint), static_cast<int>(sizeof(serverHint))) == SOCKET_ERROR) // Socket - IP - Port (Triple connection binding)
//	{
//		std::cout << "Can't bind socket! " << WSAGetLastError() << std::endl;
//	}
//
//	int Tacloop = 0;
//	char *Tacloop_S;
//	char buf[1024];	// message from client saved to buf
//
//	while (flag) {
//		//cout << "Thread loop ... " << flag << endl;
//		/*char sendbuf[100] = "Server send\n";
//		int bytesIn = sendto(in, sendbuf, strlen(sendbuf), 0, (struct sockaddr*) &client, sizeof(client));
//		if(bytesIn == SOCKET_ERROR) {
//		cout << "Error sending to client " << WSAGetLastError() << endl;
//		continue;
//		}*/
//		ZeroMemory(buf, 1024);
//		// Wait for message
//		int bytesIn = recvfrom(in, buf, 1024, 0, (sockaddr*)&client, &clientLength); // recv 는 TCP용
//		if (bytesIn == SOCKET_ERROR)
//		{
//			std::cout << "Error receiving from client " << WSAGetLastError() << std::endl;
//			continue;
//		}
//		// Display message and client info
//		char clientIp[256];
//		ZeroMemory(clientIp, 256);
//
//		inet_ntop(AF_INET, &client.sin_addr, clientIp, 256); //version 4 IP address type is AF_INET
//
//															 //cout << "Message recv from " << clientIp << " : " << buf << endl;
//		char * pos;
//		char * context;
//
//		//printf("원본: %s\n", buf);
//		//strtok_s 함수 이용
//		//printf("== 공백이나 콤마, 느낌표, 마침표를 기준으로 분할 ==\n");
//		pos = strtok_s(buf, "Arduino Out:,", &context);  //처음 호출 시에 대상 문자열 전달
//		int cnt = 0;
//		while (pos != NULL)
//		{
//			if (pos != NULL) {
//				//printf("%f, ", atof(pos));
//				gestdata.GestSensor[cnt++] = atof(pos);
//			}
//			pos = strtok_s(context, "Arduino Out:,", &context);//이 후 NULL 혹은 context 전달
//
//		}
//		//printf("\n Copy Done");
//	}
//	std::cout << "UDP Done" << std::endl;
//	closesocket(in);
//	// Shutdown Winsock
//	WSACleanup();
//	return;
//}
//
//void UDPthread_2(atomic<bool>& flag) {
//	// 1.3) UDP Communication
//	// Startup Winsock
//	WSADATA data;
//	WORD version = MAKEWORD(2, 2);
//	int wsOk = WSAStartup(version, &data);
//	if (wsOk != 0)
//	{
//		std::cout << "Can't start Winsock!" << wsOk;
//	}
//	std::cout << "start winsock" << std::endl;
//	SOCKET in = socket(AF_INET, SOCK_DGRAM, 0); // in 은 그냥 interger
//												// UDP는 packet이 도착하는게 보장되지 않음. 제어할 수 없음. 
//												// TCP_IP는 5개 도착할때까지 기다림. 안도착한 packet을 재요청 및 기다림.
//												/*struct timeval optVal = { 10, 0 };
//												int optLen = sizeof(optVal);
//												setsockopt(in, SOL_SOCKET, SO_RCVTIMEO, (char*)&optVal, optLen);*/
//	DWORD recvTO = 5000;
//	setsockopt(in, SOL_SOCKET, SO_RCVTIMEO, (char*)&recvTO, sizeof(recvTO));
//	sockaddr_in serverHint;
//	serverHint.sin_addr.S_un.S_addr = ADDR_ANY; // give me any address, whatever address give that to me
//	serverHint.sin_family = AF_INET;
//	serverHint.sin_port = htons(58432); // Conver from little to big endian //htons:host to network short
//
//	sockaddr_in client;
//	int clientLength = sizeof(client);
//	ZeroMemory(&client, clientLength);
//
//	// 2.3) UDP Communication
//	// Bind sokcet to ip address and port
//	if (bind(static_cast<SOCKET>(in), static_cast<const sockaddr*>((sockaddr*)&serverHint), static_cast<int>(sizeof(serverHint))) == SOCKET_ERROR) // Socket - IP - Port (Triple connection binding)
//	{
//		std::cout << "Can't bind socket! " << WSAGetLastError() << std::endl;
//	}
//
//	int Tacloop = 0;
//	char *Tacloop_S;
//	char buf[1024];	// message from client saved to buf
//
//	while (flag) {
//		//cout << "Thread loop ... " << flag << endl;
//		/*char sendbuf[100] = "Server send\n";
//		int bytesIn = sendto(in, sendbuf, strlen(sendbuf), 0, (struct sockaddr*) &client, sizeof(client));
//		if(bytesIn == SOCKET_ERROR) {
//		cout << "Error sending to client " << WSAGetLastError() << endl;
//		continue;
//		}*/
//		ZeroMemory(buf, 1024);
//		// Wait for message
//		int bytesIn = recvfrom(in, buf, 1024, 0, (sockaddr*)&client, &clientLength); // recv 는 TCP용
//		if (bytesIn == SOCKET_ERROR)
//		{
//			std::cout << "Error receiving from client " << WSAGetLastError() << std::endl;
//			continue;
//		}
//		// Display message and client info
//		char clientIp[256];
//		ZeroMemory(clientIp, 256);
//
//		inet_ntop(AF_INET, &client.sin_addr, clientIp, 256); //version 4 IP address type is AF_INET
//
//															 //cout << "Message recv from " << clientIp << " : " << buf << endl;
//		char * pos;
//		char * context;
//
//		//printf("원본: %s\n", buf);
//		//strtok_s 함수 이용
//		//printf("== 공백이나 콤마, 느낌표, 마침표를 기준으로 분할 ==\n");
//		pos = strtok_s(buf, "SBF():,", &context);  //처음 호출 시에 대상 문자열 전달
//		Tacloop = atoi(pos);
//		//printf("%d : ", Tacloop);
//		int cnt = 0;
//		while (pos != NULL)
//		{
//			pos = strtok_s(context, "SBF():,", &context);//이 후 NULL 혹은 context 전달
//			if (pos != NULL) {
//				//printf("%f", atof(pos));
//				tactdata.Tactile[cnt++] = atof(pos);
//			}
//		}
//		//printf("\n Copy Done");
//	}
//	std::cout << "UDP Done" << std::endl;
//	closesocket(in);
//	// Shutdown Winsock
//	WSACleanup();
//	return;
//}
//
//// Pico Flexx
//class MyListener : public royale::IDepthDataListener
//{
//
//public:
//
//	MyListener() : undistortImage(false)
//	{
//	}
//
//
//	void onNewData(const royale::DepthData *data)
//	{
//		// this callback function will be called for every new depth frame
//
//		std::lock_guard<std::mutex> lock(flagMutex);
//
//		float *zRowPtr, *grayRowPtr = NULL;
//		// zImage
//		zImage.create(cv::Size(data->width, data->height), CV_32FC1);
//		zImage = cv::Scalar::all(0); // set the image to zero
//
//		int k = 0;
//		for (int y = 0; y < zImage.rows; y++)
//		{
//			zRowPtr = zImage.ptr<float>(y);
//
//			for (int x = 0; x < zImage.cols; x++, k++)
//			{
//				auto curPoint = data->points.at(k);
//				if (curPoint.depthConfidence > 0)
//				{
//					// if the point is valid, map the pixel from 3D world
//					// coordinates to a 2D plane (this will distort the image)					
//					zRowPtr[x] = adjustZValue(curPoint.z);
//				}
//				else {
//					zRowPtr[x] = 255; //distortion part => black
//				}
//			}
//		}
//		zImage8.create(cv::Size(data->width, data->height), CV_8UC1);
//		zImage.convertTo(zImage8, CV_8UC1);		// normalize(zImage, zImage8, 0, 255, NORM_MINMAX, CV_8UC1)
//
//		if (undistortImage)
//		{
//			// call the undistortion function on the z image
//			cv::Mat temp = zImage8.clone();
//			undistort(temp, zImage8, cameraMatrix, distortionCoefficients);
//		}
//
//		scaledZImage.create(cv::Size(data->width * 4, data->height * 4), CV_8UC1);		// scale and display the depth image
//		cv::resize(zImage8, scaledZImage, scaledZImage.size());
//
//		//cv::imshow("Depth", scaledZImage);
//
//		// grayImage
//		grayImage.create(cv::Size(data->width, data->height), CV_32FC1);
//		grayImage = cv::Scalar::all(0); // set the image to zero
//
//		k = 0;
//		for (int y = 0; y < grayImage.rows; y++)
//		{
//			grayRowPtr = grayImage.ptr<float>(y);
//
//			for (int x = 0; x < grayImage.cols; x++, k++)
//			{
//				auto curPoint = data->points.at(k);
//				if (curPoint.depthConfidence > 0)
//				{
//					// if the point is valid, map the pixel from 3D world
//					// coordinates to a 2D plane (this will distort the image)
//					grayRowPtr[x] = adjustGrayValue(curPoint.grayValue);
//				}
//				else {
//					grayRowPtr[x] = 255; //distortion part => black
//				}
//			}
//		}
//
//		grayImage8.create(cv::Size(data->width, data->height), CV_8UC1);
//		grayImage.convertTo(grayImage8, CV_8UC1);		// normalize(grayImage, grayImage8, 0, 255, NORM_MINMAX, CV_8UC1)
//
//		if (undistortImage)
//		{
//			// call the undistortion function on the gray image
//			cv::Mat temp = grayImage8.clone();
//			undistort(temp, grayImage8, cameraMatrix, distortionCoefficients);
//		}
//
//		// scale and display the gray image
//		scaledGrayImage.create(cv::Size(data->width * 4, data->height * 4), CV_8UC1);
//		cv::resize(grayImage8, scaledGrayImage, scaledGrayImage.size());
//
//		//cv::imshow("Gray", scaledGrayImage);
//
//		cv::Vec4f pOutLine;
//
//		imagetype = 0;
//
//		result_ZImage.create(cv::Size(data->width * 4, data->height * 4), CV_8UC1);
//		overlay_Bounding_Box(scaledZImage, data, result_ZImage, &pOutLine);
//		Kinova_calib(&pOutLine, &d_XYZt_Kinova);
//		cv::imshow("Test_Z", result_ZImage);
//
//		//imagetype = 1;		
//		//result_GImage.create(Size(data->width * 4, data->height * 4), CV_8UC1);
//		//overlay_Bounding_Box(scaledGrayImage, data, result_GImage, &pOutLine);// read from gray image
//		//Kinova_calib(&pOutLine, &d_XYZt_Kinova);
//		//imshow("Test_G", result_GImage);
//	}
//
//	void Kinova_calib(cv::Vec4f* OutLine, cv::Vec3f *d_XYZt_Kinova) {
//		// Kinova Gripper Initial : (Z=0, theta=0) ===>> pico : (X =466/960, Y=125/720), real_world : (0.055 m, 0.02 m)
//		float dx = ((*OutLine)[2] * 0.0322 - 15)*0.01; //466
//		float dy = ((*OutLine)[3] * -0.032 + 4)*0.01; //125
//		float dth = atan((float)(*OutLine)[1] / (float)(*OutLine)[0]) - PI / 2.0f + 0.2f;
//
//		*d_XYZt_Kinova = { dx, dy, dth };
//
//		/*	cout << " Out : " << dx << ", " << dy << ", " << dth << endl;
//		cout << " OutLine : " << *OutLine << endl;
//		cout << " d_Kinova : " << *d_XYZt_Kinova << endl;*/
//	}
//
//
//	void overlay_Bounding_Box(cv::Mat tImage, const royale::DepthData *data, cv::Mat result_tImage, cv::Vec4f *pOutLine) {
//
//		normalize(tImage, tImage8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
//
//		if (imagetype == 0) { // Z image
//			cv::threshold(~tImage8, tImage8, 150, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);// +cv::THRESH_BINARY); cv::THRESH_OTSU
//																							//Adaptive Thresholding을 한다.
//																							//adaptiveThreshold(tImage8, tImage8, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, 10);
//		}
//		else if (imagetype == 1) { // Gray image
//			cv::threshold(tImage8, tImage8, 127, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);// +cv::THRESH_BINARY);
//		}
//		cv::GaussianBlur(tImage8, tImage8, cv::Size(5, 5), 1, 1, 1);
//		cv::Mat tImageMat = tImage8.clone();
//
//
//		double area, max_area = 0;
//		double min_err = 10000;
//		obj_id = 0;
//		cv::Rect bounding_rect;
//		vector<cv::Vec4i> hierarchy;
//		vector<vector<cv::Point> > contours;
//		findContours(tImageMat, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
//		for (int i = 0; i < contours.size(); i++) {
//			/* // Maximum Area Detection
//			if (area > max_area) {
//			max_area = area;
//			max_id = i;
//			bounding_rect = boundingRect(contours[i]);
//			}*/
//			area = contourArea(contours[i], false);
//			if (area > 5000 & area < 80000) {
//				//cout << "Area :" << area << endl;
//				cv::Moments M = cv::moments(contours[i]);
//				int cX = int(M.m10 / M.m00);
//				int cY = int(M.m01 / M.m00);
//				// cout << "CX" << cX << "CY" << cY << endl << endl;
//				double d_err = sqrt((cX - 466) ^ 2 + (cY - 125) ^ 2);
//				if (d_err < min_err) {
//					min_err = d_err;
//					obj_id = i;
//					bounding_rect = boundingRect(contours[i]);
//				}
//			}
//
//		}
//
//		rectangle(tImage8, bounding_rect, cv::Scalar(100, 100, 100), 1, 0);
//
//		cv::Vec4f OutLine;
//		fitLine(cv::Mat(contours[obj_id]), OutLine, cv::DIST_L2, 0, 0.01, 0.01);
//
//		//cout << "BR :" << bounding_rect.x << "  " << bounding_rect.width << endl;
//		//cout << "zImage.rows :" << zImage.rows << "zImage.cols :" << zImage.cols << endl;
//
//		// Find Down Z value
//		int k = 0;
//		float min_z = 1000;
//		for (int y = 0; y < zImage.rows; y++)
//		{
//			for (int x = 0; x < zImage.cols; x++, k++)
//			{
//				auto curPoint = data->points.at(k);
//				if (((bounding_rect.x / 4) < (k % zImage.rows)) & (((bounding_rect.x / 4) + (bounding_rect.width / 4) - 1) >(k % zImage.rows))) {
//					if (min_z > curPoint.z & curPoint.z > 0) {
//						min_z = curPoint.z;
//					}
//				}
//			}
//		}
//		if (min_z < 1) {
//			Down_Z = min_z;
//		}
//		//cout << " Z :" << Down_Z << endl;
//
//
//		/****************/
//		//cout << "OutLine Data" << OutLine << endl;
//		cv::line(tImage8, cv::Point(OutLine[2], OutLine[3]), cv::Point(100 * OutLine[0] + OutLine[2], 100 * OutLine[1] + OutLine[3]), cv::Scalar(50, 50, 50), 2);
//		//line(tImage, Point(0,0), Point(200,200), Scalar(50, 50, 50), 2);
//		cv::circle(tImage8, cv::Point(OutLine[2], OutLine[3]), 4, cv::Scalar(0, 255, 0), 2);
//		cv::circle(tImage8, cv::Point(20 * OutLine[0] + OutLine[2], 20 * OutLine[1] + OutLine[3]), 4, cv::Scalar(0, 255, 0), 2);
//
//		//Sleep(1);		
//		/*scaledtImage.create(Size(data->width * 4, data->height * 4), CV_8UC1);
//		resize(tImage8, scaledtImage, scaledtImage.size());
//		imshow("Test", scaledtImage);*/
//		*pOutLine = OutLine;
//
//		resize(tImage8, result_tImage, result_tImage.size());
//	}
//
//	void setLensParameters(const royale::LensParameters &lensParameters)
//	{
//		// Construct the camera matrix
//		// (fx   0    cx)
//		// (0    fy   cy)
//		// (0    0    1 )
//		cameraMatrix = (cv::Mat1d(3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
//			0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
//			0, 0, 1);
//
//		// Construct the distortion coefficients
//		// k1 k2 p1 p2 k3
//		distortionCoefficients = (cv::Mat1d(1, 5) << lensParameters.distortionRadial[0],
//			lensParameters.distortionRadial[1],
//			lensParameters.distortionTangential.first,
//			lensParameters.distortionTangential.second,
//			lensParameters.distortionRadial[2]);
//	}
//
//	void toggleUndistort()
//	{
//		std::lock_guard<std::mutex> lock(flagMutex);
//		undistortImage = !undistortImage;
//	}
//
//private:
//
//	// adjust z value to fit fixed scaling, here max dist is 2.5m
//	// the max dist here is used as an example and can be modified
//	float adjustZValue(float zValue)
//	{
//		float clampedDist = std::min(0.44f, zValue);
//		float newZValue = clampedDist / 0.44f * 255.0f;
//		return newZValue;
//	}
//
//	// adjust gray value to fit fixed scaling, here max value is 180
//	// the max value here is used as an example and can be modified
//	float adjustGrayValue(uint16_t grayValue)
//	{
//		float clampedVal = std::min(2350.0f, grayValue * 1.0f); // 180.0f (Original clamp Value)
//		float newGrayValue = clampedVal / 2350.f * 255.0f;
//		return newGrayValue;
//	}
//
//	// define images for depth and gray
//	// and for their 8Bit and scaled versions
//	cv::Mat zImage, zImage8, scaledZImage, result_ZImage;
//	cv::Mat grayImage, grayImage8, scaledGrayImage, result_GImage;
//
//	// 2018Oct27 YJ
//	cv::Mat tImage, tImage8, scaledtImage; //Test image
//
//										   // lens matrices used for the undistortion of
//										   // the image
//	cv::Mat cameraMatrix;
//	cv::Mat distortionCoefficients;
//
//	std::mutex flagMutex;
//	bool undistortImage;
//};
//
//// Keyboard handler
//void keyboard(unsigned char key, int x, int y)
//{
//	switch (key)
//	{
//		// On Esc key press
//	case 27:
//	{
//		sample.release();
//		int result = (*MyCloseAPI)();
//		FreeLibrary(commandLayer_handle);
//
//		glutDestroyWindow(glutGetWindow());
//		exit(EXIT_FAILURE);
//	}
//
//	default:
//	{
//		// Do nothing otherwise
//		break;
//	}
//	}
//}
//
//void mouseClick(int button, int state, int x, int y)
//{
//	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
//	{
//		sample.nextViewMode();
//	}
//}
//
//
//// Update tracking data and visualize it
//void display()
//{
//	// Delegate this action to example's main class
//	bool update = sample.update();
//
//	if (!update)
//	{
//		// End the work if update failed
//		sample.release();
//		int result = (*MyCloseAPI)();
//		FreeLibrary(commandLayer_handle);
//		glutDestroyWindow(glutGetWindow());
//		exit(EXIT_FAILURE);
//	}
//
//	// Do flush and swap buffers to update viewport
//	glFlush();
//	glutSwapBuffers();
//}
//
////void record_Data(int ID, int state, float* HumanHand, float* RobotHand, float* RobotArm, float* RobotHandCurrent, float* RobotArmTorque) {
////	float ArmPos[6] = { currentPosition.Coordinates.X, currentPosition.Coordinates.Y,currentPosition.Coordinates.Z,
////		currentPosition.Coordinates.ThetaX ,currentPosition.Coordinates.ThetaY ,currentPosition.Coordinates.ThetaZ };
////
////	AngularPosition AngularCurrent;
////	AngularCurrent.InitStruct();
////	MyGetAngularCurrent(AngularCurrent);
////	float ArmCur[7] = { AngularCurrent.Actuators.Actuator1,	AngularCurrent.Actuators.Actuator2,	AngularCurrent.Actuators.Actuator3,
////		AngularCurrent.Actuators.Actuator4,	AngularCurrent.Actuators.Actuator5,	AngularCurrent.Actuators.Actuator6,	AngularCurrent.Actuators.Actuator7 };
////
////	float HandCur[2] = { 0.0f };
////	float tmpval = Read2(1, 126, 2, 1);
////	if (tmpval > 2048) {
////		tmpval = tmpval - 65535;
////	}
////	HandCur[0] = tmpval;
////	tmpval = Read2(2, 126, 2, 1);
////	if (tmpval > 2048) {
////		tmpval = tmpval - 65535;
////	}
////	HandCur[1] = tmpval;
////
////	float HandPos[2] = { Read2(1, 132, 4, 1), Read2(2, 132, 4, 1) };
////
////	LogData[ID].HumanHandPos[0] = *(HumanHand);	LogData[ID].HumanHandPos[1] = *(HumanHand + 1);	LogData[ID].HumanHandPos[2] = *(HumanHand + 2);
////	LogData[ID].RobotHandPos[0] = *(RobotHand);	LogData[ID].RobotHandPos[1] = *(RobotHand + 1);
////	LogData[ID].RobotHandCurrent[0] = *(RobotHandCurrent); LogData[ID].RobotHandCurrent[1] = *(RobotHandCurrent + 1);
////	LogData[ID].RobotArmPos[0] = *(RobotArm); LogData[ID].RobotArmPos[1] = *(RobotArm + 1); LogData[ID].RobotArmPos[2] = *(RobotArm + 2);
////	LogData[ID].RobotArmPos[3] = *(RobotArm + 3); LogData[ID].RobotArmPos[4] = *(RobotArm + 4);	LogData[ID].RobotArmPos[5] = *(RobotArm + 5);
////	LogData[ID].RobotArmTorque[0] = *(RobotArmTorque); LogData[ID].RobotArmTorque[1] = *(RobotArmTorque + 1);
////	LogData[ID].RobotArmTorque[2] = *(RobotArmTorque + 2); LogData[ID].RobotArmTorque[3] = *(RobotArmTorque + 3);
////	LogData[ID].RobotArmTorque[4] = *(RobotArmTorque + 4); LogData[ID].RobotArmTorque[5] = *(RobotArmTorque + 5);
////	LogData[ID].RobotArmTorque[6] = *(RobotArmTorque + 6);
////	LogData[ID].TactileData[0] = tactdata.Tactile[12]; LogData[ID].TactileData[1] = tactdata.Tactile[13];
////	LogData[ID].TactileData[2] = tactdata.Tactile[14]; LogData[ID].TactileData[3] = tactdata.Tactile[15];
////	LogData[ID].TactileData[4] = tactdata.Tactile[28]; LogData[ID].TactileData[5] = tactdata.Tactile[29];
////	LogData[ID].TactileData[6] = tactdata.Tactile[30]; LogData[ID].TactileData[7] = tactdata.Tactile[31];
////	//출력 byte로 나오는거 체크
////	std::cout << "Saved Data in ID " << ID << std::endl;
////	std::cout << "HumanHandPos : " << LogData[ID].HumanHandPos[0] << ", " << LogData[ID].HumanHandPos[1] << ", " << LogData[ID].HumanHandPos[2] << std::endl;
////	std::cout << "RobotHandPos : " << LogData[ID].RobotHandPos[0] << ", " << LogData[ID].RobotHandPos[1] << std::endl;
////	std::cout << "RobotHandCurrnet : " << LogData[ID].RobotHandCurrent[0] << ", " << LogData[ID].RobotHandCurrent[1] << std::endl;
////	std::cout << "RobotAmrPos : " << LogData[ID].RobotArmPos[0] << ", " << LogData[ID].RobotArmPos[1] << ", "
////		<< LogData[ID].RobotArmPos[2] << ", " << LogData[ID].RobotArmPos[3] << ", "
////		<< LogData[ID].RobotArmPos[4] << ", " << LogData[ID].RobotArmPos[5] << std::endl;
////	std::cout << "RobotArmTorque : " << LogData[ID].RobotArmTorque[0] << ", " << LogData[ID].RobotArmTorque[1] << ", "
////		<< LogData[ID].RobotArmTorque[2] << ", " << LogData[ID].RobotArmTorque[3] << ", " << LogData[ID].RobotArmTorque[4] << ", "
////		<< LogData[ID].RobotArmTorque[5] << ", " << LogData[ID].RobotArmTorque[6] << std::endl;
////	std::cout << "TactileData : " << LogData[ID].TactileData[0] << ", " << LogData[ID].TactileData[1] << ", "
////		<< LogData[ID].TactileData[2] << ", " << LogData[ID].TactileData[3] << ", "
////		<< LogData[ID].TactileData[4] << ", " << LogData[ID].TactileData[5] << ", "
////		<< LogData[ID].TactileData[6] << ", " << LogData[ID].TactileData[7] << std::endl << std::endl;
////
////
////}
//
//void record_Data(atomic<bool>& flag) {
//	Sleep(10000);
//
//	while (flag) {
//		int ID = numloop++;
//		CartesianPosition recordPosition;
//		recordPosition.InitStruct();
//		MyGetCartesianCommand(recordPosition);
//		float ArmPos[6] = { recordPosition.Coordinates.X, recordPosition.Coordinates.Y,recordPosition.Coordinates.Z,
//			recordPosition.Coordinates.ThetaX , recordPosition.Coordinates.ThetaY , recordPosition.Coordinates.ThetaZ };
//
//		AngularPosition AngularCurrent;
//		AngularCurrent.InitStruct();
//		MyGetAngularCurrent(AngularCurrent);
//		float ArmCur[7] = { AngularCurrent.Actuators.Actuator1,	AngularCurrent.Actuators.Actuator2,	AngularCurrent.Actuators.Actuator3,
//			AngularCurrent.Actuators.Actuator4,	AngularCurrent.Actuators.Actuator5,	AngularCurrent.Actuators.Actuator6,	AngularCurrent.Actuators.Actuator7 };
//
//		float HandCur[2] = { 0.0f };
//		float tmpval = Read2(1, 126, 2, 1);
//		if (tmpval > 2048) {
//			tmpval = tmpval - 65535;
//		}
//		HandCur[0] = tmpval;
//		tmpval = Read2(2, 126, 2, 1);
//		if (tmpval > 2048) {
//			tmpval = tmpval - 65535;
//		}
//		HandCur[1] = tmpval;
//
//		float HandPos[2] = { Read2(1, 132, 4, 1), Read2(2, 132, 4, 1) };
//
//		LogData[ID].HumanHandPos[0] = goal[0];	LogData[ID].HumanHandPos[1] = goal[1];	LogData[ID].HumanHandPos[2] = goal[2];
//		LogData[ID].RobotHandPos[0] = HandPos[0];	LogData[ID].RobotHandPos[1] = HandPos[1];
//		LogData[ID].RobotHandCurrent[0] = HandCur[0]; LogData[ID].RobotHandCurrent[1] = HandCur[1];
//		LogData[ID].RobotArmPos[0] = ArmPos[0]; LogData[ID].RobotArmPos[1] = ArmPos[1]; LogData[ID].RobotArmPos[2] = ArmPos[2];
//		LogData[ID].RobotArmPos[3] = ArmPos[3]; LogData[ID].RobotArmPos[4] = ArmPos[4];	LogData[ID].RobotArmPos[5] = ArmPos[5];
//		LogData[ID].RobotArmTorque[0] = ArmCur[0]; LogData[ID].RobotArmTorque[1] = ArmCur[1];
//		LogData[ID].RobotArmTorque[2] = ArmCur[2]; LogData[ID].RobotArmTorque[3] = ArmCur[3];
//		LogData[ID].RobotArmTorque[4] = ArmCur[4]; LogData[ID].RobotArmTorque[5] = ArmCur[5];
//		LogData[ID].RobotArmTorque[6] = ArmCur[6];
//		LogData[ID].TactileData[0] = tactdata.Tactile[12]; LogData[ID].TactileData[1] = tactdata.Tactile[13];
//		LogData[ID].TactileData[2] = tactdata.Tactile[14]; LogData[ID].TactileData[3] = tactdata.Tactile[15];
//		LogData[ID].TactileData[4] = tactdata.Tactile[28]; LogData[ID].TactileData[5] = tactdata.Tactile[29];
//		LogData[ID].TactileData[6] = tactdata.Tactile[30]; LogData[ID].TactileData[7] = tactdata.Tactile[31];
//		//출력 byte로 나오는거 체크
//		std::cout << "Saved Data in ID " << ID << std::endl;
//		/*std::cout << "HumanHandPos : " << LogData[ID].HumanHandPos[0] << ", " << LogData[ID].HumanHandPos[1] << ", " << LogData[ID].HumanHandPos[2] << std::endl;
//		std::cout << "RobotHandPos : " << LogData[ID].RobotHandPos[0] << ", " << LogData[ID].RobotHandPos[1] << std::endl;
//		std::cout << "RobotHandCurrnet : " << LogData[ID].RobotHandCurrent[0] << ", " << LogData[ID].RobotHandCurrent[1] << std::endl;
//		std::cout << "RobotAmrPos : " << LogData[ID].RobotArmPos[0] << ", " << LogData[ID].RobotArmPos[1] << ", "
//		<< LogData[ID].RobotArmPos[2] << ", " << LogData[ID].RobotArmPos[3] << ", "
//		<< LogData[ID].RobotArmPos[4] << ", " << LogData[ID].RobotArmPos[5] << std::endl;
//		std::cout << "RobotArmTorque : " << LogData[ID].RobotArmTorque[0] << ", " << LogData[ID].RobotArmTorque[1] << ", "
//		<< LogData[ID].RobotArmTorque[2] << ", " << LogData[ID].RobotArmTorque[3] << ", " << LogData[ID].RobotArmTorque[4] << ", "
//		<< LogData[ID].RobotArmTorque[5] << ", " << LogData[ID].RobotArmTorque[6] << std::endl;
//		std::cout << "TactileData : " << LogData[ID].TactileData[0] << ", " << LogData[ID].TactileData[1] << ", "
//		<< LogData[ID].TactileData[2] << ", " << LogData[ID].TactileData[3] << ", "
//		<< LogData[ID].TactileData[4] << ", " << LogData[ID].TactileData[5] << ", "
//		<< LogData[ID].TactileData[6] << ", " << LogData[ID].TactileData[7] << std::endl << std::endl;
//		*/
//		if (LogFile.is_open()) {
//			cout << "Writing Log Data... " << endl;
//			LogFile << "Saved Data in ID " << ID << "\n";
//			LogFile << "HumanHandPos : " << LogData[ID].HumanHandPos[0] << ", " << LogData[ID].HumanHandPos[1] << ", " << LogData[ID].HumanHandPos[2] << "\n";
//			LogFile << "Gradinet : " << momentum[0] << ", " << momentum[1] << ", " << momentum[2] << "\n";
//			LogFile << "RobotHandPos : " << LogData[ID].RobotHandPos[0] << ", " << LogData[ID].RobotHandPos[1] << "\n";
//			LogFile << "RobotHandCurrnet : " << LogData[ID].RobotHandCurrent[0] << ", " << LogData[ID].RobotHandCurrent[1] << "\n";
//			LogFile << "RobotAmrPos : " << LogData[ID].RobotArmPos[0] << ", " << LogData[ID].RobotArmPos[1] << ", "
//			<< LogData[ID].RobotArmPos[2] << ", " << LogData[ID].RobotArmPos[3] << ", "
//			<< LogData[ID].RobotArmPos[4] << ", " << LogData[ID].RobotArmPos[5] << "\n";
//			LogFile << "RobotArmTorque : " << LogData[ID].RobotArmTorque[0] << ", " << LogData[ID].RobotArmTorque[1] << ", "
//			<< LogData[ID].RobotArmTorque[2] << ", " << LogData[ID].RobotArmTorque[3] << ", " << LogData[ID].RobotArmTorque[4] << ", "
//			<< LogData[ID].RobotArmTorque[5] << ", " << LogData[ID].RobotArmTorque[6] << "\n";
//			LogFile << "TactileData : " << LogData[ID].TactileData[0] << ", " << LogData[ID].TactileData[1] << ", "
//			<< LogData[ID].TactileData[2] << ", " << LogData[ID].TactileData[3] << ", "
//			<< LogData[ID].TactileData[4] << ", " << LogData[ID].TactileData[5] << ", "
//			<< LogData[ID].TactileData[6] << ", " << LogData[ID].TactileData[7] << "\n\n";
//		}
//		Sleep(500);
//		if (numloop > NumRecord) {
//			LogFile.close();
//			flag = false;
//			(*MyCloseAPI)();
//			FreeLibrary(commandLayer_handle);
//			glutDestroyWindow(glutGetWindow());
//			Write2(1, 64, 0, 4); Write2(2, 64, 0, 4); // Torque OFF
//			dxl_terminate();
//			cout << "End Program" << endl;
//			Sleep(1000);
//			sample.release();
//			exit(EXIT_FAILURE);
//		}
//	}
//}
//
//
//void idle()
//{
//	glutPostRedisplay();
//
//	NumofBodies = sample.getNumOfBodies();
//	
//	CartesianPosition currentPosition;
//	currentPosition.InitStruct();
//	MyGetCartesianCommand(currentPosition);
//	
//	TrajectoryPoint pointToSend;
//	pointToSend.InitStruct();
//	pointToSend.Position.Type = CARTESIAN_POSITION;
//	pointToSend.Position.CartesianPosition.X = currentPosition.Coordinates.X;
//	pointToSend.Position.CartesianPosition.Y = currentPosition.Coordinates.Y;
//	pointToSend.Position.CartesianPosition.Z = currentPosition.Coordinates.Z;
//	pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
//	pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
//	pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ;
//	if (G_Mode_S == Wait) {		
//		RHandPos = sample.getRHandPosReal();
//		HeadPos = sample.getHeadPosReal();
//		if (*(RHandPos + 1) < -0.5f) {
//			goal[0] = *(RHandPos);
//			goal[1] = *(RHandPos + 1) + 0.3f; // 0.6f*H_Y - (R_Y - 0.6f*H_Y);
//			goal[2] = *(RHandPos + 2);
//		}
//
//		MyGetCartesianCommand(currentPosition);
//		rob_pos[0] = currentPosition.Coordinates.X;
//		rob_pos[1] = currentPosition.Coordinates.Y;
//		rob_pos[2] = currentPosition.Coordinates.Z;
//
//		Dtogoal = sqrt(pow(goal[0] - rob_pos[0], 2) + pow(goal[1] - rob_pos[1], 2) + pow(goal[2] - rob_pos[2], 2));
//		if (Dtogoal < 0.30) {
//			G_Mode_S = HandTracking;		
//			cout << "Start Hand Tracking" << endl;
//		}
//	} else if (G_Mode_S == HandTracking) //G_MODE_S == HandTracking
//	{
//		if (NumofBodies > 0) {
//			cout << "Body Detected" << endl;
//			RHandPos = sample.getRHandPosReal();
//			HeadPos = sample.getHeadPosReal();
//			if (*(RHandPos + 1) < -0.5f) {
//				goal[0] = *(RHandPos);
//				goal[1] = *(RHandPos + 1) + 0.3f; // 0.6f*H_Y - (R_Y - 0.6f*H_Y);
//				goal[2] = *(RHandPos + 2);
//			}
//
//			MyGetCartesianCommand(currentPosition);
//			rob_pos[0] = currentPosition.Coordinates.X;
//			rob_pos[1] = currentPosition.Coordinates.Y;
//			rob_pos[2] = currentPosition.Coordinates.Z;
//
//			Dtogoal = sqrt(pow(goal[0] - rob_pos[0] - momentum[0], 2) + pow(goal[1] - rob_pos[1] - momentum[1], 2) + pow(goal[2] - rob_pos[2] - momentum[2], 2));
//
//			if (Dtogoal > 0.20) {
//				//reset PF
//				GUrep_bnd[0] = 0; GUrep_bnd[1] = 0;	GUrep_bnd[2] = 0;
//				GUrep_obs[0] = 0; GUrep_obs[1] = 0;	GUrep_obs[2] = 0;
//
//				//virtual position temp : position moved by momentum
//				MyGetCartesianCommand(currentPosition);
//				rob_pos[0] = currentPosition.Coordinates.X;
//				rob_pos[1] = currentPosition.Coordinates.Y;
//				rob_pos[2] = currentPosition.Coordinates.Z;
//
//				temp[0] = rob_pos[0] + rate*momentum[0];
//				temp[1] = rob_pos[1] + rate*momentum[1];
//				temp[2] = rob_pos[2] + rate*momentum[2];
//				
//
//				//float bnd[2][3] = { { -0.3,-0.7,-0.1 },{ 0.4,-0.3,0.7 } };	//boundary
//				//float bnd_center[] = { 0.5*(bnd[1][1] + bnd[2][1]),0.5*(bnd[1][2] + bnd[2][2]),0.5*(bnd[1][3] + bnd[2][3]) };
//				goal[0] = (goal[0] > bnd[1][0]) ? bnd[1][0] - 0.01 : (goal[0] < bnd[0][0]) ? bnd[0][0] + 0.01 : goal[0];
//				goal[1] = (goal[1] > bnd[1][1]) ? bnd[1][1] - 0.01 : (goal[1] < bnd[0][1]) ? bnd[0][1] + 0.01 : goal[1];
//				goal[2] = (goal[2] > bnd[1][2]) ? bnd[1][2] - 0.01 : (goal[2] < bnd[0][2]) ? bnd[0][2] + 0.01 : goal[2];
//
//				//boundary PF at temp
//				for (int i = 0; i < 3; i++)
//				{
//					D = min(abs(bnd[0][i] - temp[i]), abs(bnd[1][i] - temp[i]));
//					Cons = Nu*(1.0 / ObsTh - 1.0 / D)*pow(1.0 / D, 2); //negative
//					DtoCenter = sqrt(pow(bnd_center[0] - temp[0], 2) + pow(bnd_center[1] - temp[1], 2) + pow(bnd_center[2] - temp[2], 2));
//					if (D <= ObsTh)
//					{
//						GUrep_bnd[i] = Cons*((bnd_center[i] - temp[i]) / DtoCenter);
//					}
//				}
//
//				//obstacles PF at temp
//				for (int i = 0; i < obsnum; i++)
//				{
//					D = sqrt(pow(obs[i][0] - temp[0], 2) + pow(obs[i][1] - temp[1], 2) + pow(obs[i][2] - temp[2], 2)) - obs[i][3];
//					if (D <= ObsTh)
//					{
//						DtoCenter = sqrt(pow(obs[i][0] - temp[0], 2) + pow(obs[i][1] - temp[1], 2) + pow(obs[i][2] - temp[2], 2));
//						Cons = Nu*(1.0 / ObsTh - 1.0 / D)*pow(1.0 / D, 2); //negative
//						GUrep_obs[0] = GUrep_obs[0] + Cons*((temp[0] - obs[i][0]) / DtoCenter); // x direction
//						GUrep_obs[1] = GUrep_obs[1] + Cons*((temp[1] - obs[i][1]) / DtoCenter); // y direction
//						GUrep_obs[2] = GUrep_obs[2] + Cons*((temp[2] - obs[i][2]) / DtoCenter); // z direction
//					}
//				}
//
//				GUrep[0] = GUrep_bnd[0] + GUrep_obs[0];
//				GUrep[1] = GUrep_bnd[1] + GUrep_obs[1];
//				GUrep[2] = GUrep_bnd[2] + GUrep_obs[2];
//
//				GUatt[0] = Kappa * (temp[0] - goal[0]);
//				GUatt[1] = Kappa * (temp[1] - goal[1]);
//				GUatt[2] = Kappa * (temp[2] - goal[2]);
//
//				gradient[0] = -GUrep[0] - GUatt[0];
//				gradient[1] = -GUrep[1] - GUatt[1];
//				gradient[2] = -GUrep[2] - GUatt[2];
//
//				norm_gradient = sqrt(pow(gradient[0], 2) + pow(gradient[1], 2) + pow(gradient[2], 2));
//
//				//momentum(delta pos) = rate*previous momentum + PF at temp
//				momentum[0] = rate * momentum[0] + stepsize*gradient[0] / norm_gradient;
//				momentum[1] = rate * momentum[1] + stepsize*gradient[1] / norm_gradient;
//				momentum[2] = rate * momentum[2] + stepsize*gradient[2] / norm_gradient;
//				norm_momentum = sqrt(pow(momentum[0], 2) + pow(momentum[1], 2) + pow(momentum[2], 2));
//				if (norm_momentum > 0.03) {
//					momentum[0] = stepsize * momentum[0] / norm_momentum;
//					momentum[1] = stepsize * momentum[1] / norm_momentum;
//					momentum[2] = stepsize * momentum[2] / norm_momentum;
//				}
//
//				//// Direct Control
//				//new_pos[0] = rob_pos[0] + momentum[0];
//				//new_pos[1] = rob_pos[1] + momentum[1];
//				//new_pos[2] = rob_pos[2] + momentum[2];
//				//				
//				//double norm_newpos = sqrt(pow((goal[0] - rob_pos[0]), 2) + pow((goal[1] - rob_pos[1]), 2) + pow((goal[2] - rob_pos[2]), 2));
//				//norm_newpos = (norm_newpos == 0) ? 1 : norm_newpos;
//				//new_pos[0] = rob_pos[0] + stepsize*(goal[0] - rob_pos[0]) / norm_newpos;
//				//new_pos[1] = rob_pos[1] + stepsize*(goal[1] - rob_pos[1]) / norm_newpos;
//				//new_pos[2] = rob_pos[2] + stepsize*(goal[2] - rob_pos[2]) / norm_newpos;
//
//				//send the robot to next pos
//				MyGetCartesianCommand(currentPosition);
//				rob_pos[0] = currentPosition.Coordinates.X;
//				rob_pos[1] = currentPosition.Coordinates.Y;
//				rob_pos[2] = currentPosition.Coordinates.Z;
//				pointToSend.Position.CartesianPosition.X = rob_pos[0] + momentum[0];
//				pointToSend.Position.CartesianPosition.Y = rob_pos[1] + momentum[1];
//				pointToSend.Position.CartesianPosition.Z = rob_pos[2] + momentum[2];
//				pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
//				pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
//				pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ;
//				
//				Dtogoal = sqrt(pow(goal[0] - rob_pos[0] - momentum[0], 2) + pow(goal[1] - rob_pos[1] - momentum[1], 2) + pow(goal[2] - rob_pos[2] - momentum[2], 2));
//				//MyEraseAllTrajectories();
//				MySendBasicTrajectory(pointToSend);
//			}
//		}
//		else {
//			G_Mode_S = Wait;
//			cout << "Body Lost" << endl;
//		}
//	} else if (G_Mode_S == Direct) 
//	{
//
//		 int TactPattern = (tactdata.Tactile[12] > 2.5) ? 128 : 0;
//		 TactPattern = (tactdata.Tactile[13] > 2.5) ? TactPattern + 64 : TactPattern;
//		 TactPattern = (tactdata.Tactile[14] > 2.5) ? TactPattern + 32 : TactPattern;
//		 TactPattern = (tactdata.Tactile[15] > 2.5) ? TactPattern + 16 : TactPattern;
//		 TactPattern = (tactdata.Tactile[28] > 2.5) ? TactPattern + 8 : TactPattern;
//		 TactPattern = (tactdata.Tactile[29] > 2.5) ? TactPattern + 4 : TactPattern;
//		 TactPattern = (tactdata.Tactile[30] > 2.5) ? TactPattern + 2 : TactPattern;
//		 TactPattern = (tactdata.Tactile[31] > 2.5) ? TactPattern + 1 : TactPattern;
//		 cout << "TactPattern:" << TactPattern << endl;
//		switch (TactPattern) {
//		case 129: {
//		 	MyStopForceControl();
//		 	cout << "Move UP " << endl;
//		 	pointToSend.Position.CartesianPosition.X = currentPosition.Coordinates.X - 0.005*sin(PI*(currentPosition.Coordinates.ThetaZ - 0.6) / 180.f);// +0.005*cos(PI*currentPosition.Coordinates.ThetaY / 180.0f) + 0.005*cos(PI*currentPosition.Coordinates.ThetaZ / 180.0f);
//		 	pointToSend.Position.CartesianPosition.Y = currentPosition.Coordinates.Y;// +0.005*cos(PI*currentPosition.Coordinates.ThetaX / 180.0f) + 0.005*cos(PI*currentPosition.Coordinates.ThetaZ / 180.0f);
//		 	pointToSend.Position.CartesianPosition.Z = currentPosition.Coordinates.Z + 0.005*cos(PI*(currentPosition.Coordinates.ThetaZ - 0.6) / 180.f);// +0.005*sin(PI*currentPosition.Coordinates.ThetaX / 180.0f) + 0.005*sin(PI*currentPosition.Coordinates.ThetaY / 180.0f);
//		 	pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
//		 	pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
//		 	pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ;
//		 	MySendBasicTrajectory(pointToSend);
//		 	break;
//		}
//		case 24: {
//		 	MyStopForceControl();
//		 	cout << "Move Down " << endl;
//		 	pointToSend.Position.CartesianPosition.X = currentPosition.Coordinates.X + 0.005*sin(PI*(currentPosition.Coordinates.ThetaZ - 0.6) / 180.f);// -0.005*cos(PI*currentPosition.Coordinates.ThetaY / 180.0f) - 0.005*cos(PI*currentPosition.Coordinates.ThetaZ / 180.0f);
//		 	pointToSend.Position.CartesianPosition.Y = currentPosition.Coordinates.Y;// -0.005*cos(PI*currentPosition.Coordinates.ThetaX / 180.0f) - 0.005*cos(PI*currentPosition.Coordinates.ThetaZ / 180.0f);
//		 	pointToSend.Position.CartesianPosition.Z = currentPosition.Coordinates.Z - 0.005*cos(PI*(currentPosition.Coordinates.ThetaZ - 0.6) / 180.f);//- 0.005*sin(PI*currentPosition.Coordinates.ThetaX / 180.0f) - 0.005*sin(PI*currentPosition.Coordinates.ThetaY / 180.0f);
//		 	pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
//		 	pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
//		 	pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ;
//		 	MySendBasicTrajectory(pointToSend);
//		 	break;
//		}
//		case 1: case 16: case 17: {
//		 	MyStopForceControl();
//		 	cout << "Rotate CW----------------- " << endl;
//		 	pointToSend.Position.CartesianPosition.X = currentPosition.Coordinates.X;
//		 	pointToSend.Position.CartesianPosition.Y = currentPosition.Coordinates.Y;
//		 	pointToSend.Position.CartesianPosition.Z = currentPosition.Coordinates.Z;
//		 	pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
//		 	pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
//		 	pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ - 0.03;
//		 	MySendBasicTrajectory(pointToSend);
//		 	break;
//		}
//		case 8: case 128: case 136: {
//		 	MyStopForceControl();
//		 	cout << "Rotate CCW " << endl;
//		 	pointToSend.Position.CartesianPosition.X = currentPosition.Coordinates.X;
//		 	pointToSend.Position.CartesianPosition.Y = currentPosition.Coordinates.Y;
//		 	pointToSend.Position.CartesianPosition.Z = currentPosition.Coordinates.Z;
//		 	pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
//		 	pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
//		 	pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ + 0.03;
//		 	MySendBasicTrajectory(pointToSend);
//		 	break;
//		}
//		case 2: case 4: case 6: {
//		 	MyStopForceControl();
//		 	cout << "Passivity Motor 1" << endl;		 
//		 	break;
//		}
//		case 32: case 64: case 96: {
//		 	MyStopForceControl();
//		 	cout << "Passivity Motor 2" << endl;
//		 	break;
//		}
//		case 34: case 36: case 66: case 68: case 38: case 70: case 98: case 100: {
//		 	Write2(1, 102, 150, 2);	Write2(2, 102, 150, 2);
//		 	MyStartForceControl();
//		}
//		}
//	}
//
//	Sleep(100);
//
//	//	////case Grasping:
//}
//
//void showHelpInfo()
//{
//	std::cout << "Usage: nuitrack_gl_sample [path/to/nuitrack.config]\n"
//		"Press Esc to close window." << std::endl;
//}
//
//void InitializeDynamixel() {
//	char input[MAX_IN_CHAR];
//	char *token, *context;
//	char param[20][30];
//	char cmd[80];
//	int port_num = 0, baud_rate = 0, input_len, num_param;
//
//
//	port_num = 3;
//	baud_rate = 57600;
//
//	printf("\nYour input info. is\n");
//	printf("COM port number : %d\n", port_num);
//	printf("       Baudrate : %d\n", baud_rate);
//
//
//	if (dxl_initialize(port_num, baud_rate) == 0)
//	{
//		printf("Failed to open USB2Dynamixel!\n");
//		printf("Press any key to terminate...\n");
//		_getch();
//		return;
//	}
//	else
//		printf("Succeed to open USB2Dynamixel!\n\n");
//
//
//	printf("\n");
//	printf("Ping Using Protocol 2.0\n");
//	for (int i = 1; i < 3; i++)
//	{
//		dxl2_ping(i);
//		if (dxl_get_comm_result() == COMM_RXSUCCESS)
//			printf("                   ... SUCCESS \r");
//		else
//			printf("                   ... FAIL \r");
//		printf(" CHECK ID : %d \n", i);
//	}
//	printf("\n");
//}
//
//int main(int argc, char* argv[])
//{
//	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");
//
//	//We load the functions from the library
//	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
//	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
//	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
//	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
//	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
//	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
//	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
//	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
//	MyGetAngularCurrent = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularCurrent");
//	MyEraseAllTrajectories = (int(*)()) GetProcAddress(commandLayer_handle, "EraseAllTrajectories");
//	MyStartForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StartForceControl");
//	MyStopForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StopForceControl");
//
//	//Verify that all functions has been loaded correctly
//	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
//		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) ||
//		(MyMoveHome == NULL) || (MyInitFingers == NULL) || (MyGetAngularCurrent == NULL) ||
//		(MyEraseAllTrajectories == NULL) || (MyStartForceControl == NULL) || (MyStopForceControl == NULL))
//
//
//	{
//		std::cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
//		programResult = 0;
//		return 0;
//	}
//	else
//	{
//		std::cout << "I N I T I A L I Z A T I O N   C O M P L E T E D - M A I N" << endl << endl;
//	}
//	int result = (*MyInitAPI)();
//
//	std::cout << "Main Initialization's result :" << result << endl;
//
//	KinovaDevice list[MAX_KINOVA_DEVICE];
//
//	int devicesCount = MyGetDevices(list, result);
//
//	std::cout << "Found a robot on the USB bus (" << list[0].SerialNumber << ")" << endl;
//
//	// Setting the current device as the active device.
//	MySetActiveDevice(list[0]);
//
//	std::cout << "Send the Robot to Home Position" << endl;
//	MyMoveHome();
//	Sleep(3000);
//
//	std::cout << "Send the Robot to Initial Position" << endl;
//	TrajectoryPoint HomePosition;
//	HomePosition.InitStruct();
//	HomePosition.Position.Type = ANGULAR_POSITION;
//	HomePosition.Position.Actuators.SetValues(360, 143.3, -85, 53.3, 220, 251.9, 300);
//	MySendBasicTrajectory(HomePosition); Sleep(2000);
//	HomePosition.Position.Actuators.SetValues(445.8, 143.3, -169.8, 53.3, 181.9, 251.9, 322.1);
//	MySendBasicTrajectory(HomePosition); Sleep(2000);
//	std::cout << "Initializing Fingers" << endl;
//
//	InitializeDynamixel();
//	Read2(1, 132, 4); Read2(2, 132, 4);
//	Write2(1, 64, 0, 4); Write2(2, 64, 0, 4); // Torque OFF
//	Write2(1, 11, 5, 1); Write2(2, 11, 5, 1);
//	Write2(1, 64, 1, 4); Write2(2, 64, 1, 4); // Torque ON
//	Write2(1, 116, 620, 4); Write2(2, 116, 460, 4); Sleep(500);
//	Write2(1, 116, 450, 4); Write2(2, 116, 290, 4); Sleep(500);
//
//	Write2(1, 64, 0, 4); Write2(2, 64, 0, 4); // Torque OFF
//	Write2(1, 11, 0, 1); Write2(2, 11, 0, 1);
//	Read2(1, 11, 1); Read2(2, 11, 1); Sleep(500);
//	Write2(1, 64, 1, 4); Write2(2, 64, 1, 4); // Torque ON
//	Sleep(500);
//	Write2(1, 102, 30, 2);	Write2(2, 102, 50, 2);
//
//	// UDP Thread Initialization
//	boost::thread U_thread_1 = boost::thread(&UDPthread_1, ref(flag));
//	boost::thread U_thread_2 = boost::thread(&UDPthread_2, ref(flag));
//
//	// Save Data Thread
//	boost::thread R_thread = boost::thread(&record_Data, ref(flag));
//	LogFile.open("LogData.txt");
//	cout << "Open Log File ..." << endl;
//	// Nuitrack Initialize
//	showHelpInfo();
//	sample.init();
//	auto outputMode = sample.getOutputMode();
//	// Initialize GLUT window
//	glutInit(&argc, argv);
//	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
//	glutInitWindowSize(outputMode.xres, outputMode.yres);
//	glutCreateWindow("Nuitrack GL Sample (Nuitrack API)");
//	//glutSetCursor(GLUT_CURSOR_NONE);
//
//	// Connect GLUT callbacks
//	glutKeyboardFunc(keyboard);
//	glutDisplayFunc(display);
//	glutIdleFunc(idle);
//	glutMouseFunc(mouseClick);
//
//	// Setup OpenGL
//	glDisable(GL_DEPTH_TEST);
//	glEnable(GL_TEXTURE_2D);
//
//	glEnableClientState(GL_VERTEX_ARRAY);
//	glDisableClientState(GL_COLOR_ARRAY);
//
//
//	glOrtho(0, outputMode.xres, outputMode.yres, 0, -1.0, 1.0);
//	glMatrixMode(GL_PROJECTION);
//	glPushMatrix();
//	glLoadIdentity();
//
//	// Pico Flexx Camera Initialization
//	sample_utils::PlatformResources resources;
//	MyListener Picolistener;
//	std::unique_ptr<royale::ICameraDevice> cameraDevice;
//	{
//		royale::CameraManager manager;
//
//		royale::Vector<royale::String> camlist(manager.getConnectedCameraList());
//		cout << "Detected " << camlist.size() << "pico camera(s)." << endl;
//
//		if (!camlist.empty())
//		{
//			cameraDevice = manager.createCamera(camlist[0]);
//		}
//		else
//		{
//			cerr << "No suitable camera device detected." << endl;
//			return 1;
//		}
//
//		camlist.clear();
//	}
//
//	if (cameraDevice == nullptr)
//	{
//		// no cameraDevice available
//		cerr << "Cannot create the camera device" << endl;
//		return 1;
//	}
//
//	// IMPORTANT: call the initialize method before working with the camera device
//	auto status = cameraDevice->initialize();
//	if (status != royale::CameraStatus::SUCCESS)
//	{
//		cerr << "Cannot initialize the camera device, error string : " << getErrorString(status) << endl;
//		return 1;
//	}
//
//	royale::String usecaseName = "MODE_5_35FPS_600"; // MODE_9_5FPS_2000,		MODE_9_10FPS_1000,		MODE_9_15FPS_700,		MODE_9_25FPS_450,
//													 // MODE_5_35FPS_600,		MODE_5_45FPS_500,		MODE_MIXED_30_5
//
//	status = cameraDevice->setUseCase(usecaseName);
//	if (status == royale::CameraStatus::SUCCESS) {
//		cout << " The use cases is successfully adapted to " << usecaseName << endl;
//	}
//	else {
//		cout << "Fail to set use case." << endl;
//	}
//
//	// retrieve the lens parameters from Royale
//	royale::LensParameters lensParameters;
//	status = cameraDevice->getLensParameters(lensParameters);
//	if (status != royale::CameraStatus::SUCCESS)
//	{
//		cerr << "Can't read out the lens parameters" << endl;
//		return 1;
//	}
//
//	Picolistener.setLensParameters(lensParameters);
//	Picolistener.toggleUndistort(); //한번 toggle하고 시작
//
//									// register a data listener
//	if (cameraDevice->registerDataListener(&Picolistener) != royale::CameraStatus::SUCCESS)
//	{
//		cerr << "Error registering data listener" << endl;
//		return 1;
//	}
//
//	// create windows
//	//cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
//	//cv::namedWindow("Gray", cv::WINDOW_AUTOSIZE);
//	//cv::namedWindow("Test_G", WINDOW_AUTOSIZE);
//	cv::namedWindow("Test_Z", cv::WINDOW_AUTOSIZE);
//
//	// start capture mode
//	if (cameraDevice->startCapture() != royale::CameraStatus::SUCCESS)
//	{
//		cerr << "Error starting the capturing" << endl;
//		return 1;
//	}
//
//	// Pico scanning
//
//	std::cout << "*********************START***************************" << endl;
//
//	// Start main loop
//	glutMainLoop();
//
//	// Terminate	
//	cout << "stopping thread" << endl;
//	flag = false;
//	U_thread_1.join();
//	U_thread_2.join();
//	R_thread.join();
//
//	return programResult;
//}