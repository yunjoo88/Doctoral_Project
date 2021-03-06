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
//// State machine
//enum G_Mode {
//	Direct,
//	HandTracking,
//	FreeMoving
//};
//
////global variables
//int programResult = 0;
//float* RHandPos;
//float* LHandPos;
//float* HeadPos;
//int NumofBodies = 0;
//int FirstDetect = 0;
//int Right_Hand_Grip = -1;
//int Left_Hand_Grip = -1;
//int Bodyflag = 0;
//double rob_pos[3];
//double Dtogoal = 1000;
//double GUrep_bnd[] = { 0,0,0 };
//double GUrep_obs[] = { 0,0,0 };
//double GUrep[] = { 0,0,0 };
//double GUatt[3];
//double gradient[3];
//double D;
//double DtoCenter;
//double Cons;
//double norm_gradient;
//int numloop = 0;
//double bnd[2][3] = { { -0.3,-0.7,-0.1 },{ 0.4,-0.3,0.7 } };	//boundary
//double bnd_center[] = { 0.5*(bnd[1][1] + bnd[2][1]),0.5*(bnd[1][2] + bnd[2][2]),0.5*(bnd[1][3] + bnd[2][3]) };
//int T_gap = 1200;
//int c_gap = -5000;
//double norm_momentum = 0.0;
//int imagetype = -1;
//int obj_id = 1000;
//float Down_Z = 1000;
//cv::Vec3f d_XYZt_Kinova = { 0,0,0 };
//atomic<bool> flag = true;
//
////test case 3 - w/ momentum, 2 balls
//double Kappa = 0.4;
//double Nu = 1.0e-6;
//double rate = 0.9;
//double ObsTh = 0.05;
//double start[] = { 0.034,-0.2,0.26 };
//double temp[] = { 0,0,0 };
//double start_theta[] = { -3.14,0.0,0.0 };
//double goal[] = { 0.0,0.0,0.0 };
//double momentum[] = { 0,0,0 };
//double obs[2][4] = { { 0.373,-0.44,0.190,0.12 },{ 0.163,-0.587,0.100,0.12 } }; // x,y,z position + R radius
//double stepsize = 0.01;
//int obsnum = 2;
//double goal_theta[] = { 3.14,0.0,0.0 };
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
//		cout << "Can't start Winsock!" << wsOk;
//	}
//	cout << "start winsock" << endl;
//	SOCKET in = socket(AF_INET, SOCK_DGRAM, 0); // in ?? ???? interger
//												// UDP?? packet?? ?????????? ???????? ????. ?????? ?? ????. 
//												// TCP_IP?? 5?? ???????????? ??????. ???????? packet?? ?????? ?? ??????.
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
//		cout << "Can't bind socket! " << WSAGetLastError() << endl;
//	}
//
//	int Tacloop = 0;
//	char *Tacloop_S;
//	char buf[1024];	// message from client saved to buf
//
//	while (flag) {
//		cout << "Thread loop ... " << flag << endl;
//		/*char sendbuf[100] = "Server send\n";
//		int bytesIn = sendto(in, sendbuf, strlen(sendbuf), 0, (struct sockaddr*) &client, sizeof(client));
//		if(bytesIn == SOCKET_ERROR) {
//		cout << "Error sending to client " << WSAGetLastError() << endl;
//		continue;
//		}*/
//		ZeroMemory(buf, 1024);
//		// Wait for message
//		int bytesIn = recvfrom(in, buf, 1024, 0, (sockaddr*)&client, &clientLength); // recv ?? TCP??
//		if (bytesIn == SOCKET_ERROR)
//		{
//			cout << "Error receiving from client " << WSAGetLastError() << endl;
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
//		printf("????: %s\n", buf);
//		//strtok_s ???? ????
//		//printf("== ???????? ????, ??????, ???????? ???????? ???? ==\n");
//		pos = strtok_s(buf, "SBF():,", &context);  //???? ???? ???? ???? ?????? ????
//		Tacloop = atoi(pos);
//		printf("%d : ", Tacloop);
//		while (pos != NULL)
//		{
//			pos = strtok_s(context, "SBF():,", &context);//?? ?? NULL ???? context ????
//			if (pos != NULL) {
//				printf("%f", atof(pos));
//			}
//		}
//		printf("\n Copy Done");
//	}
//	cout << "UDP Done" << endl;
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
//		cout << "Can't start Winsock!" << wsOk;
//	}
//	cout << "start winsock" << endl;
//	SOCKET in = socket(AF_INET, SOCK_DGRAM, 0); // in ?? ???? interger
//												// UDP?? packet?? ?????????? ???????? ????. ?????? ?? ????. 
//												// TCP_IP?? 5?? ???????????? ??????. ???????? packet?? ?????? ?? ??????.
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
//		cout << "Can't bind socket! " << WSAGetLastError() << endl;
//	}
//
//	int Tacloop = 0;
//	char *Tacloop_S;
//	char buf[1024];	// message from client saved to buf
//
//	while (flag) {
//		cout << "Thread loop ... " << flag << endl;
//		/*char sendbuf[100] = "Server send\n";
//		int bytesIn = sendto(in, sendbuf, strlen(sendbuf), 0, (struct sockaddr*) &client, sizeof(client));
//		if(bytesIn == SOCKET_ERROR) {
//		cout << "Error sending to client " << WSAGetLastError() << endl;
//		continue;
//		}*/
//		ZeroMemory(buf, 1024);
//		// Wait for message
//		int bytesIn = recvfrom(in, buf, 1024, 0, (sockaddr*)&client, &clientLength); // recv ?? TCP??
//		if (bytesIn == SOCKET_ERROR)
//		{
//			cout << "Error receiving from client " << WSAGetLastError() << endl;
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
//		printf("????: %s\n", buf);
//		//strtok_s ???? ????
//		//printf("== ???????? ????, ??????, ???????? ???????? ???? ==\n");
//		pos = strtok_s(buf, "SBF():,", &context);  //???? ???? ???? ???? ?????? ????
//		Tacloop = atoi(pos);
//		printf("%d : ", Tacloop);
//		while (pos != NULL)
//		{
//			pos = strtok_s(context, "SBF():,", &context);//?? ?? NULL ???? context ????
//			if (pos != NULL) {
//				printf("%f", atof(pos));
//			}
//		}
//		printf("\n Copy Done");
//	}
//	cout << "UDP Done" << endl;
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
//																							//Adaptive Thresholding?? ????.
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
//void idle()
//{
//	glutPostRedisplay();
//	//printf("Width : %d, Height : %d \n", sample.getWidth(), sample.getHeight());
//	RHandPos = sample.getRHandPos();
//	LHandPos = sample.getLHandPos();
//	HeadPos = sample.getHeadPos();
//	//printf("Rx: %f Ry: %f Rz: %f // Lx : %f Ly: %f Lz: %f \n", *(RHandPos), *(RHandPos + 1), *(RHandPos + 2), *(LHandPos), *(LHandPos + 1), *(LHandPos + 2));
//	float R_X = (*(RHandPos))*0.001f - 0.20;
//	float R_Y = (-sin(PI / 4.0f)*(*(RHandPos + 2) * 0.001) - sin(PI / 4.0f)*(*(RHandPos + 1) * 0.001) - 0.16);
//	float R_Z = (-sin(PI / 4.0f)*(*(RHandPos + 2) * 0.001) + sin(PI / 4.0f)*(*(RHandPos + 1) * 0.001) + 1.30);
//	float L_X = (*(LHandPos))*0.001f - 0.20;
//	float L_Y = (-sin(PI / 4.0f)*(*(LHandPos + 2) * 0.001) - sin(PI / 4.0f)*(*(LHandPos + 1) * 0.001) - 0.16);
//	float L_Z = (-sin(PI / 4.0f)*(*(LHandPos + 2) * 0.001) + sin(PI / 4.0f)*(*(LHandPos + 1) * 0.001) + 1.30);
//	float H_X = (*(HeadPos))*0.001f - 0.20;
//	float H_Y = (-sin(PI / 4.0f)*(*(HeadPos + 2) * 0.001) - sin(PI / 4.0f)*(*(HeadPos + 1) * 0.001) + 0.16);
//	float H_Z = (-sin(PI / 4.0f)*(*(HeadPos + 2) * 0.001) + sin(PI / 4.0f)*(*(HeadPos + 1) * 0.001) + 1.30);
//	printf("Rx: %f Ry: %f Rz: %f // Lx: %f Ly: %f Lz: %f // Hx: %f Hy: %f Hz: %f \n", R_X, R_Y, R_Z, L_X, L_Y, L_Z, H_X, H_Y, H_Z);
//
//	goal[0] = R_X;
//	goal[1] = R_Y + 0.3f; // 0.5f*H_Y - (R_Y - 0.5f*H_Y);
//	goal[2] = R_Z;
//
//	CartesianPosition currentPosition;
//	MyGetCartesianCommand(currentPosition);
//	rob_pos[0] = currentPosition.Coordinates.X;
//	rob_pos[1] = currentPosition.Coordinates.Y;
//	rob_pos[2] = currentPosition.Coordinates.Z;
//	//Dtogoal = sqrt(pow(goal[0] - rob_pos[0] - momentum[0], 2) + pow(goal[1] - rob_pos[1] - momentum[1], 2) + pow(goal[2] - rob_pos[2] - momentum[2], 2));
//
//
//	////reset PF
//	//GUrep_bnd[0] = 0;
//	//GUrep_bnd[1] = 0;
//	//GUrep_bnd[2] = 0;
//	//GUrep_obs[0] = 0;
//	//GUrep_obs[1] = 0;
//	//GUrep_obs[2] = 0;
//
//	//temp[0] = rob_pos[0] + rate*momentum[0];
//	//temp[1] = rob_pos[1] + rate*momentum[1];
//	//temp[2] = rob_pos[2] + rate*momentum[2];
//
//	//////goal update
//	////if ((abs(currentPosition.Coordinates.X - goal[0]) > 0.3)
//	////	|| (abs(currentPosition.Coordinates.Y - goal[1]) > 0.3)
//	////	|| (abs(currentPosition.Coordinates.Z - goal[2]) > 0.3)
//	////	)
//	////{
//	////	std::cout << "You moved too fast!" << endl;
//	////	return;
//	////}
//
//	////boundary PF at temp
//	//for (int i = 0; i < 3; i++)
//	//{
//	//	D = abs(bnd[0][i] - temp[i]) < abs(bnd[1][i] - temp[i]) ? abs(bnd[0][i] - temp[i]) : abs(bnd[1][i] - temp[i]);
//	//	Cons = Nu*(1.0 / ObsTh - 1.0 / D)*pow(1.0 / D, 2); //negative
//	//	DtoCenter = sqrt(pow(bnd_center[0] - temp[0], 2) + pow(bnd_center[1] - temp[1], 2) + pow(bnd_center[2] - temp[2], 2));
//	//	if (D <= ObsTh)
//	//	{
//	//		GUrep_bnd[i] = Cons*((bnd_center[i] - temp[i]) / DtoCenter);
//	//	}
//	//}
//
//	////obstacles PF at temp
//	//for (int i = 0; i < obsnum; i++)
//	//{
//	//	D = sqrt(pow(obs[i][0] - temp[0], 2) + pow(obs[i][1] - temp[1], 2) + pow(obs[i][2] - temp[2], 2)) - obs[i][3];
//	//	if (D <= ObsTh)
//	//	{
//	//		DtoCenter = sqrt(pow(obs[i][0] - temp[0], 2) + pow(obs[i][1] - temp[1], 2) + pow(obs[i][2] - temp[2], 2));
//	//		Cons = Nu*(1.0 / ObsTh - 1.0 / D)*pow(1.0 / D, 2); //negative
//	//		GUrep_obs[0] = GUrep_obs[0] + Cons*((temp[0] - obs[i][0]) / DtoCenter); // x direction
//	//		GUrep_obs[1] = GUrep_obs[1] + Cons*((temp[1] - obs[i][1]) / DtoCenter); // y direction
//	//		GUrep_obs[2] = GUrep_obs[2] + Cons*((temp[2] - obs[i][2]) / DtoCenter); // z direction
//	//	}
//	//}
//
//	//GUrep[0] = GUrep_bnd[0] + GUrep_obs[0];
//	//GUrep[1] = GUrep_bnd[1] + GUrep_obs[1];
//	//GUrep[2] = GUrep_bnd[2] + GUrep_obs[2];
//
//	//GUatt[0] = Kappa * (temp[0] - goal[0]);
//	//GUatt[1] = Kappa * (temp[1] - goal[1]);
//	//GUatt[2] = Kappa * (temp[2] - goal[2]);
//
//	//gradient[0] = -GUrep[0] - GUatt[0];
//	//gradient[1] = -GUrep[1] - GUatt[1];
//	//gradient[2] = -GUrep[2] - GUatt[2];
//
//	//norm_gradient = sqrt(pow(gradient[0], 2) + pow(gradient[1], 2) + pow(gradient[2], 2));
//
//	////momentum(delta pos) = rate*previous momentum + PF at temp
//	//momentum[0] = rate * momentum[0] + stepsize*gradient[0] / norm_gradient;
//	//momentum[1] = rate * momentum[1] + stepsize*gradient[1] / norm_gradient;
//	//momentum[2] = rate * momentum[2] + stepsize*gradient[2] / norm_gradient;
//	//norm_momentum = sqrt(pow(momentum[0], 2) + pow(momentum[1], 2) + pow(momentum[2], 2));
//	//momentum[0] = stepsize * momentum[0] / norm_momentum;
//	//momentum[1] = stepsize * momentum[1] / norm_momentum;
//	//momentum[2] = stepsize * momentum[2] / norm_momentum;
//
//	norm_momentum = sqrt(pow((R_X - rob_pos[0]), 2) + pow((R_Y - rob_pos[1]), 2) + pow((R_Z - rob_pos[2]), 2));
//	momentum[0] = stepsize * (R_X - rob_pos[0]) / norm_momentum;
//	momentum[1] = stepsize * (R_Y - rob_pos[1]) / norm_momentum;
//	momentum[2] = stepsize * (R_Z - rob_pos[2]) / norm_momentum;
//	//send the robot to next pos
//	TrajectoryPoint pointToSend;
//	pointToSend.InitStruct();
//	pointToSend.Position.Type = CARTESIAN_POSITION;
//
//	pointToSend.Position.CartesianPosition.X = goal[0];
//	pointToSend.Position.CartesianPosition.Y = goal[1];
//	pointToSend.Position.CartesianPosition.Z = goal[2];
//	pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
//	pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
//	pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ;
//
//
//	Dtogoal = sqrt(pow(goal[0] - rob_pos[0] - momentum[0], 2) + pow(goal[1] - rob_pos[1] - momentum[1], 2) + pow(goal[2] - rob_pos[2] - momentum[2], 2));
//	numloop = numloop + 1;
//	MySendBasicTrajectory(pointToSend);
//
//	std::cout << numloop << endl;
//	std::cout << "rob X : " << rob_pos[0] << "	rob Y : " << rob_pos[1] << "		rob Z : " << rob_pos[2] << endl;
//	std::cout << "delta X : " << momentum[0] << "	delta Y : " << momentum[1] << "	delta Z : " << momentum[2] << endl;
//	std::cout << "goal X : " << goal[0] << "	goal Y : " << goal[1] << "	goal Z : " << goal[2] << endl << endl;
//
//	Sleep(80);
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
//
//	//Verify that all functions has been loaded correctly
//	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
//		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) ||
//		(MyMoveHome == NULL) || (MyInitFingers == NULL))
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
//	Write2(1, 64, 1, 4); Write2(2, 64, 1, 4); // Torque ON
//	Write2(1, 116, 620, 4); Write2(2, 116, 460, 4); Sleep(500);
//	Write2(1, 116, 450, 4); Write2(2, 116, 290, 4); Sleep(500);
//	// Write2(1, 64, 0, 4); Write2(2, 64, 0, 4); // Torque OFF
//
//	// UDP Thread Initialization
//	boost::thread U_thread_1 = boost::thread(&UDPthread_1, ref(flag));
//	boost::thread U_thread_2 = boost::thread(&UDPthread_2, ref(flag));
//
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
//	Picolistener.toggleUndistort(); //???? toggle???? ????
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
//	dxl_terminate();
//	return programResult;
//}