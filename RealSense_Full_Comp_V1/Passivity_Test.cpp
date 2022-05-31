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
//// State machine
//enum G_Mode {
//	Wait,
//	Direct,
//	HandTracking,
//	Grasping
//};
//
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
//float Kappa = 0.4;
//float Nu = 1.0e-6;
//float rate = 0.9;
//float ObsTh = 0.05;
//float start[] = { 0.034,-0.2,0.26 };
//float temp[] = { 0,0,0 };
//float start_theta[] = { -3.14,0.0,0.0 };
//float goal[] = { 0.0,0.0,0.0 };
//float momentum[] = { 0,0,0 };
//float obs[2][4] = { { 0.373,-0.44,0.190,0.12 },{ 0.163,-0.587,0.100,0.12 } }; // x,y,z position + R radius
//float stepsize = 0.01;
//int obsnum = 2;
//float goal_theta[] = { 3.14,0.0,0.0 };
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
//#define PI 3.141592
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
//
//int main() {
//	InitializeDynamixel();
//	Read2(1, 132, 4); Read2(2, 132, 4);
//	Write2(1, 64, 0, 4); Write2(2, 64, 0, 4); // Torque OFF
//	Write2(1, 11, 5, 1); Write2(2, 11, 5, 1);
//	Write2(1, 64, 1, 4); Write2(2, 64, 1, 4); // Torque ON
//	Write2(1, 116, 620, 4); Write2(2, 116, 460, 4); Sleep(500);
//	Write2(1, 116, 450, 4); Write2(2, 116, 290, 4); Sleep(500);
//	Write2(1, 64, 0, 4); Write2(2, 64, 0, 4); // Torque OFF
//	Write2(1, 11, 0, 1); Write2(2, 11, 0, 1);
//	Read2(1, 11, 1); Read2(2, 11, 1); Sleep(500);
//	Write2(1, 64, 1, 4); Write2(2, 64, 1, 4); // Torque ON
//	Sleep(500);
//	
//	Write2(1, 102, 50, 2);	
//	Write2(2, 102, 50, 2);
//	Sleep(3000);
//	int n = 0;
//	float prev_xc1, prev_xc2;
//	float xc1 = Read2(1, 132, 4, 1);
//	float xc2 = Read2(2, 132, 4, 1);
//	prev_xc1 = xc1;
//	prev_xc2 = xc2;
//	while (n<150) {
//		cout << "Current 1: " << Read2(1, 126, 2, 1) << ", 2: " << Read2(2, 126, 2, 1) << endl;
//		xc1 = Read2(1, 132, 4, 1); 
//		xc2 = Read2(2, 132, 4, 1);
//		float dpass = 340 - (xc1 - 450) - (xc2 - 290);
//		float xd1 = 690.0f;
//		float xd2 = 540.0f;
//		float xdot_c1 = xc1 - prev_xc1;
//		float xdot_c2 = xc2 - prev_xc2;
//
//		float K1 = 0.28f;
//		float D1 = 0.06f;
//		float K2 = 0.28f;
//		float D2 = 0.06f;
//		
//		float cur1 = K1*(xd1 - xc1) + D1*xdot_c1 +50;// -D*xdot_c1;
//		int cur2 = (dpass < 0)? 80 : 1.4 * dpass + 80;
//		cur2 = (cur2 > 150)? 150 : cur2;
//		//K2*(xd2 - xc2) + D2*xdot_c2 + 50;// -D*xdot_c1;
//		cout << "Velocity 1 : " << xdot_c1 << endl;
//		cout << "Stiffness 1 : " << K1*(xd1 - xc1) << ", Damping 1 : " << D1*xdot_c1 << endl;
//		
//		cout << "Desired Position 1 " << xd1 << ", Current Position 1 " << xc1 << endl;
//		cout << "Desired Current 1 " << cur1 << ", Current Current 1 " << Read2(1, 126, 2, 1) << endl;
//		cout << "Passivity Distance 2 " << dpass << endl;
//		cout << "Desired Current 2 " << cur2 << ", Current Current 2 " << Read2(2, 126, 2, 1) << endl;
//		if (cur1 < 0) {
//			cur1 = cur1 + 65535;
//		}
//		Write2(1, 102, cur1, 2); 
//		if (cur2 < 0) {
//			cur2 = cur2 + 65535;
//		}
//		Write2(2, 102, cur2, 2);
//		Sleep(80);
//		n++;
//	}
//	Write2(1, 64, 0, 4); Write2(2, 64, 0, 4); // Torque OFF
//
//
//	return 0;
//}