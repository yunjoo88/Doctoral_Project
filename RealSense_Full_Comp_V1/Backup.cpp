//#define UNICODE
//#ifdef _MSC_VER
//#define _CRT_SECURE_NO_WARNINGS
//#endif
//
//#include "KinovaTypes.h"
//#include <Windows.h>
//#include "CommunicationLayerWindows.h"
//#include "CommandLayer.h"
//#include <conio.h>
//#include <cstring>
//#include <iostream>
//#include <thread>
//#include <atomic>
//
//#include "NuitrackGLSample.h"
//#include <GL/glut.h>
//#include <iomanip>
//using namespace std;
//
//HINSTANCE  commandLayer_handle;
//NuitrackGLSample sample;
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
//}
//
//void showHelpInfo()
//{
//	std::cout << "Usage: nuitrack_gl_sample [path/to/nuitrack.config]\n"
//		"Press Esc to close window." << std::endl;
//}
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
//
////global variables
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
//double bnd[2][3] = { { -0.3,-0.6,-0.2 },{ 0.5,0.0,0.6 } };	//boundary
//double bnd_center[] = { 0.5*(bnd[1][1] + bnd[2][1]),0.5*(bnd[1][2] + bnd[2][2]),0.5*(bnd[1][3] + bnd[2][3]) };
//int T_gap = 1200;
//int c_gap = -5000;
//double norm_momentum = 0.0;
//
//#define PI 3.141592
//
//
//void thread_hand(atomic<bool>& flag, float* xgoal, float* ygoal, float* zgoal)
//{
//	while (flag)
//	{
//
//		if (NumofBodies > 0)
//		{
//			if (Bodyflag == 0)
//			{
//				Bodyflag = 1;
//			}
//			else //Bodyflag == 1
//			{
//				//hand position in Kinova coordinate
//				float L_X = ((Left_Hand_Pos.x * 0.001) - 0.22);
//				float L_Y = (-sin(PI / 4.0f)*(Left_Hand_Pos.z * 0.001) - sin(PI / 4.0f)*(Left_Hand_Pos.y * 0.001) - 0.05);
//				float L_Z = (-sin(PI / 4.0f)*(Left_Hand_Pos.z * 0.001) + sin(PI / 4.0f)*(Left_Hand_Pos.y * 0.001) + 1.15);
//				float R_X = ((Right_Hand_Pos.x * 0.001) - 0.22);
//				float R_Y = (-sin(PI / 4.0f)*(Right_Hand_Pos.z * 0.001) - sin(PI / 4.0f)*(Right_Hand_Pos.y * 0.001) - 0.05);
//				float R_Z = (-sin(PI / 4.0f)*(Right_Hand_Pos.z * 0.001) + sin(PI / 4.0f)*(Right_Hand_Pos.y * 0.001) + 1.15);
//
//				//update goal position (hand pos)
//				*xgoal = R_X + 0.0f;
//				*ygoal = R_Y + 0.6f;
//				*zgoal = R_Z + 0.0f;
//			}
//		}
//
//		window.clear(sf::Color::Black);
//		listener.draw_to(window);
//		window.display();
//	}
//
//
//	return;
//}
//
//int main()
//{
//	//test case 1 - hand tracking w/o momentum
//	//double Kappa = 0.4;  // Attractive Potential Gain
//	//double Nu = 1.0e-6;  // Repulsive Potential Gain
//	//double ObsTh = 0.05; // Obstacle
//	//double ObsTh = 0.03;
//	//double start[] = { 0.1,-0.3,0.5 };
//	//double start_theta[] = { -3.14,0.0,0.0 };
//	//double goal[] = { 0.0,0.0,0.0 };
//	//double obs[2][4] = { { 0.15,-0.3,0.28,0.05 },{ 0.2,-0.5,0.22,0.04 } };
//	//double stepsize = 0.01;
//	//int obsnum = 0;
//
//	//test case 2 - w/ momentum, 1 ball
//	//double Kappa = 0.4;
//	//double Nu = 1.0e-6;
//	//double rate = 0.9;
//	//double ObsTh = 0.05;
//	//double start[] = { 0.034,-0.2,0.26 };
//	//double temp[] = { 0,0,0 };
//	//double start_theta[] = { -3.14,0.0,0.0 };
//	//double goal[] = { 0.27,-0.60,-0.02 };
//	//double momentum[] = { 0,0,0 };
//	//double obs[1][4] = { { 0.18,-0.45,0.08,0.13 } };
//	//double stepsize = 0.01;
//	//int obsnum = 1;
//	//double goal_theta[] = { 3.14,0.0,0.0 };
//
//	//test case 3 - w/ momentum, 2 balls
//	double Kappa = 0.4;
//	double Nu = 1.0e-6;
//	double rate = 0.9;
//	double ObsTh = 0.05;
//	double start[] = { 0.034,-0.2,0.26 };
//	double temp[] = { 0,0,0 };
//	double start_theta[] = { -3.14,0.0,0.0 };
//	double goal[] = { 0.0,0.0,0.0 };
//	double momentum[] = { 0,0,0 };
//	double obs[2][4] = { { 0.237,-0.29,0.08,0.12 },{ 0.085,-0.49,0.02,0.12 } }; // x,y,z position + R radius
//	double stepsize = 0.01;
//	int obsnum = 2;
//	double goal_theta[] = { 3.14,0.0,0.0 };
//
//
//	int programResult = 0;
//
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
//	//Setting the current device as the active device.
//	MySetActiveDevice(list[0]);
//
//	std::cout << "Send the robot to Home position" << endl;
//	MyMoveHome();
//
//	std::cout << "Initializing the fingers" << endl;
//	MyInitFingers();
//
//	TrajectoryPoint pointToSend;
//	pointToSend.InitStruct();
//	pointToSend.Position.Type = CARTESIAN_POSITION;
//
//	atomic<bool> flag = true;
//
//	float xp, yp, zp = 0;
//	float* xgoal = &xp;
//	float* ygoal = &yp;
//	float* zgoal = &zp;
//
//	std::cout << "*********************START***************************" << endl;
//
//	std::thread hand_t(&thread_hand, ref(flag), xgoal, ygoal, zgoal);
//
//	CartesianPosition currentPosition;
//
//	//Sending to start position
//	MyGetCartesianCommand(currentPosition);
//	pointToSend.Position.CartesianPosition.Z = start[2];
//	pointToSend.Position.CartesianPosition.Y = currentPosition.Coordinates.Y;
//	pointToSend.Position.CartesianPosition.X = currentPosition.Coordinates.X;
//	pointToSend.Position.CartesianPosition.ThetaX = start_theta[0];
//	pointToSend.Position.CartesianPosition.ThetaY = start_theta[1];
//	pointToSend.Position.CartesianPosition.ThetaZ = start_theta[2];
//	MySendBasicTrajectory(pointToSend);
//	Sleep(3000);
//
//	MyGetCartesianCommand(currentPosition);
//	pointToSend.Position.CartesianPosition.X = start[0];
//	pointToSend.Position.CartesianPosition.Y = start[1];
//	pointToSend.Position.CartesianPosition.Z = currentPosition.Coordinates.Z;
//	pointToSend.Position.CartesianPosition.ThetaX = start_theta[0];
//	pointToSend.Position.CartesianPosition.ThetaY = start_theta[1];
//	pointToSend.Position.CartesianPosition.ThetaZ = start_theta[2];
//	pointToSend.Position.Fingers.Finger1 = 5700;
//	pointToSend.Position.Fingers.Finger2 = 5700;
//	pointToSend.Position.Fingers.Finger3 = 5700;
//
//	MySendBasicTrajectory(pointToSend);
//	Sleep(3000);
//
//
//	while (true)
//	{
//		//wait for next goal after reaching the goal
//		if (Bodyflag == 1)
//		{
//			MyGetCartesianCommand(currentPosition);
//			rob_pos[0] = currentPosition.Coordinates.X;
//			rob_pos[1] = currentPosition.Coordinates.Y;
//			rob_pos[2] = currentPosition.Coordinates.Z;
//			Dtogoal = sqrt(pow(*xgoal - rob_pos[0] - momentum[0], 2) + pow(*ygoal - rob_pos[1] - momentum[1], 2) + pow(*zgoal - rob_pos[2] - momentum[2], 2));
//		}
//
//		while (Dtogoal > 0.02)
//		{
//			if (Bodyflag == 1)
//			{
//				//reset PF
//				GUrep_bnd[0] = 0;
//				GUrep_bnd[1] = 0;
//				GUrep_bnd[2] = 0;
//				GUrep_obs[0] = 0;
//				GUrep_obs[1] = 0;
//				GUrep_obs[2] = 0;
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
//				//goal update
//				if ((abs(currentPosition.Coordinates.X - *xgoal) > 0.3)
//					|| (abs(currentPosition.Coordinates.Y - *ygoal) > 0.8)
//					|| (abs(currentPosition.Coordinates.Z - *zgoal) > 0.3)
//					)
//				{
//					std::cout << "You moved too fast!" << endl;
//				}
//				else
//				{
//					goal[0] = *xgoal;
//					goal[1] = *ygoal;
//					goal[2] = *zgoal;
//				}
//
//				goal[0] = *xgoal;
//				goal[1] = *ygoal;
//				goal[2] = *zgoal;
//
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
//				momentum[0] = stepsize * momentum[0] / norm_momentum;
//				momentum[1] = stepsize * momentum[1] / norm_momentum;
//				momentum[2] = stepsize * momentum[2] / norm_momentum;
//
//				//send the robot to next pos
//				pointToSend.Position.CartesianPosition.X = rob_pos[0] + momentum[0];
//				pointToSend.Position.CartesianPosition.Y = rob_pos[1] + momentum[1];
//				pointToSend.Position.CartesianPosition.Z = rob_pos[2] + momentum[2];
//				pointToSend.Position.CartesianPosition.ThetaX = currentPosition.Coordinates.ThetaX;
//				pointToSend.Position.CartesianPosition.ThetaY = currentPosition.Coordinates.ThetaY;
//				pointToSend.Position.CartesianPosition.ThetaZ = currentPosition.Coordinates.ThetaZ;
//
//
//				Dtogoal = sqrt(pow(goal[0] - rob_pos[0] - momentum[0], 2) + pow(goal[1] - rob_pos[1] - momentum[1], 2) + pow(goal[2] - rob_pos[2] - momentum[2], 2));
//				numloop = numloop + 1;
//				MySendBasicTrajectory(pointToSend);
//
//				std::cout << numloop << endl;
//				std::cout << "rob X : " << rob_pos[0] << "	rob Y : " << rob_pos[1] << "		rob Z : " << rob_pos[2] << endl;
//				std::cout << "delta X : " << momentum[0] << "	delta Y : " << momentum[1] << "	delta Z : " << momentum[2] << endl;
//				std::cout << "goal X : " << goal[0] << "	goal Y : " << goal[1] << "	goal Z : " << goal[2] << endl << endl;
//
//				Sleep(80);
//			}
//		}
//	}
//
//	flag = false;
//	hand_t.join();
//
//	result = (*MyCloseAPI)();
//	astra::terminate();
//	FreeLibrary(commandLayer_handle);
//
//	return programResult;
//}