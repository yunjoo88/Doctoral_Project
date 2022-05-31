//#include <Windows.h>
//#include <conio.h>
//#include <iostream>
//#include "CommandLayer.h"
//#include "CommunicationLayerWindows.h"
//#include "KinovaTypes.h"
//#include <fstream>
//
//using namespace std;
//
////A handle to the API.
//HINSTANCE commandLayer_handle;
//
////Function pointers to the functions we need
//int(*MyInitAPI)();
//int(*MyCloseAPI)();
//int(*MyGetAngularCommand)(AngularPosition &Response);
//
//int(*MyRunGravityZEstimationSequence7DOF)(ROBOT_TYPE type, float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF]);
//int(*MySetGravityOptimalParameter)(float Command[GRAVITY_PARAM_SIZE]);
//int(*MySetGravityType)(GRAVITY_TYPE Type);
//
//int(*MyStartForceControl)();
//int(*MyStopForceControl)();
//int(*MyGetGlobalTrajectoryInfo)(TrajectoryFIFO &Response);
//
//int main(int argc, char* argv[]) {
//
//	//We load the API.
//	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");
//	AngularPosition current;
//	int result;
//	int programResult = 0;
//
//	//We load the functions from the library
//	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
//	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
//	MyGetAngularCommand = (int(*)(AngularPosition &Response)) GetProcAddress(commandLayer_handle, "GetAngularCommand");
//	MyRunGravityZEstimationSequence7DOF = (int(*)(ROBOT_TYPE type, float OptimalzParam[OPTIMAL_Z_PARAM_SIZE])) GetProcAddress(commandLayer_handle, "RunGravityZEstimationSequence7DOF");
//	MySetGravityOptimalParameter = (int(*)(float Command[GRAVITY_PARAM_SIZE])) GetProcAddress(commandLayer_handle, "SetGravityOptimalZParam");
//	MySetGravityType = (int(*)(GRAVITY_TYPE Type)) GetProcAddress(commandLayer_handle, "SetGravityType");
//
//	MyStartForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StartForceControl");
//	MyStopForceControl = (int(*)()) GetProcAddress(commandLayer_handle, "StopForceControl");
//
//	//Verify that all functions have been loaded correctly
//	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyGetAngularCommand == NULL) ||
//		(MySetGravityOptimalParameter == NULL) || (MyRunGravityZEstimationSequence7DOF == NULL) || (MySetGravityType == NULL)) {
//		cout << "* * * E R R O R D U R I N G I N I T I A L I Z A T I O N * * *" << endl;
//		programResult = 0;
//	}
//	else {
//		cout << "I N I T I A L I Z A T I O N C O M P L E T E D" << endl << endl;
//		int result = (*MyInitAPI)();
//		int resultComm;
//		AngularPosition DataCommand;
//
//		// Get the angular command to test the communication with the robot
//		resultComm = MyGetAngularCommand(DataCommand);
//		KinovaDevice list[MAX_KINOVA_DEVICE];
//
//		// If the API is initialized and the communication with the robot is working
//		if (result == 1 && resultComm == 1) {
//			// Choose robot type
//			ROBOT_TYPE type = SPHERICAL_7DOF_SERVICE;
//
//			// Run identification sequence
//			cout << "Running gravity parameters estimation trajectory..." << endl;
//			//MyRunGravityZEstimationSequence7DOF(type, OptimalzParam);
//			float OptimalzParam[OPTIMAL_Z_PARAM_SIZE_7DOF];
//			OptimalzParam[0] = 0.00232201;			OptimalzParam[1] = -1.54957;			OptimalzParam[2] = -0.0104254;
//			OptimalzParam[3] = -0.0432875;			OptimalzParam[4] = -0.00726426;			OptimalzParam[5] = -0.653612;
//			OptimalzParam[6] = -0.00406478;			OptimalzParam[7] = 0.00387636;			OptimalzParam[8] = 0.00321042;
//			OptimalzParam[9] = -0.19573;			OptimalzParam[10] = -0.00604031;			OptimalzParam[11] = -0.000174665;
//			OptimalzParam[12] = 0.326592;			OptimalzParam[13] = -0.118066;			OptimalzParam[14] = -0.390597;
//			OptimalzParam[15] = 0.340566;			OptimalzParam[16] = -0.667309;			OptimalzParam[17] = -0.626836;
//			OptimalzParam[18] = -1.74297;
//			MySetGravityOptimalParameter(OptimalzParam);
//
//			//// informs the robot on the new optimal gravity parameters
//			//MySetGravityType(OPTIMAL); //sets the gravity compensation mode to Optimal		
//			MyStartForceControl();
//			int n = 0;
//			while (n < 100000) {
//				cout << n << endl;
//				n++;
//			}
//			cout << "End loop" << endl;
//			MyStopForceControl();
//		}
//		cout << endl << "C L O S I N G A P I" << endl;
//		result = (*MyCloseAPI)();
//		programResult = 1;
//
//	}
//
//	FreeLibrary(commandLayer_handle);
//	return programResult;
//}
////
////The parameters are :
////Param[0] = 0.00232201
////Param[1] = -1.54957
////Param[2] = -0.0104254
////Param[3] = -0.0432875
////Param[4] = -0.00726426
////Param[5] = -0.653612
////Param[6] = -0.00406478
////Param[7] = 0.00387636
////Param[8] = 0.00321042
////Param[9] = -0.19573
////Param[10] = -0.00604031
////Param[11] = -0.000174665
////Param[12] = 0.326592
////Param[13] = -0.118066
////Param[14] = -0.390597
////Param[15] = 0.340566
////Param[16] = -0.667309
////Param[17] = -0.626836
////Param[18] = -1.74297