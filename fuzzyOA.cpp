/* Fuzzy logic controller for the mobile robot.
 * By Aliaksei Laurynovich
 * For Intelligent Systems and Robotics module
 * Univerisity of Essex
 * 15.12.2018
 */

/* 
 * Obstacle avoidance behaviour.
 * The main function consists of the following parts:
 *	1) Robot hardware initialization
 *	2) Sensor data filtering
 *	3) Fuzzy logic controller
 *		- Fuzzification
 *		- Inference Engine (includes the rule base and input processing)
 */		- Defuzzification


#include <stdio.h>
#include <cmath>
#include <aria.h>
#include <iostream>
using namespace std;

//Function declarations
void remove5000(int numOfElements, int array[]);
int findAverageIgnoreZeros(int numOfElements, int array[]);
void removeHighLow(int numOfElements, int array[], int avrg);
float fsForLOW(int input, int lastHigh, int lastPoint);
float fsForMEDIUM(int input, int firstPoint, int midPoint, int lastPoint);
float fsForHIGH(int input, int firstPoint, int firstHigh);
float fsBlendEdgeFollowLOW(int input);
float fsBlendObstacleAvoidLOW(int input);


int main(int argc, char **argv)
{
	///////////////////////////////////////////////////////////////////////////////////////
	/////////////////////--------1) HARDWARE INITIALIZATIONS--------///////////////////////
	///////////////////////////////////////////////////////////////////////////////////////
	
	Aria::init();
	ArRobot robot;
	ArPose pose;
	ArSensorReading *sonarSensor[8];

	//motor speeds
	int leftVel = 0;
	int rightVel = 0;

	//sensor inputs
	int inputLEFT = 0, inputFRONT = 0, inputRIGHT = 0;

	float leftFuzzyOut[3] = { 0, 0, 0 }, rightFuzzyOut[3] = { 0, 0, 0 }; //fuzzy variables for left and right outputs

	int outputMFcenters[3] = { 100, 150, 200 }; //centers of output membership functions


	// parse command line arguments
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();


	// connect to robot
	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		cout << "Robot connected!" << endl;
	robot.runAsync(false);

	robot.lock();
	robot.enableMotors();
	robot.unlock();


	while (true)
	{
		////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////-----------2) DATA COLLECTION AND FILTERING-----------///////////////////
		////////////////////////////////////////////////////////////////////////////////////////////

		//Obtain SONAR measurements
		int sonarRange[8], s5Vector[6], s4Vector[6], s3Vector[6], s2Vector[6];
		int s5 = 5000, s4 = 5000, s3 = 5000, s2 = 5000;

		//Make 6 readings for each SONAR
		//Readings 1
		for (int i = 0; i < 8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}
		s5Vector[0] = sonarRange[5]; //save S5 reading to a array
		s4Vector[0] = sonarRange[4]; //save S4 reading to a array
		s3Vector[0] = sonarRange[3]; //save S3 reading to a array
		s2Vector[0] = sonarRange[2]; //save S2 reading to a array

		ArUtil::sleep(20); //wait for 20ms

		//Readings 2
		for (int i = 0; i < 8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}
		s5Vector[1] = sonarRange[5]; //save S5 reading to a array
		s4Vector[1] = sonarRange[4]; //save S4 reading to a array
		s3Vector[1] = sonarRange[3]; //save S3 reading to a array
		s2Vector[1] = sonarRange[2]; //save S2 reading to a array

		ArUtil::sleep(20); //wait for 20ms

		//Readings 3
		for (int i = 0; i < 8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}
		s5Vector[2] = sonarRange[5]; //save S5 reading to a array
		s4Vector[2] = sonarRange[4]; //save S4 reading to a array
		s3Vector[2] = sonarRange[3]; //save S3 reading to a array
		s2Vector[2] = sonarRange[2]; //save S2 reading to a array

		ArUtil::sleep(20); //wait for 20ms

		//Readings 4
		for (int i = 0; i < 8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}
		s5Vector[3] = sonarRange[5]; //save S5 reading to a array
		s4Vector[3] = sonarRange[4]; //save S4 reading to a array
		s3Vector[3] = sonarRange[3]; //save S3 reading to a array
		s2Vector[3] = sonarRange[2]; //save S2 reading to a array

		ArUtil::sleep(20); //wait for 20ms

		//Readings 5
		for (int i = 0; i < 8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}
		s5Vector[4] = sonarRange[5]; //save S5 reading to a array
		s4Vector[4] = sonarRange[4]; //save S4 reading to a array
		s3Vector[4] = sonarRange[3]; //save S3 reading to a array
		s2Vector[4] = sonarRange[2]; //save S2 reading to a array

		ArUtil::sleep(20); //wait for 20ms

		//Readings 6
		for (int i = 0; i < 8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}
		s5Vector[5] = sonarRange[5]; //save S5 reading to a array
		s4Vector[5] = sonarRange[4]; //save S4 reading to a array
		s3Vector[5] = sonarRange[3]; //save S3 reading to a array
		s2Vector[5] = sonarRange[2]; //save S2 reading to a array



		//Filter the arrays with s5, s4, S3, S2 readings
		remove5000(6, s5Vector);
		remove5000(6, s4Vector);
		remove5000(6, s3Vector);
		remove5000(6, s2Vector);

		s5 = findAverageIgnoreZeros(6, s5Vector);
		s4 = findAverageIgnoreZeros(6, s4Vector);
		s3 = findAverageIgnoreZeros(6, s3Vector);
		s2 = findAverageIgnoreZeros(6, s2Vector);

		removeHighLow(6, s7Vector, s7);
		removeHighLow(6, s6Vector, s6);
		removeHighLow(6, s5Vector, s5);
		removeHighLow(6, s4Vector, s4);
		removeHighLow(6, s3Vector, s3);
		removeHighLow(6, s2Vector, s2);

		//Find the average again	
		s5 = findAverageIgnoreZeros(6, s5Vector);
		s4 = findAverageIgnoreZeros(6, s4Vector);
		s3 = findAverageIgnoreZeros(6, s3Vector);
		s2 = findAverageIgnoreZeros(6, s2Vector);

		/*
		* Find LEFT input
		*/
		inputLEFT = s2;

		/*
		* Find FRONT input
		*/
		//Consider readings only from S3, if S4 has not detected anything
		if (s3 < 5000 && s4 >= 5000)
			inputFRONT = s3;
		else if (s3 < 5000 && s4 < 5000)
		{
			//Consider s3, if it's much less than s4
			if (s4 / s3 >= 2)
				inputFRONT = s3;
			//Consider s4, if it's much less than s3
			else if (s3 / s4 >= 2)
				inputFRONT = s4;
			//otherwise find the average of S3 and S4
			else
				inputFRONT = (s3 + s4) / 2; //average
		}
		//Consider readings only from S4, if S3 has not detected anything
		else if (s3 >= 5000 && s4 < 5000)
			inputFRONT = s4;

		/*
		* Find RIGHT input
		*/
		inputRIGHT = s5;


		////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////-----------FUZZY LOGIC CONTROLLER-----------/////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////


		/* Obstacle avoidance behaviour */

		// Reset fuzzy output variables
		for (int i = 0; i < 3; i++)
		{
			leftFuzzyOut[i] = 0;
			rightFuzzyOut[i] = 0;
		}


		////////////// FUZZIFICATION ////////////////

		/*
		* Find all Firing Strenghts for all inputs
		*/

		/*
		*  The table for fuzzy inputs arrays elements. Each element represents a membership value.
		* 		leftFuzzyIn[0] -- LOW		frontFuzzyIn[0] -- LOW		rightFuzzyIn[0] -- LOW
		* 		leftFuzzyIn[1] -- MEDIUM	frontFuzzyIn[1] -- MEDIUM	rightFuzzyIn[1] -- MEDIUM
		* 		leftFuzzyIn[2] -- HIGH		frontFuzzyIn[2] -- HIGH		rightFuzzyIn[2] -- HIGH
		*
		*  The same principle is for fuzzy outputs.
		*/

		float leftFuzzyIn[3], frontFuzzyIn[3], rightFuzzyIn[3]; //fuzzy variables for left, front, and right inputs


		//left fs
		leftFuzzyIn[0] = fsForLOW(inputLEFT, 600, 1000);
		leftFuzzyIn[1] = fsForMEDIUM(inputLEFT, 600, 1000, 1400);
		leftFuzzyIn[2] = fsForHIGH(inputLEFT, 1000, 1400);

		//front fs
		frontFuzzyIn[0] = fsForLOW(inputFRONT, 600, 1000);
		frontFuzzyIn[1] = fsForMEDIUM(inputFRONT, 600, 1000, 1400);
		frontFuzzyIn[2] = fsForHIGH(inputFRONT, 1000, 1400);

		//right fs
		rightFuzzyIn[0] = fsForLOW(inputRIGHT, 600, 1000);
		rightFuzzyIn[1] = fsForMEDIUM(inputRIGHT, 600, 1000, 1400);
		rightFuzzyIn[2] = fsForHIGH(inputRIGHT, 1000, 1400);


		/*
		 * /////////// INFERENCE ENGINE ////////////
		 */

		/* The obstacle avoidance rule base. */
		min_ = 0; //store the minimum fs in min

		// Rule 1. If inputs are (Low Low Low), the outputs are (Low Low)
		if (leftFuzzyIn[0] > 0 && frontFuzzyIn[0] > 0 && rightFuzzyIn[0] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[0], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[0]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 2. If inputs are (Low Medium Low), the outputs are (Low Low)
		if (leftFuzzyIn[0] > 0 && frontFuzzyIn[1] > 0 && rightFuzzyIn[0] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[0], frontFuzzyIn[1]);
			min_ = min(min_, rightFuzzyIn[0]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 3. If inputs are (Low High Low), the outputs are (Low Low)
		if (leftFuzzyIn[0] > 0 && frontFuzzyIn[2] > 0 && rightFuzzyIn[0] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[0], frontFuzzyIn[2]);
			min_ = min(min_, rightFuzzyIn[0]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 4. If inputs are (Medium Low Low), the outputs are (Low High)
		if (leftFuzzyIn[1] > 0 && frontFuzzyIn[0] > 0 && rightFuzzyIn[0] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[1], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[0]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[2] > min_ || rightFuzzyOut[2] == 0)
				rightFuzzyOut[2] = min_;
		}

		// Rule 5. If inputs are (High Low Low), the outputs are (Low High)
		if (leftFuzzyIn[2] > 0 && frontFuzzyIn[0] > 0 && rightFuzzyIn[0] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[2], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[0]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[2] > min_ || rightFuzzyOut[2] == 0)
				rightFuzzyOut[2] = min_;
		}

		// Rule 6. If inputs are (Medium Medium Low), the outputs are (Low High)
		if (leftFuzzyIn[1] > 0 && frontFuzzyIn[1] > 0 && rightFuzzyIn[0] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[1], frontFuzzyIn[1]);
			min_ = min(min_, rightFuzzyIn[0]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[2] > min_ || rightFuzzyOut[2] == 0)
				rightFuzzyOut[2] = min_;
		}

		// Rule 7. If inputs are (High Medium Low), the outputs are (Low High)
		if (leftFuzzyIn[2] > 0 && frontFuzzyIn[1] > 0 && rightFuzzyIn[0] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[2], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[0]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[2] > min_ || rightFuzzyOut[2] == 0)
				rightFuzzyOut[2] = min_;
		}

		// Rule 8. If inputs are (Medium High Low), the outputs are (Low High)
		if (leftFuzzyIn[1] > 0 && frontFuzzyIn[2] > 0 && rightFuzzyIn[0] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[1], frontFuzzyIn[2]);
			min_ = min(min_, rightFuzzyIn[0]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[2] > min_ || rightFuzzyOut[2] == 0)
				rightFuzzyOut[2] = min_;
		}

		// Rule 9. If inputs are (High High Low), the outputs are (Low High)
		if (leftFuzzyIn[2] > 0 && frontFuzzyIn[2] > 0 && rightFuzzyIn[0] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[2], frontFuzzyIn[2]);
			min_ = min(min_, rightFuzzyIn[0]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[2] > min_ || rightFuzzyOut[2] == 0)
				rightFuzzyOut[2] = min_;
		}

		// Rule 10. If inputs are (High Low Medium), the outputs are (Low High)
		if (leftFuzzyIn[2] > 0 && frontFuzzyIn[0] > 0 && rightFuzzyIn[1] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[2], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[1]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[0] > min_ || leftFuzzyOut[0] == 0)
				leftFuzzyOut[0] = min_;
			if (rightFuzzyOut[2] > min_ || rightFuzzyOut[2] == 0)
				rightFuzzyOut[2] = min_;
		}

		// Rule 11. If inputs are (High Low High), the outputs are (High Low)
		if (leftFuzzyIn[2] > 0 && frontFuzzyIn[0] > 0 && rightFuzzyIn[2] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[2], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[2]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[2] > min_ || leftFuzzyOut[2] == 0)
				leftFuzzyOut[2] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 12. If inputs are (Medium Low Medium), the outputs are (High Low)
		if (leftFuzzyIn[1] > 0 && frontFuzzyIn[0] > 0 && rightFuzzyIn[1] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[1], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[1]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[2] > min_ || leftFuzzyOut[2] == 0)
				leftFuzzyOut[2] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 13. If inputs are (Medium Low High), the outputs are (High Low)
		if (leftFuzzyIn[1] > 0 && frontFuzzyIn[0] > 0 && rightFuzzyIn[2] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[1], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[2]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[2] > min_ || leftFuzzyOut[2] == 0)
				leftFuzzyOut[2] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 14. If inputs are (Low Low Medium), the outputs are (High Low)
		if (leftFuzzyIn[0] > 0 && frontFuzzyIn[0] > 0 && rightFuzzyIn[1] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[0], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[1]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[2] > min_ || leftFuzzyOut[2] == 0)
				leftFuzzyOut[2] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 15. If inputs are (Low Low High), the outputs are (High Low)
		if (leftFuzzyIn[0] > 0 && frontFuzzyIn[0] > 0 && rightFuzzyIn[2] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[0], frontFuzzyIn[0]);
			min_ = min(min_, rightFuzzyIn[2]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[2] > min_ || leftFuzzyOut[2] == 0)
				leftFuzzyOut[2] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 16. If inputs are (Low Medium Medium), the outputs are (High Low)
		if (leftFuzzyIn[0] > 0 && frontFuzzyIn[1] > 0 && rightFuzzyIn[1] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[0], frontFuzzyIn[1]);
			min_ = min(min_, rightFuzzyIn[1]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[2] > min_ || leftFuzzyOut[2] == 0)
				leftFuzzyOut[2] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 17. If inputs are (Low Medium High), the outputs are (High Low)
		if (leftFuzzyIn[0] > 0 && frontFuzzyIn[1] > 0 && rightFuzzyIn[2] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[0], frontFuzzyIn[1]);
			min_ = min(min_, rightFuzzyIn[2]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[2] > min_ || leftFuzzyOut[2] == 0)
				leftFuzzyOut[2] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 18. If inputs are (Low High Medium), the outputs are (High Low)
		if (leftFuzzyIn[0] > 0 && frontFuzzyIn[2] > 0 && rightFuzzyIn[1] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[0], frontFuzzyIn[2]);
			min_ = min(min_, rightFuzzyIn[1]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[2] > min_ || leftFuzzyOut[2] == 0)
				leftFuzzyOut[2] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 19. If inputs are (Low High High), the outputs are (High Low)
		if (leftFuzzyIn[0] > 0 && frontFuzzyIn[2] > 0 && rightFuzzyIn[2] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[0], frontFuzzyIn[2]);
			min_ = min(min_, rightFuzzyIn[2]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[2] > min_ || leftFuzzyOut[2] == 0)
				leftFuzzyOut[2] = min_;
			if (rightFuzzyOut[0] > min_ || rightFuzzyOut[0] == 0)
				rightFuzzyOut[0] = min_;
		}

		// Rule 20. If inputs are (Medium Medium Medium), the outputs are (Medium Medium)
		if (leftFuzzyIn[1] > 0 && frontFuzzyIn[1] > 0 && rightFuzzyIn[1] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[1], frontFuzzyIn[1]);
			min_ = min(min_, rightFuzzyIn[1]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[1] > min_ || leftFuzzyOut[1] == 0)
				leftFuzzyOut[1] = min_;
			if (rightFuzzyOut[1] > min_ || rightFuzzyOut[1] == 0)
				rightFuzzyOut[1] = min_;
		}

		// Rule 21. If inputs are (Medium Medium High), the outputs are (Medium Medium)
		if (leftFuzzyIn[1] > 0 && frontFuzzyIn[1] > 0 && rightFuzzyIn[2] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[1], frontFuzzyIn[1]);
			min_ = min(min_, rightFuzzyIn[2]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[1] > min_ || leftFuzzyOut[1] == 0)
				leftFuzzyOut[1] = min_;
			if (rightFuzzyOut[1] > min_ || rightFuzzyOut[1] == 0)
				rightFuzzyOut[1] = min_;
		}

		// Rule 22. If inputs are (Medium High Medium), the outputs are (Medium Medium)
		if (leftFuzzyIn[1] > 0 && frontFuzzyIn[2] > 0 && rightFuzzyIn[1] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[1], frontFuzzyIn[2]);
			min_ = min(min_, rightFuzzyIn[1]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[1] > min_ || leftFuzzyOut[1] == 0)
				leftFuzzyOut[1] = min_;
			if (rightFuzzyOut[1] > min_ || rightFuzzyOut[1] == 0)
				rightFuzzyOut[1] = min_;
		}

		// Rule 23. If inputs are (High Medium Medium), the outputs are (Medium Medium)
		if (leftFuzzyIn[2] > 0 && frontFuzzyIn[1] > 0 && rightFuzzyIn[1] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[2], frontFuzzyIn[1]);
			min_ = min(min_, rightFuzzyIn[1]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[1] > min_ || leftFuzzyOut[1] == 0)
				leftFuzzyOut[1] = min_;
			if (rightFuzzyOut[1] > min_ || rightFuzzyOut[1] == 0)
				rightFuzzyOut[1] = min_;
		}

		// Rule 24. If inputs are (Medium High High), the outputs are (Medium Medium)
		if (leftFuzzyIn[1] > 0 && frontFuzzyIn[2] > 0 && rightFuzzyIn[2] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[1], frontFuzzyIn[2]);
			min_ = min(min_, rightFuzzyIn[2]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[1] > min_ || leftFuzzyOut[1] == 0)
				leftFuzzyOut[1] = min_;
			if (rightFuzzyOut[1] > min_ || rightFuzzyOut[1] == 0)
				rightFuzzyOut[1] = min_;
		}

		// Rule 25. If inputs are (High High Medium), the outputs are (Medium Medium)
		if (leftFuzzyIn[2] > 0 && frontFuzzyIn[2] > 0 && rightFuzzyIn[1] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[2], frontFuzzyIn[2]);
			min_ = min(min_, rightFuzzyIn[1]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[1] > min_ || leftFuzzyOut[1] == 0)
				leftFuzzyOut[1] = min_;
			if (rightFuzzyOut[1] > min_ || rightFuzzyOut[1] == 0)
				rightFuzzyOut[1] = min_;
		}

		// Rule 26. If inputs are (High Medium High), the outputs are (Medium Medium)
		if (leftFuzzyIn[2] > 0 && frontFuzzyIn[1] > 0 && rightFuzzyIn[2] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[2], frontFuzzyIn[1]);
			min_ = min(min_, rightFuzzyIn[2]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[1] > min_ || leftFuzzyOut[1] == 0)
				leftFuzzyOut[1] = min_;
			if (rightFuzzyOut[1] > min_ || rightFuzzyOut[1] == 0)
				rightFuzzyOut[1] = min_;
		}

		// Rule 27. If inputs are (High High High), the outputs are (Medium Medium)
		if (leftFuzzyIn[2] > 0 && frontFuzzyIn[2] > 0 && rightFuzzyIn[2] > 0)
		{
			//Find the minimum non-zero FS from all of the above mentioned
			min_ = min(leftFuzzyIn[2], frontFuzzyIn[2]);
			min_ = min(min_, rightFuzzyIn[2]);

			// Apply a minimum to the required by the rule fuzzy inputs, and assert it to the fuzzy output if it's less than the current fuzzy output
			if (leftFuzzyOut[1] > min_ || leftFuzzyOut[1] == 0)
				leftFuzzyOut[1] = min_;
			if (rightFuzzyOut[1] > min_ || rightFuzzyOut[1] == 0)
				rightFuzzyOut[1] = min_;
		}

		/*
		* /////////  DEFFUZIFICATION  //////////
		*/

		/* Array with output centers of membership functions
		*  outputMFcenters[0]  --  center of the LOW output MF (Membership Function)
		*  outputMFcenters[1]  --  center of the MEDIUM output MF
		*  outputMFcenters[2]  --  center of the HIGH output MF
		*/

		int leftOut, rightOut;


		/* The obstacle avoidance outputs */

		/* LEFT OUTPUT CALCULATION */
		/* Calculate the weighted average for the left motor*/
		a = leftFuzzyOut[0] * outputMFcenters[0];
		b = leftFuzzyOut[1] * outputMFcenters[1];
		c = leftFuzzyOut[2] * outputMFcenters[2];
		d = leftFuzzyOut[0] + leftFuzzyOut[1] + leftFuzzyOut[2];
		leftOut = (a + b + c) / d;


		/* RIGHT OUTPUT CALCULATION */
		/* Calculate the weighted average for the right motor*/

		a = rightFuzzyOut[0] * outputMFcenters[0];
		b = rightFuzzyOut[1] * outputMFcenters[1];
		c = rightFuzzyOut[2] * outputMFcenters[2];
		d = rightFuzzyOut[0] + rightFuzzyOut[1] + rightFuzzyOut[2];
		rightOut = (a + b + c) / d;

		//Set the motor speeds
		leftVel = leftOut;
		rightVel = rightOut;

		//Setup the speed
		robot.setVel2(leftVel, rightVel);

		ArUtil::sleep(0.5 * 1000); //wait for 0.5 second
	}
	// termination
	// stop the robot
	robot.lock();
	robot.stop();
	robot.unlock();

	// terminate all threads and exit
	Aria::exit();
}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////-----------HELPER FUNCTIONS-----------//////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

//Find 5000 readings and replace them to 0
void remove5000(int numOfElements, int array[])
{
	for (int i = 0; i < numOfElements; i++)
	{
		if (array[i] == 5000)
			array[i] = 0;
	}
}

int findAverageIgnoreZeros(int numOfElements, int array[])
{
	//avrg is initially 5000, because in case if all array elements are zeros, then it means they used to be 5000 initially,
	//so the average won't be changed and be returned as 5000
	int avrg = 5000;
	int sum = 0, amount = 0;
	for (int i = 0; i < numOfElements; i++)
	{
		if (array[i] != 0)
		{
			sum += array[i];
			amount++;
		}

	}
	if (amount != 0)
		avrg = sum / amount;
	return avrg;
}

//Remove all values from the supplied vector that are much higher or lower than the average
void removeHighLow(int numOfElements, int array[], int avrg)
{
	for (int i = 0; i < numOfElements; i++)
	{
		if (array[i] > avrg + 1500 || array[i] < avrg - 1500)
			array[i] = 0;
	}
}

/* Find the low FS for the input */
float fsForLOW(int input, int lastHigh, int lastPoint)
{
	float fs = 0;
	if (input <= lastHigh)
		fs = 1;
	else if (input > lastHigh && input < lastPoint)
		fs = (lastPoint - input) / (float)(lastPoint - lastHigh);
	else
		fs = 0;
	return fs;
}

/* Find the medium FS for the input */
float fsForMEDIUM(int input, int firstPoint, int midPoint, int lastPoint)
{
	float fs = 0;
	if (input <= firstPoint || input >= lastPoint)
		fs = 0;
	else if (input > firstPoint && input <= midPoint)
		fs = (input - firstPoint) / (float)(midPoint - firstPoint);
	else if (input > midPoint && input < lastPoint)
		fs = (lastPoint - input) / (float)(lastPoint - midPoint);
	return fs;
}

/* Find the high FS for the input */
float fsForHIGH(int input, int firstPoint, int firstHigh)
{
	float fs = 0;
	if (input <= firstPoint)
		fs = 0;
	else if (input > firstPoint && input < firstHigh)
		fs = (input - firstPoint) / (float)(firstHigh - firstPoint);
	else
		fs = 1;
	return fs;
}

float fsBlendEdgeFollowLOW(int input)
{
	float fs = 0;
	if (input <= 1000)
		fs = 1;
	else if (input > 1000 && input <= 5000)
		fs = (5000 - input) / (5000.0 - 1000.0);
	else
		fs = 0;
	return fs;
}

float fsBlendObstacleAvoidLOW(int input)
{
	float fs = 0;
	if (input <= 600)
		fs = 1;
	else if (input > 600 && input < 1400)
		fs = (1400 - input) / (1400.0 - 600.0);
	else
		fs = 0;
	return fs;
}
