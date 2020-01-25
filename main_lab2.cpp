#include "Aria.h"
#include <stdio.h>
#include <iostream>
#include <conio.h>
#include <math.h>

using namespace std;

int main(int argc, char** argv)
{
	// =========Initialization of Robot===========//
	
	Aria::init();
	ArRobot robot;
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobotConnector
		robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		std::cout << "Robot connected!" << std::endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	//=====Defining the variables==========//
	
	int act_dis, cur_dis; //Actual distance and current distance of the robot from the wall
	int sense7, sense6, sens6_hor; //Variables to store the sensor 7 reading, sensor 6 and horizontal component of sensor 6 reading
	act_dis = 500; //Actual distance to be maintained between the right edge and the wall
	double kp, kd, ki, e, base_speed; //Parameters of the PID controller
	double ei, ed, eprev, output; 
	ei = 0; //Initializing the ei values
	eprev = 0;
	double rms, lms; //Left and right motor speeds
	double left, right;
	int count = 0;
	
	//=====Parameters of the PID controller======//
	
	kp = 0.6;  
	ki = 0.005;
	kd = 0.02;
	
	while (true)
	{
		//=========Getting Sonar Readings=============//
		ArSensorReading *sonarSensor[8];
		int sonarRange[8];
		for (int i = 0; i < 8; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}
		sense7 = sonarRange[7]; //Sensor 7 readings
		sense6 = sonarRange[6]; //Sensor 6 readings
		sens6_hor = sense6*cos(0.698); //Horizontal component of sensor 6 reading

		if ((sonarRange[6] <= 3000) || (sonarRange[7] <= 3000)) //To ignore readings when the robot is very far
		{
			cur_dis = min(sens6_hor, sense7); //Calculating the current distance of separation
		}

		e = act_dis - cur_dis; //The error in each step
		if (count<5) //To control the rapid increase in ei values , this condition makes ei values increase only every five steps
		{
			ei += e;
		}
		else
		{
			ei = 0;
			count = -1;
		}
		ed = e - eprev; //storing the ed value
		eprev = e; //storing the previous error
		output = (kp*e) + (ki*ei) + (kd*ed); //Getting the PID output
		count++;
		base_speed = 150; //Setting the base speed for the wheels
		left = 0; right = 0; //Initializing both the left and right output
		left = output; //Assigning the PID output to the left wheel (by trial and error)
		lms = base_speed - left; //final left motor speed
		rms = base_speed; //final right motor speed
		robot.setVel2(lms, rms);

		cout << "Inputs: "<<sense7 << " " << sense6 << " " << sens6_hor << " " << endl;
		cout << "Outputs: "<< lms << " " << rms << " " << endl;
		ArUtil::sleep(100);

	}
	robot.lock();
	robot.stop();
	robot.unlock();
	// terminate all threads and exit
	Aria::exit();
	return 0;

}
