#include <Aria.h>
#include <stdio.h>
#include <iostream>
#include <conio.h>

using namespace std;

struct membership_fn //a structure to define all the membership functions
{
	double x1, x2, x3, x4, centroid;
	void make_shape(double a, double b, double c, double d)
	{
		x1 = a;	x2 = b;	x3 = c;	x4 = d;  //assigning the x values to the shapes
		centroid = (x1 + x2 + x3 + x4) / double(4); //calculating the centroid of the shapes
	}
};

double get_membership_value(membership_fn func, double input) //a function to get the membership values of the supplied inputs
{
	double output;
	if ((input > func.x1) && (input < func.x2))
		output = (input - func.x1) / (func.x2 - func.x1); //rising edge
	else if ((input >= func.x2) && (input <= func.x3))
		output = 1;
	else if ((input > func.x3) && (input < func.x4))
		output = (func.x4 - input) / (func.x4 - func.x3); //falling edge
	else
		output = 0.0001;

	return output;
}

struct rule_REF //a structure to define the rules in the rule base of Right Edge following
{
	double lms, rms;
	double value;
	void define_rule(double input1val, double input2val, double centroid_lms, double centroid_rms)
	{
		value = min(input1val, input2val); //taking the minimum to find the value correspoding to a  particular rule
		lms = centroid_lms; //lms output for that rule
		rms = centroid_rms; //rms output for that rule
	}
};

struct rule_OA //a structure to define the rules for Obstacle Avoidance
{
	double lms, rms;
	double value;
	void define_rule(double input1val, double input2val, double input3val, double centroid_lms, double centroid_rms)
	{
		value = min(input1val, input2val, input3val);
		lms = centroid_lms;
		rms = centroid_rms;
	}
};

int main(int argc, char **argv)
{
	//Defining Variables for Right Edge Following (REF)
	double REFinput1, REFinput2, REF_lms, REF_rms;
	double REFin1low, REFin1med, REFin1high, REFin2low, REFin2med, REFin2high;
	double REFlms_numerator, REFlms_denominator, REFrms_numerator, REFrms_denominator;
	int i, j;
	const int REF_no_of_rules = 9;
	rule_REF REFrulebase[REF_no_of_rules];

	//Defining Variables for Obstacle Avoidance (OA)
	double OAinput1, OAinput2, OAinput3, OA_lms, OA_rms;
	double OAin1low, OAin1med, OAin1high, OAin2low, OAin2med, OAin2high, OAin3low, OAin3med, OAin3high;
	double OAlms_numerator, OAlms_denominator, OArms_numerator, OArms_denominator;
	const int OA_no_of_rules = 27;
	rule_OA OArulebase[OA_no_of_rules];

	//Input membership functions for Right Edge Following
	membership_fn low_in, medium_in, high_in;
	low_in.make_shape(0, 0, 600, 700);
	medium_in.make_shape(600, 700, 750, 1000);
	high_in.make_shape(750, 1000, 5000, 5000);

	//Input membership functions for Obstacle Avoidance
	membership_fn low_inOA, medium_inOA, high_inOA;
	low_inOA.make_shape(0, 0, 700, 900);
	medium_inOA.make_shape(700, 900, 1000, 1250);
	high_inOA.make_shape(1000, 1250, 5000, 5000);
	
	//Output membership functions 
	membership_fn low_out, medium_out, high_out;
	low_out.make_shape(0, 20, 20, 40);
	medium_out.make_shape(80, 150, 150, 220);
	high_out.make_shape(150, 220, 220, 290);

	//=========Fuzzy Control Architecture (Initialization)===========//
	membership_fn control_OA;
	control_OA.make_shape(0, 0, 600, 700);
	membership_fn control_REF;
	control_REF.make_shape(600, 700, 5000, 5000);
	double min_sens, control_inOA, control_inREF;
	double final_lms, final_rms;

	//============== Initialisation of Robot =============//
	Aria::init();
	ArRobot robot;
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		cout << "Robot Connected!" << endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();
	ArSensorReading *sonarSensor[8];
	int sonarRange[8];

	while (true)
	{
		//========Getting Sonar Readings=========//
		for (i = 0; i < 8; i++) {
			sonarSensor[i] = robot.getSonarReading(i);
			sonarRange[i] = sonarSensor[i]->getRange();
		}
		//=========Right Edge Following (Calculations)============//
		REFinput1 = min(sonarRange[6], sonarRange[5], sonarRange[4]);
		REFinput2 = sonarRange[7];

		REFin1low = get_membership_value(low_in, REFinput1);
		REFin1med = get_membership_value(medium_in, REFinput1);
		REFin1high = get_membership_value(high_in, REFinput1);

		REFin2low = get_membership_value(low_in, REFinput2);
		REFin2med = get_membership_value(medium_in, REFinput2);
		REFin2high = get_membership_value(high_in, REFinput2);

		//Defining Rule Base
		REFrulebase[0].define_rule(REFin1low, REFin2low, low_out.centroid, high_out.centroid);
		REFrulebase[1].define_rule(REFin1low, REFin2med, low_out.centroid, high_out.centroid);
		REFrulebase[2].define_rule(REFin1low, REFin2high, low_out.centroid, high_out.centroid);
		REFrulebase[3].define_rule(REFin1med, REFin2low, low_out.centroid, high_out.centroid);
		REFrulebase[4].define_rule(REFin1med, REFin2med, medium_out.centroid, medium_out.centroid);
		REFrulebase[5].define_rule(REFin1med, REFin2high, low_out.centroid, medium_out.centroid);
		REFrulebase[6].define_rule(REFin1high, REFin2low, high_out.centroid, low_out.centroid);
		REFrulebase[7].define_rule(REFin1high, REFin2med, high_out.centroid, low_out.centroid);
		REFrulebase[8].define_rule(REFin1high, REFin2high, high_out.centroid, low_out.centroid);

		REFlms_numerator = double(0);
		REFlms_denominator = double(0);
		REFrms_numerator = double(0);
		REFrms_denominator = double(0);

		for (i = 0;i<9;i++) //calculating the output values
		{
			REFlms_numerator += (REFrulebase[i].value)*(REFrulebase[i].lms);
			REFlms_denominator += REFrulebase[i].value;
			REFrms_numerator += (REFrulebase[i].value)*(REFrulebase[i].rms);
			REFrms_denominator += REFrulebase[i].value;
		}

		REF_lms = REFlms_numerator / REFlms_denominator;
		REF_rms = REFrms_numerator / REFrms_denominator;

		//=========Obstacle Avoidance (Calculations)============//
		OAinput1 = sonarRange[2];
		OAinput2 = min(sonarRange[3], sonarRange[4]);
		OAinput3 = sonarRange[5];

		OAin1low = get_membership_value(low_inOA, OAinput1);
		OAin1med = get_membership_value(medium_inOA, OAinput1);
		OAin1high = get_membership_value(high_inOA, OAinput1);

		OAin2low = get_membership_value(low_inOA, OAinput2);
		OAin2med = get_membership_value(medium_inOA, OAinput2);
		OAin2high = get_membership_value(high_inOA, OAinput2);

		OAin3low = get_membership_value(low_inOA, OAinput3);
		OAin3med = get_membership_value(medium_inOA, OAinput3);
		OAin3high = get_membership_value(high_inOA, OAinput3);

		//Defining Rule Base
		OArulebase[0].define_rule(OAin1low, OAin2low, OAin3low, low_out.centroid, high_out.centroid);
		OArulebase[1].define_rule(OAin1low, OAin2low, OAin3med, low_out.centroid, high_out.centroid);
		OArulebase[2].define_rule(OAin1low, OAin2low, OAin3high, low_out.centroid, high_out.centroid);
		OArulebase[3].define_rule(OAin1low, OAin2med, OAin3low, high_out.centroid, low_out.centroid);
		OArulebase[4].define_rule(OAin1low, OAin2med, OAin3med, high_out.centroid, low_out.centroid);
		OArulebase[5].define_rule(OAin1low, OAin2med, OAin3high, high_out.centroid, low_out.centroid);
		OArulebase[6].define_rule(OAin1low, OAin2high, OAin3low, low_out.centroid, high_out.centroid);
		OArulebase[7].define_rule(OAin1low, OAin2high, OAin3med, medium_out.centroid, low_out.centroid);
		OArulebase[8].define_rule(OAin1low, OAin2high, OAin3high, high_out.centroid, low_out.centroid);
		OArulebase[9].define_rule(OAin1med, OAin2low, OAin3low, low_out.centroid, high_out.centroid);
		OArulebase[10].define_rule(OAin1med, OAin2low, OAin3med, medium_out.centroid, high_out.centroid);
		OArulebase[11].define_rule(OAin1med, OAin2low, OAin3high, high_out.centroid, low_out.centroid);
		OArulebase[12].define_rule(OAin1med, OAin2med, OAin3low, low_out.centroid, high_out.centroid);
		OArulebase[13].define_rule(OAin1med, OAin2med, OAin3med, medium_out.centroid, high_out.centroid);
		OArulebase[14].define_rule(OAin1med, OAin2med, OAin3high, high_out.centroid, low_out.centroid);
		OArulebase[15].define_rule(OAin1med, OAin2high, OAin3low, low_out.centroid, high_out.centroid);
		OArulebase[16].define_rule(OAin1med, OAin2high, OAin3med, medium_out.centroid, high_out.centroid);
		OArulebase[17].define_rule(OAin1med, OAin2high, OAin3high, high_out.centroid, low_out.centroid);
		OArulebase[18].define_rule(OAin1high, OAin2low, OAin3low, low_out.centroid, high_out.centroid);
		OArulebase[19].define_rule(OAin1high, OAin2low, OAin3med, low_out.centroid, high_out.centroid);
		OArulebase[20].define_rule(OAin1high, OAin2low, OAin3high, low_out.centroid, high_out.centroid);
		OArulebase[21].define_rule(OAin1high, OAin2med, OAin3low, low_out.centroid, high_out.centroid);
		OArulebase[22].define_rule(OAin1high, OAin2med, OAin3med, low_out.centroid, high_out.centroid);
		OArulebase[23].define_rule(OAin1high, OAin2med, OAin3high, low_out.centroid, high_out.centroid);
		OArulebase[24].define_rule(OAin1high, OAin2high, OAin3low, low_out.centroid, high_out.centroid);
		OArulebase[25].define_rule(OAin1high, OAin2high, OAin3med, medium_out.centroid, high_out.centroid);
		OArulebase[26].define_rule(OAin1high, OAin2high, OAin3high, medium_out.centroid, medium_out.centroid);

		OAlms_numerator = double(0);
		OAlms_denominator = double(0);
		OArms_numerator = double(0);
		OArms_denominator = double(0);

		for (i = 0;i<OA_no_of_rules;i++) //calculating the output values
		{
			OAlms_numerator += (OArulebase[i].value)*(OArulebase[i].lms);
			OAlms_denominator += OArulebase[i].value;
			OArms_numerator += (OArulebase[i].value)*(OArulebase[i].rms);
			OArms_denominator += OArulebase[i].value;
		}

		OA_lms = OAlms_numerator / OAlms_denominator;
		OA_rms = OArms_numerator / OArms_denominator;

		//======Context Blending=======//
		/*
		min_sens = min(sonarRange[3], sonarRange[4], sonarRange[5]);
		control_inOA = get_membership_value(control_OA, min_sens);
		control_inREF = get_membership_value(control_REF, min_sens);
		final_lms = ((control_inOA*OA_lms) + (control_inREF*REF_lms)) / (control_inOA + control_inREF);
		final_rms = ((control_inOA*OA_rms) + (control_inREF*REF_rms)) / (control_inOA + control_inREF);
		robot.setVel2(final_lms, final_rms);
		*/

		//=======Subsumption ============//
		/*
		min_sens = min(sonarRange[3], sonarRange[4], sonarRange[5]);
		if (min_sens<600)
			robot.setVel2(OA_lms, OA_rms);
		else
			robot.setVel2(REF_lms, REF_rms);
		*/
		//========Only Obstacle Avoidance==========//
		//robot.setVel2(OA_lms, OA_rms);
		//=========================================//

		//========Only Right edge following=========//
		robot.setVel2(REF_lms, REF_rms);
		//==========================================//
	}


	// termination (SLIDE 47)
	// stop the robot
	robot.lock();
	robot.stop();
	robot.unlock();
	// terminate all threads and exit
	Aria::exit();
	return 0;
}