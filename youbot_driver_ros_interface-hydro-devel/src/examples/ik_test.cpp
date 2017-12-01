/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Sebastian Blumenthal
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Locomotec nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <math.h>

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "ik_test");
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	ros::Publisher gripperPositionPublisher;

	armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

	ros::Rate rate(20); //Hz

	double a1 = 033;
	double d1 = 147;
	double a2 = 155;
	double a3 = 135;
	double d5 = 218;

	double x; //wspolrzedna x koncowki manipulatora
	double y; //wspolrzedna y koncowki manipulatora
	double z; //wspolrzedna z koncowki manipulatora
	double Rz; //kat R wg RPY
	double Ry; //kat P wg RPY
	double Rx; //kat Y wg RPY

	double RPY_3_1; //element r31 z macierzy RPY
	double RPY_3_2; //element r32 z macierzy RPY
	double RPY_3_3; //element r33 z macierzy RPY

	double xp4; //wspolrzedna x pkt4
	double yp4; //wspolrzedna y pkt4
	double zp4; //wspolrzedna z pkt4

	double l; //odleglosc od pkt2 do pk4

	//zmienne pomocnicze
	double cos_phi;
	double sin_phi;
	double phi;
	double beta;

	double A;

	double cos_B;
	double sin_B;
	double B;

	double theta_1;
	double theta_2;
	double theta_3;
	double theta_4;
	double theta_5;

//offsety od svena
/*	double offset1 = 2.9496;
    double offset2 = 1.1344;
    double offset3 = -2.5481;
    double offset4 = 1.7990;
    double offset5 = 2.9234; */

//offsety z pozycji folded_v2
   	double offset1 = -2.9395308;
	double offset2 = 2.7138692;
	double offset3 = -2.562508;
	double offset4 = 1.8093239;
	double offset5 = -2.812781;

//offsety z pozycji folded_v1
/*	double offset1 = 0.2020612;
	double offset2 = 0.009639505;
	double offset3 = 2.434182;
	double offset4 = -1.2206461;
	double offset5 = 0.110619; */


	//double xk;
	//double yk;
	//double zk;

	static const int numberOfArmJoints = 5;
	static const int numberOfGripperJoints = 2;
	while (n.ok()) {
		brics_actuator::JointPositions command;
		vector <brics_actuator::JointValue> armJointPositions;
		vector <brics_actuator::JointValue> gripperJointPositions;

		armJointPositions.resize(numberOfArmJoints); //TODO:change that
		gripperJointPositions.resize(numberOfGripperJoints);

		std::stringstream jointName;

	cout << "Podaj pozycje x: ";
	cin >> x;
	cout << "Podaj pozycje y: ";
	cin >> y;
	cout << "Podaj pozycje z: ";
	cin >> z;
	cout << "Podaj rotacje wokol osi z: ";
	cin >> Rz;
	cout << "Podaj rotacje wokol osi y: ";
	cin >> Ry;
	cout << "Podaj rotacje wokol osi x: ";
	cin >> Rx;

		cout << " " << endl;
		cout << "Liczenie kinematyki odwrotnej: " << endl;

	//Obliczanie macierzy RPY
	RPY_3_1 = cos(Rz)*sin(Ry)*cos(Rx) + sin(Rz)*sin(Rx);
		cout << "RPY_3_1: " <<  RPY_3_1 << endl;	
	RPY_3_2 = sin(Rz)*sin(Ry)*cos(Rx) - cos(Rz)*sin(Rx);
		cout << "RPY_3_2: " <<  RPY_3_2 << endl;
	RPY_3_3 = cos(Ry)*cos(Rx);
		cout << "RPY_3_3: " <<  RPY_3_3 << endl;

	// Odwrotna
	theta_1 = atan2(y,x);
		cout << "theta_1: " <<  theta_1 << endl;

	beta = atan2(RPY_3_3,sqrt(RPY_3_1*RPY_3_1 + RPY_3_2*RPY_3_2));
		cout << "beta: " <<  beta << endl;

	xp4 = x - a1*cos(theta_1) - d5*cos(theta_1)*cos(beta);
		cout << "xp4: " <<  xp4 << endl;
	yp4 = y - a1*sin(theta_1) - d5*sin(theta_1)*cos(beta);
		cout << "yp4: " <<  yp4 << endl;
	zp4 = z - d1 - d5*sin(beta);
		cout << "zp4: " <<  zp4 << endl;

	l = sqrt(xp4*xp4 + yp4*yp4 + zp4*zp4);
		cout << "l: " <<  l << endl;

	cos_phi = (a2*a2 + a3*a3 - l*l)/(2*a2*a3);
		cout << "cos_phi: " <<  cos_phi << endl;
	sin_phi = sqrt(1 - cos_phi*cos_phi);
		cout << "sin_phi: " <<  sin_phi << endl;

	phi = atan2(sin_phi,cos_phi);
		cout << "phi: " <<  phi << endl;
	if (Rx == 0)
		theta_3 = (-1)*(M_PI - phi);
	else if (Rx == M_PI)
		theta_3 = M_PI - phi;
//
	A = atan2(zp4,sqrt(xp4*xp4 + yp4*yp4));
		cout << "A: " <<  A << endl;

	cos_B = (a2*a2 + l*l - a3*a3)/(2*l*a2);
		cout << "cos_B: " <<  cos_B << endl;	
	sin_B = sqrt(1-cos_B*cos_B);
		cout << "sin_B: " <<  sin_B << endl;
	B = atan2(sin_B,cos_B);
		cout << "B: " <<  B << endl;

	if (theta_3 < 0)
    	theta_2 = (-1)*(A - B);
	else if (theta_3 > 0)
    	theta_2 = (-1)*(A + B);
	else if (theta_3 == 0)
    	theta_2 = atan2(zp4,sqrt(xp4*xp4 + yp4*yp4));
	else if (theta_3 == (-1)*M_PI)
    	theta_2 = atan2(zp4,sqrt(xp4*xp4 + yp4*yp4));

    theta_4 = (-1)*(beta + theta_2 + theta_3);

    theta_5 = 0;

    cout << "" << endl;
	cout << "Wyliczona z IK theta_1: " << theta_1 << endl;
	cout << "Wyliczona z IK theta_2: " << theta_2 << endl;
	cout << "Wyliczona z IK theta_3: " << theta_3 << endl;
	cout << "Wyliczona z IK theta_4: " << theta_4 << endl;
	cout << "Wyliczona z IK theta_5: " << theta_5 << endl;

/*	armJointPositions[0].value = theta_1;
	armJointPositions[1].value = theta_2;
	armJointPositions[2].value = theta_3;
	armJointPositions[3].value = theta_4;
	armJointPositions[4].value = theta_5; */

	armJointPositions[0].value = theta_1 + offset1;
	armJointPositions[1].value = theta_2 + offset2;
	armJointPositions[2].value = theta_3 + offset3;
	armJointPositions[3].value = theta_4 + offset4;
	armJointPositions[4].value = theta_5 + offset5; 

    cout << "" << endl;
    cout << "Wysylana do robota theta_1: " << armJointPositions[0].value << endl;
    cout << "Wysylana do robota theta_2: " << armJointPositions[1].value << endl;
    cout << "Wysylana do robota theta_3: " << armJointPositions[2].value << endl;
    cout << "Wysylana do robota theta_4: " << armJointPositions[3].value << endl;
	cout << "Wysylana do robota theta_5: " << armJointPositions[4].value << endl;
	cout << "" << endl;

		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();
		for (int i = 0; i < numberOfArmJoints; ++i) {
		
			jointName.str("");
			jointName << "arm_joint_" << (i + 1);
			armJointPositions[i].joint_uri = jointName.str();
			armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;
		};

//		cout << "Please type in value for a left jaw of the gripper " << endl;
//		cin >> readValue;
//		gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
//		gripperJointPositions[0].value = readValue;
//		gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

//		cout << "Please type in value for a right jaw of the gripper " << endl;
//		cin >> readValue;
//		gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
//		gripperJointPositions[1].value = readValue;
//		gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

		cout << "sending command ..." << endl;

		command.positions = armJointPositions;
		armPositionsPublisher.publish(command);

//		command.positions = gripperJointPositions;
//		gripperPositionPublisher.publish(command);

		cout << "--------------------" << endl;
		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}

/* EOF */
