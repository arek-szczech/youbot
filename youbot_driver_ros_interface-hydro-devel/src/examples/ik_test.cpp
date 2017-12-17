/******************************************************************************
* 2017
*
* Author:
* Arkadiusz Szczech
* Mateusz Talma
* Jakub Wawrzeńczak
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
* This program is based on Sebastian Blumenthal's examples in this package.
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

//Parameters [m]
double a1 = 33;
double d1 = 147;
double a2 = 155;
double a3 = 135;
double d5 = 218;

double RPY_3_1; //r31 from RPY matrix
double RPY_3_2; //r32 from RPY matrix
double RPY_3_3; //r33 from RPY matrix

double xp4; //x coordinate for point 4
double yp4; //y coordinate for point 4
double zp4; //z coordinate for point 4

double x4; //x from point 1 to point 4
double y4; //y from point 1 to point 4
double z4; //z from point 1 to point 4

double l; //distance form point 1 to point 4

//Additional variables to help calculate IK
double cos_phi;
double sin_phi;
double phi;
double beta;

double A;

double cos_B;
double sin_B;
double B;

//Final values
double theta_1;
double theta_2;
double theta_3;
double theta_4;
double theta_5;

//Offsets for youBot driver
double offset1 = 2.8668;
double offset2 = 2.59191;
double offset3 = -2.52113;
double offset4 = 1.75973;
double offset5 = 2.93141; 

//Signum function
int sgn(double v)
{
        if (v < 0) return -1;
        if (v >= 0) return 1;
}

void inverseKinematics(double xk, double yk, double zk, double Rz, double Ry, double Rx)
{
    //Calculating elements from RPY matrix
	RPY_3_1 = cos(Rz)*sin(Ry)*cos(Rx) + sin(Rz)*sin(Rx);
	RPY_3_2 = sin(Rz)*sin(Ry)*cos(Rx) - cos(Rz)*sin(Rx);
	RPY_3_3 = cos(Ry)*cos(Rx);

	xk = round(xk*100)/100;
	yk = round(yk*100)/100;
	zk = round(zk*100)/100;

	theta_1 = atan2(yk,xk);

	beta = atan2(RPY_3_3,sqrt(RPY_3_1*RPY_3_1 + RPY_3_2*RPY_3_2));

	xp4 = round((xk - d5*cos(theta_1)*cos(beta))*100)/100;
    yp4 = round((yk - d5*sin(theta_1)*cos(beta))*100)/100;
    zp4 = zk - d5*sin(beta);

    if (sgn(xp4)!=sgn(xk) || sgn(yp4)!=sgn(yk))
	{
		theta_1 = atan2(yp4,xp4);
   
   		x4 = xp4 - a1*cos(theta_1);
   		y4 = yp4 - a1*sin(theta_1);
   		z4 = zp4 - d1;

    	l = sqrt(x4*x4 + y4*y4 + z4*z4);
    
    	cos_phi = (a2*a2 + a3*a3 - l*l)/(2*a2*a3);
    	sin_phi = sqrt(1 - cos_phi*cos_phi);
    
    	phi = atan2(sin_phi,cos_phi);

    	theta_3 = (-1)*(M_PI - phi);
    	//theta_3 = (M_PI - phi);
    
    	A = atan2(z4,sqrt(x4*x4 + y4*y4));
    
   		cos_B = (a2*a2 + l*l - a3*a3)/(2*l*a2);
    	sin_B = sqrt(1 - cos_B*cos_B);
 
    	B = atan2(sin_B,cos_B);

    	if (theta_3 < 0)
    	{
   			theta_2 = (-1)*(A - B);
   		}
    	else if (theta_3 > 0)
    	{
    		theta_2 = (-1)*(A+B);
   		}
    	else if (theta_3 == 0 || theta_3 == M_PI)
    	{
    		theta_2 = (-1)*atan2(z4*z4,sqrt(x4*x4 + y4*y4));
   		}

    	theta_4 = (-1)*(beta + theta_2 + theta_3) - M_PI;

    	theta_5 = 0;

    	double th1 = theta_1;
    	double th2 = theta_2;
    	double th3 = theta_3;
        double th4 = theta_4 - M_PI/2;
   		double th5 = theta_5;

    	double x = a1*cos(th1) - d5*(cos(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3);
		double y = a1*sin(th1) - d5*(cos(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) + a2*cos(th2)*sin(th1) + a3*cos(th2)*cos(th3)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3);
		double z = d1 - a2*sin(th2) - d5*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))) - a3*cos(th2)*sin(th3) - a3*cos(th3)*sin(th2);

		//double roll = atan2(- cos(th1)*sin(th5) - cos(th5)*(cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2))), sin(th1)*sin(th5) - cos(th5)*(cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2))));
		//double pitch = atan2(cos(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sqrt(sin(th5)*sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)))*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))) + (cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))));
		//double yaw = atan2(sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) - cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)));

        x = round(x*1)/1;
        y = round(y*1)/1;
        z = round(z*1)/1;

        cout << "Wpisana wspolrzedna X: " << xk << endl;
        cout << "Obliczona wspolrzedna X: " << x << endl;
        cout << "Wpisana wspolrzedna Y: " << yk << endl;
        cout << "Obliczona wspolrzedna Y: " << y << endl;
        cout << "Wpisana wspolrzedna Z: " << zk << endl;
        cout << "Obliczona wspolrzedna Z: " << z << endl;

		if (xk == x && yk == y && zk == z)
		{
			cout << "Wyliczona z IK theta_1: " << theta_1 << endl;
			cout << "Wyliczona z IK theta_2: " << theta_2 << endl;
			cout << "Wyliczona z IK theta_3: " << theta_3 << endl;
			cout << "Wyliczona z IK theta_4: " << theta_4 << endl;
			cout << "Wyliczona z IK theta_5: " << theta_5 << endl;
			cout << "" << endl;

			cout << "Wysylana do robota theta_1: " << theta_1 + offset1 << endl;
    		cout << "Wysylana do robota theta_2: " << theta_2 + offset2 << endl;
    		cout << "Wysylana do robota theta_3: " << theta_3 + offset3 << endl;
    		cout << "Wysylana do robota theta_4: " << theta_4 + offset4 << endl;
			cout << "Wysylana do robota theta_5: " << theta_5 + offset5 << endl;
			cout << "" << endl;
		}
		else
		{
			cout << "BRAK ROZWIAZANIA!" << endl;
			cout << "" << endl;
		}
	}
	else if (sgn(xp4)==sgn(xk) || sgn(yp4)==sgn(yk))
	{
		theta_1 = atan2(yk,xk);
   
   		x4 = xp4 - a1*cos(theta_1);
   		y4 = yp4 - a1*sin(theta_1);
   		z4 = zp4 - d1;

    	l = sqrt(x4*x4 + y4*y4 + z4*z4);
    
    	cos_phi = (a2*a2 + a3*a3 - l*l)/(2*a2*a3);
    	sin_phi = sqrt(1 - cos_phi*cos_phi);
    
    	phi = atan2(sin_phi,cos_phi);

    	theta_3 = (-1)*(M_PI - phi);
    	//theta_3 = (M_PI - phi);
    
    	A = atan2(z4,sqrt(x4*x4 + y4*y4));
    
   		cos_B = (a2*a2 + l*l - a3*a3)/(2*l*a2);
   		sin_B = sqrt(1 - cos_B*cos_B);
 
    	B = atan2(sin_B,cos_B);

    	if (theta_3 < 0)
    	{
   			theta_2 = (-1)*(A - B);
   		}
    	else if (theta_3 > 0)
    	{
    		theta_2 = (-1)*(A+B);
   		}
    	else if (theta_3 == 0 || theta_3 == M_PI)
    	{
    		theta_2 = (-1)*atan2(z4*z4,sqrt(x4*x4 + y4*y4));
   		}

    	theta_4 = (-1)*(beta + theta_2 + theta_3);

    	theta_5 = 0;

    	double th1 = theta_1;
    	double th2 = theta_2;
    	double th3 = theta_3;
        double th4 = theta_4-M_PI/2;
   		double th5 = theta_5;

    	double x = a1*cos(th1) - d5*(cos(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3);
		double y = a1*sin(th1) - d5*(cos(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) + a2*cos(th2)*sin(th1) + a3*cos(th2)*cos(th3)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3);
		double z = d1 - a2*sin(th2) - d5*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))) - a3*cos(th2)*sin(th3) - a3*cos(th3)*sin(th2);

		//double roll = atan2(- cos(th1)*sin(th5) - cos(th5)*(cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2))), sin(th1)*sin(th5) - cos(th5)*(cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2))));
		//double pitch = atan2(cos(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sqrt(sin(th5)*sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)))*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))) + (cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))));
		//double yaw = atan2(sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) - cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)));

        x = round(x*1)/1;
        y = round(y*1)/1;
        z = round(z*1)/1;

        cout << "Wpisana wspolrzedna X: " << xk << endl;
        cout << "Obliczona wspolrzedna X: " << x << endl;
        cout << "Wpisana wspolrzedna Y: " << yk << endl;
        cout << "Obliczona wspolrzedna Y: " << y << endl;
        cout << "Wpisana wspolrzedna Z: " << zk << endl;
        cout << "Obliczona wspolrzedna Z: " << z << endl;

		if (xk == x && yk == y && zk == z)
		{
			cout << "" << endl;
			cout << "Wyliczona z IK theta_1: " << theta_1 << endl;
			cout << "Wyliczona z IK theta_2: " << theta_2 << endl;
			cout << "Wyliczona z IK theta_3: " << theta_3 << endl;
			cout << "Wyliczona z IK theta_4: " << theta_4 << endl;
			cout << "Wyliczona z IK theta_5: " << theta_5 << endl;
			cout << "" << endl;

			cout << "Wysylana do robota theta_1: " << theta_1 + offset1 << endl;
    		cout << "Wysylana do robota theta_2: " << theta_2 + offset2 << endl;
    		cout << "Wysylana do robota theta_3: " << theta_3 + offset3 << endl;
    		cout << "Wysylana do robota theta_4: " << theta_4 + offset4 << endl;
			cout << "Wysylana do robota theta_5: " << theta_5 + offset5 << endl;
			cout << "" << endl;
		}
		else
		{
			cout << "BRAK ROZWIAZANIA!" << endl;
		}
	}
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "ik_test");
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	ros::Publisher gripperPositionPublisher;

	armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

	ros::Rate rate(20); //Hz

	double xk; //x coordinate of end effector
	double yk; //y coordinate of end effector
	double zk; //z coordinate of end effector
	double Rz; //Roll from RPY
	double Ry; //Pitch from RPY
	double Rx; //Yaw from RPY

	static const int numberOfArmJoints = 5;
	static const int numberOfGripperJoints = 2;
	while (n.ok()) 
	{
		brics_actuator::JointPositions command;
		vector <brics_actuator::JointValue> armJointPositions;
		vector <brics_actuator::JointValue> gripperJointPositions;

		armJointPositions.resize(numberOfArmJoints);
		gripperJointPositions.resize(numberOfGripperJoints);

		std::stringstream jointName;

		cout << "Podaj pozycje x: ";
		cin >> xk;
		cout << "Podaj pozycje y: ";
		cin >> yk;
		cout << "Podaj pozycje z: ";
		cin >> zk;
		cout << "Podaj rotacje wokol osi z: ";
		cin >> Rz;
		cout << "Podaj rotacje wokol osi y: ";
		cin >> Ry;
		cout << "Podaj rotacje wokol osi x: ";
		cin >> Rx;
		cout << " " << endl;

		inverseKinematics(xk, yk, zk, Rz, Ry, Rx);

		armJointPositions[0].value = theta_1 + offset1;
		armJointPositions[1].value = theta_2 + offset2;
		armJointPositions[2].value = theta_3 + offset3;
		armJointPositions[3].value = theta_4 + offset4;
		armJointPositions[4].value = theta_5 + offset5; 

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
