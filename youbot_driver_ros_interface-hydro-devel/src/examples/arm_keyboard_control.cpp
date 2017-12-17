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

#include <signal.h>
#include <curses.h>


using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "arm_keyboard_control");
	ros::NodeHandle n;
	ros::Publisher armPositionsPublisher;
	ros::Publisher gripperPositionPublisher;

	armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

	ros::Rate rate(20); //Hz
	double readValue;
	static const int numberOfArmJoints = 5;
	static const int numberOfGripperJoints = 2;
	while (n.ok()) {
		brics_actuator::JointPositions command;
		vector <brics_actuator::JointValue> armJointPositions;
		vector <brics_actuator::JointValue> gripperJointPositions;

		armJointPositions.resize(numberOfArmJoints); //TODO:change that
		gripperJointPositions.resize(numberOfGripperJoints);

		std::stringstream jointName;
		   int ch = 0;
	static int zmienna = 0;
	(void) initscr(); /* initialize the curses library */
    keypad(stdscr, TRUE); /* enable keyboard mapping */

    (void) cbreak(); /* take input chars one at a time, no wait for \n */

	double min_1 = 0.0100692;
	double max_1 = 5.84014;
	static double joint_1 = 0.0100692;

	double min_2 = 0.0100692;  
	double max_2 = 2.61799;
	static double joint_2 = 0.0100692;

	double min_3 = -5.02655; 
	double max_3 = -0.015708; 
	static double joint_3 = -0.015708;

	double min_4 = 0.0221239;  
	double max_4 = 3.4292;
	static double joint_4 = 0.0221239;

	double min_5 = 0.110619;
	double max_5 = 5.64159;
	static double joint_5 = 0.110619;

	double min_6 = 0;
	double max_6 = 0.011;
	static double gripper_1 = 0;

	double min_7 = 0;
	double max_7 = 0.011;
	static double gripper_2 = 0;

	 	

	 	ch = getch();

	      switch (ch) 
	      {
	// joint_1
	        case 'q':
	if(joint_1 < max_1 - ((max_1 - min_1)/100))
	joint_1 = joint_1 + (max_1 - min_1)/100;
	          break;
	        case 'a':
	if(joint_1 > min_1 + ((max_1 - min_1)/100))
	joint_1 = joint_1 - (max_1 - min_1)/100;
	          break;

	// joint_2
	        case 'w':
	if(joint_2 < max_2 - ((max_2 - min_2)/100))
	joint_2 = joint_2 + (max_2 - min_2)/100;
	          break;
	        case 's':
	if(joint_2 > min_2 + ((max_2 - min_2)/100))
	joint_2 = joint_2 - (max_2 - min_2)/100;
	          break;          

	// joint_3
	        case 'e':
	if(joint_3 < max_3 - ((max_3 - min_3)/100))
	joint_3 = joint_3 + (max_3 - min_3)/100;
	          break;
	        case 'd':
	if(joint_3 > min_3 + ((max_3 - min_3)/100))
	joint_3 = joint_3 - (max_3 - min_3)/100;
	          break;          

	// joint_4
	        case 'r':
	if(joint_4 < max_4 - ((max_4 - min_4)/100))
	joint_4 = joint_4 + (max_4 - min_4)/100;
	          break;
	        case 'f':
	if(joint_4 > min_4 + ((max_4 - min_4)/100))
	joint_4 = joint_4 - (max_4 - min_4)/100;
	          break;          

	// joint_5
	        case 't':
	if(joint_5 < max_5 - ((max_5 - min_5)/100))
	joint_5 = joint_5 + (max_5 - min_5)/100;
	          break;
	        case 'g':
	if(joint_5 > min_5 + ((max_5 - min_5)/100))
	joint_5 = joint_5 - (max_5 - min_5)/100;
	          break;          

	// gripper_1
	        case 'y':
//	if(gripper_1 < max_6 - ((max_6 - min_6)/100))
	gripper_1 = gripper_1 + (max_6 - min_6)/10;
	          break;
	        case 'h':
//	if(gripper_1 > min_6 + ((max_1 - min_1)/100))
	gripper_1 = gripper_1 - (max_6 - min_6)/10;
	          break;  

	// gripper_2
	        case 'u':
	if(gripper_2 < max_7 - ((max_7 - min_7)/10))
	gripper_2 = gripper_2 + (max_7 - min_7)/10;
	          break;
	        case 'j':
	if(gripper_2 > min_7 + ((max_7 - min_7)/10))
	gripper_2 = gripper_2 - (max_7 - min_7)/10;
	          break;  

	        default:
	          break;
	      }
         refresh();
         endwin();

			armJointPositions[0].value = joint_1;
			armJointPositions[1].value = joint_2;
			armJointPositions[2].value = joint_3;
			armJointPositions[3].value = joint_4;
			armJointPositions[4].value = joint_5;

			for (int i = 0; i < numberOfArmJoints; ++i) {

			jointName.str("");
			jointName << "arm_joint_" << (i + 1);

			armJointPositions[i].joint_uri = jointName.str();

			armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;

		};

	cout<< "Joint 1: " << joint_1 << endl;
	cout<< "Joint 2: " << joint_2 << endl;
	cout<< "Joint 3: " << joint_3 << endl;
	cout<< "Joint 4: " << joint_4 << endl;
	cout<< "Joint 5: " << joint_5 << endl;
	cout << "sending command ..." << endl;


		command.positions = armJointPositions;
		armPositionsPublisher.publish(command);

	cout << "--------------------" << endl;
		
		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}

/* EOF */
