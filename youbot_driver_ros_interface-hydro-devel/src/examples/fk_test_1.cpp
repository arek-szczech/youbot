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

#include "std_msgs/String.h"
#include <math.h>

using namespace std;

void chatterCallback(const brics_actuator::JointPositionsConstPtr& youbotArmCommand)
{
	// Wartosci odczytane z robota
	double th_1 = youbotArmCommand->positions[0].value;
	double th_2 = youbotArmCommand->positions[1].value;
	double th_3 = youbotArmCommand->positions[2].value;
	double th_4 = youbotArmCommand->positions[3].value;
	double th_5 = youbotArmCommand->positions[4].value;

	// Wartosci podstawiane do fk
	double th1 = th_1;// + 2.9496;
	double th2 = th_2;// - 2.7038;
	double th3 = th_3;// + 2.5468;
	double th4 = th_4;// - 3.3580;
	double th5 = th_5;// +	2.9234;

	cout << "Wartosci wysyłane do robota: " << endl;
	cout << "th_1: " << th_1 << endl;
	cout << "th_2: " << th_2 << endl;
	cout << "th_3: " << th_3 << endl;
	cout << "th_4: " << th_4 << endl;
	cout << "th_5: " << th_5 << endl;
	cout << "" << endl;
	cout << "Wartosci uzywane do liczenia fk: " << endl;
	cout << "th1: " << th1 << endl;
	cout << "th2: " << th2 << endl;
	cout << "th3: " << th3 << endl;
	cout << "th4: " << th4 << endl;
	cout << "th5: " << th5 << endl;
	cout << "" << endl;

	double cos_th1 = cos(th1);
	double cos_th2 = cos(th2);
	double cos_th3 = cos(th3);
	double cos_th4 = cos(th4);
	double cos_th5 = cos(th5);

	double sin_th1 = sin(th1);
	double sin_th2 = sin(th2);
	double sin_th3 = sin(th3);
	double sin_th4 = sin(th4);
	double sin_th5 = sin(th5);

	double a1 = 0.033;
	double d1 = 0.147;
	double a2 = 0.155;
	double a3 = 0.135;
	double d5 = 0.218;
	
	double x = a1*cos(th1) - d5*(cos(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) + sin(th5)*(cos(th1)*cos(th2)*cos(th3) - cos(th1)*sin(th2)*sin(th3))) + a2*cos(th1)*cos(th2) - a3*cos(th1)*sin(th2)*sin(th3) + a3*cos(th1)*cos(th2)*cos(th3);
	double y = a1*sin(th1) - d5*(cos(th5)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) + sin(th5)*(cos(th2)*cos(th3)*sin(th1) - sin(th1)*sin(th2)*sin(th3))) + a2*cos(th2)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3) + a3*cos(th2)*cos(th3)*sin(th1);
	double z = d1 - a2*sin(th2) - d5*(cos(th5)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th5)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))) - a3*cos(th2)*sin(th3) - a3*cos(th3)*sin(th2);

	cout << "Pozycja x: " << x << endl;
	cout << "Pozycja y: " << y << endl;
	cout << "Pozycja z: " << z << endl;
	cout << "----------" << endl;

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "fk_test");
	ros::NodeHandle n;
	ros::Subscriber myarmPositionsSubscriber;

	//ros::Publisher armPositionsPublisher;
	//ros::Publisher gripperPositionPublisher;

	myarmPositionsSubscriber = n.subscribe<brics_actuator::JointPositions >("arm_1/arm_controller/position_command", 1, chatterCallback);
	//armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
	//gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

/*  topicName.str("");
    topicName << youBotConfiguration.youBotArmConfigurations[armIndex].commandTopicName << "arm_controller/position_command"; // e.g. arm_1/arm_controller/positionCommand
    youBotConfiguration.youBotArmConfigurations[armIndex].armPositionCommandSubscriber = node.subscribe<brics_actuator::JointPositions > (topicName.str(), 1000, boost::bind(&YouBotOODLWrapper::armPositionsCommandCallback, this, _1, armIndex));*/

	//youbotArmCommand->positions[i].value

	/*
	ros::Rate rate(1); //Hz
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


		// ::io::base_unit_info <boost::units::si::angular_velocity>).name();
		for (int i = 0; i < numberOfArmJoints; ++i) {
			cout << "Please type in value for joint " << i + 1 << endl;
			cin >> readValue;

			jointName.str("");
			jointName << "arm_joint_" << (i + 1);

			armJointPositions[i].joint_uri = jointName.str();
			armJointPositions[i].value = readValue;

			armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
			cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;

		};


		cout << "sending command ..." << endl;

		command.positions = armJointPositions;
		armPositionsPublisher.publish(command);


		cout << "--------------------" << endl;
		//ros::spinOnce();
		//rate.sleep();
		

	}
	*/
	ros::spin();
	return 0;
}

/* EOF */
