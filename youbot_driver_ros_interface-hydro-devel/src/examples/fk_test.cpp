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
#include "std_msgs/String.h"
#include <math.h>
#include "sensor_msgs/JointState.h"

using namespace std;

//Joints
double q1 = 0;
double q2 = 0;
double q3 = 0;
double q4 = 0;
double q5 = 0;

//Offsets [rad]
double offset1 = 2.9496;
double offset2 = 2.70519;
double offset3 = -2.54818;
double offset4 = 1.78896;
double offset5 = 2.92342;

//Parameters [m]
double a1 = 33;
double d1 = 147;
double a2 = 155;
double a3 = 135;
double d5 = 218;

void forwardKinematics(double q1, double q2,double q3,double q4,double q5)
{
    cout << "Wartosci wysyłane do robota: " << endl;
    cout << "q1: " << q1 << endl;
    cout << "q2: " << q2 << endl;
    cout << "q3: " << q3 << endl;
    cout << "q4: " << q4 << endl;
    cout << "q5: " << q5 << endl;
    cout << "" << endl;

    q1 = q1 - offset1;
    q2 = q2 - offset2;
    q3 = q3 - offset3;
    q4 = q4 - offset4 - M_PI/2;
    q5 = q5 - offset5;

    cout << "Wartosci uzywane do liczenia fk: " << endl;
    cout << "q1: " << q1 << endl;
    cout << "q2: " << q2 << endl;
    cout << "q3: " << q3 << endl;
    cout << "q4: " << q4 << endl;
    cout << "q5: " << q5 << endl;
    cout << "" << endl;

    double x = a1*cos(q1) - d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - sin(q4)*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3))) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3);
    double y = a1*sin(q1) - d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3);
    double z = d1 - a2*sin(q2) - d5*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - a3*cos(q2)*sin(q3) - a3*cos(q3)*sin(q2);
    double roll = atan2(- cos(q1)*sin(q5) - cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))), sin(q1)*sin(q5) - cos(q5)*(cos(q4)*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))));
    double pitch = atan2(cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))), sqrt(sin(q5)*sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + (cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))));
    double yaw = atan2(sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))), sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));

    cout << "Pozycja x: " << x << endl;
    cout << "Pozycja y: " << y << endl;
    cout << "Pozycja z: " << z << endl;
    cout << "Kąt roll: " << roll << endl;
    cout << "Kąt pitch: " << pitch << endl;
    cout << "Kąt yaw: " << yaw << endl;
    cout << "----------" << endl;
}

void jointsCallback(const sensor_msgs::JointStateConstPtr& youbotArmState)
{
    q1 = youbotArmState->position[0];
    q2 = youbotArmState->position[1];
    q3 = youbotArmState->position[2];
    q4 = youbotArmState->position[3];
    q5 = youbotArmState->position[4];
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "fk_test");
    ros::start();
    ros::NodeHandle n;
    ros::Subscriber jointsSubscriber;
    jointsSubscriber = n.subscribe<sensor_msgs::JointState >("/joint_states", 1, jointsCallback);

    //start();
    ros::Rate loop_rate(0.2);
    while ( ros::ok() )
    {
        forwardKinematics(q1,q2,q3,q4,q5);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
