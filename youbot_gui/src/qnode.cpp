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
******************************************************************************/

#include "../include/youbot_gui/main_window.hpp" //Kolejność includów jest krytyczna
#include <ros/ros.h>
#include <ros/network.h>
#include <string.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Core>
#include <assert.h>
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"
#include <boost/units/io.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <signal.h>
#include <curses.h>
#include <fstream>
#include <stdlib.h>
#include <cstdlib>
#include <vector>
#include <unistd.h>
#include "brics_actuator/ProgramExecuteVelocity.h"

#include "../include/youbot_gui/qnode.hpp"
#include "../youbot_driver-hydro-devel/include/youbot_driver/youbot/YouBotBase.hpp"
#include "../youbot_driver-hydro-devel/include/youbot_driver/youbot/YouBotManipulator.hpp"
#include "../youbot_driver_ros_interface-hydro-devel/include/youbot_driver_ros_interface/YouBotOODLWrapper.h"

using namespace std;
using namespace youbot;
using namespace Eigen;
namespace youbot_gui {

//jointsCallback
double QNode::subscriber_joint1;
double QNode::subscriber_joint2;
double QNode::subscriber_joint3;
double QNode::subscriber_joint4;
double QNode::subscriber_joint5;
double QNode::subscriber_gripper_1;
double QNode::subscriber_gripper_2;

//forward_kinematic
double QNode::x;
double QNode::y;
double QNode::z;
double QNode::roll;
double QNode::pitch;
double QNode::yaw;

int QNode::program_state;
int QNode::program_line_number;
int QNode::point[100];
std::string QNode::command[100];
bool QNode::back_to_home;

bool QNode::execute_movement_flag=true;
int QNode::movement_iteration=0;
bool QNode::play_program=false;
bool QNode::ethercat_connection=false;
bool QNode::ethercat_connection_temp=false;
bool QNode::ethercat_connection_temp2=false;
bool QNode::elbow_state=false; //false dół, true góra

bool QNode::linear_solution_exist=true;

bool QNode::opening_gripper=false;
bool QNode::closing_gripper=false;

double QNode::q1_lin_check;
double QNode::q2_lin_check;
double QNode::q3_lin_check;
double QNode::q4_lin_check;
double QNode::q5_lin_check;

int QNode::greatest_value=0;
bool QNode::executed_little_step=false;
double QNode::actual_little_step_position[5];
double QNode::prev_x;
double QNode::prev_y;
double QNode::prev_z;
double QNode::prev_roll;
double QNode::prev_pitch;
double QNode::prev_yaw;
double QNode::distance_x;
double QNode::distance_y;
double QNode::distance_z;
bool QNode::start_lin_mov=false;

int QNode::velocity[100];
bool QNode::isManualPTPActiv=false;
bool QNode::points_list_view_mode=false;

//*********Zmienne do funkcji executeProgram****************
string line[100];
int line_nmb=0;
int state=0;
double q1[100];
double q2[100];
double q3[100];
double q4[100];
double q5[100];
double P[100][5];
double QNode::home[5];

size_t pos1,pos2,pos3,pos4,pos5,pos6;
fstream points;
//**********************************************************

//youBot parameters
double a1 = 33;
double d1 = 147;
double a2 = 155;
double a3 = 135;
double d5 = 218;

//Offsets for youBot driver
//stare
//double offset1 = 2.8668;
//double offset2 = 2.59191;
//double offset3 = -2.52113;
//double offset4 = 1.75973;
//double offset5 = 2.93141;

//double offset1 = 2.91503;
//double offset2 = 2.61799;
//double offset3 = -2.50542;
//double offset4 = 1.70353;
//double offset5 = 2.76548;

//22.01 stare
//double offset1 = 2.9496;
//double offset2 = 2.616;
//double offset3 = -2.54818;
//double offset4 = 1.78896;
//double offset5 = 2.92342;

//22.01 nowe
double offset1 = 2.9496;
double offset2 = 2.70519;
double offset3 = -2.54818;
double offset4 = 1.78896;
double offset5 = 2.92342;

//full working ik
double* QNode::inverseKinematicJacobi(double xk, double yk, double zk, double Rz, double Ry, double Rx, bool logi, bool check)
{
    static double q[5];

    double jlimit = 1000; //limit value for iterations
    double tolerancja = 1e-10; //tollerance of deviation
    double blad = 1; //error value
    double count = 0; //iteration counter

    linear_solution_exist=true;

    double theta_1;
    double theta_2;
    double theta_3;
    double theta_4;
    double theta_5;

    //actual position from robot
    if(check)
    {
         theta_1=q1_lin_check;
         theta_2=q2_lin_check;
         theta_3=q3_lin_check;
         theta_4=q4_lin_check;
         theta_5=q5_lin_check;

    }
    else
    {
         theta_1 = QNode::subscriber_joint1;
         theta_2 = QNode::subscriber_joint2;
         theta_3 = QNode::subscriber_joint3;
         theta_4 = QNode::subscriber_joint4;
         theta_5 = QNode::subscriber_joint5;
    }

    //cout<<"theta_1: "<<theta_1<<endl;
    //cout<<"theta_2: "<<theta_2<<endl;
    //cout<<"theta_3: "<<theta_3<<endl;
    //cout<<"theta_4: "<<theta_4<<endl;
    //cout<<"theta_5: "<<theta_5<<endl;


    //actual position from robot after offsets and -pi/2 on 4th deegre of freedom
    theta_1 = theta_1 - offset1;
    theta_2 = theta_2 - offset2;
    theta_3 = theta_3 - offset3;
    theta_4 = theta_4 - offset4 - M_PI/2;
    theta_5 = theta_5 - offset5;


    //cout<<"offtheta_1: "<<theta_1<<endl;
    //cout<<"offtheta_2: "<<theta_2<<endl;
    //cout<<"offtheta_3: "<<theta_3<<endl;
    //cout<<"offtheta_4: "<<theta_4<<endl;
    //cout<<"offtheta_5: "<<theta_5<<endl;



//    double theta_1 = 0; 2.9496
//    double theta_2 = -1.57;1.046
//    double theta_3 = -0.78;-3.32818
//    double theta_4 = -2.61;0.74896
//    double theta_5 = 0;2.9342

    MatrixXd t_input(4,4); //matrix of destination created from input values

    //input matrix
    t_input(0,0) = cos(Rz)*cos(Ry);
    t_input(0,1) = cos(Rz)*sin(Ry)*sin(Rx) - sin(Rz)*cos(Rx);
    t_input(0,2) = cos(Rz)*sin(Ry)*cos(Rx) + sin(Rz)*sin(Rx);
    t_input(0,3) = xk;
    t_input(1,0) = sin(Rz)*cos(Ry);
    t_input(1,1) = sin(Rz)*sin(Ry)*sin(Rx) + cos(Rz)*cos(Rx);
    t_input(1,2) = sin(Rz)*sin(Ry)*cos(Rx) - cos(Rz)*sin(Rx);
    t_input(1,3) = yk;
    t_input(2,0) = -sin(Ry);
    t_input(2,1) = cos(Ry)*sin(Rx);
    t_input(2,2) = cos(Ry)*cos(Rx);
    t_input(2,3) = zk;
    t_input(3,0) = 0;
    t_input(3,1) = 0;
    t_input(3,2) = 0;
    t_input(3,3) = 1;

    MatrixXd t_start_1(4,4);
    MatrixXd t_start_2(4,4);
    MatrixXd t_start_3(4,4);
    MatrixXd t_start_4(4,4);
    MatrixXd t_start_5(4,4);

    MatrixXd t_start(4,4);

    //vector of actual values from robot
    VectorXd inputs(5);

    inputs << theta_1,theta_2,theta_3,theta_4,theta_5;

    while (blad > tolerancja)
    {
        //matrix from 1 to 0
        t_start_1(0,0) = cos(inputs(0));
        t_start_1(0,1) = 0;
        t_start_1(0,2) = -sin(inputs(0));
        t_start_1(0,3) = a1*cos(inputs(0));
        t_start_1(1,0) = sin(inputs(0));
        t_start_1(1,1) = 0;
        t_start_1(1,2) = cos(inputs(0));
        t_start_1(1,3) = a1*sin(inputs(0));
        t_start_1(2,0) = 0;
        t_start_1(2,1) = -1;
        t_start_1(2,2) = 0;
        t_start_1(2,3) = d1;
        t_start_1(3,0) = 0;
        t_start_1(3,1) = 0;
        t_start_1(3,2) = 0;
        t_start_1(3,3) = 1;

        //matrix from 2 to 1
        t_start_2(0,0) = cos(inputs(1));
        t_start_2(0,1) = -sin(inputs(1));
        t_start_2(0,2) = 0;
        t_start_2(0,3) = a2*cos(inputs(1));
        t_start_2(1,0) = sin(inputs(1));
        t_start_2(1,1) = cos(inputs(1));
        t_start_2(1,2) = 0;
        t_start_2(1,3) = a2*sin(inputs(1));
        t_start_2(2,0) = 0;
        t_start_2(2,1) = 0;
        t_start_2(2,2) = 1;
        t_start_2(2,3) = 0;
        t_start_2(3,0) = 0;
        t_start_2(3,1) = 0;
        t_start_2(3,2) = 0;
        t_start_2(3,3) = 1;

        //matrix from 3 to 2
        t_start_3(0,0) = cos(inputs(2));
        t_start_3(0,1) = -sin(inputs(2));
        t_start_3(0,2) = 0;
        t_start_3(0,3) = a3*cos(inputs(2));
        t_start_3(1,0) = sin(inputs(2));
        t_start_3(1,1) = cos(inputs(2));
        t_start_3(1,2) = 0;
        t_start_3(1,3) = a3*sin(inputs(2));
        t_start_3(2,0) = 0;
        t_start_3(2,1) = 0;
        t_start_3(2,2) = 1;
        t_start_3(2,3) = 0;
        t_start_3(3,0) = 0;
        t_start_3(3,1) = 0;
        t_start_3(3,2) = 0;
        t_start_3(3,3) = 1;

        //matrix from 4 to 3
        t_start_4(0,0) = cos(inputs(3));
        t_start_4(0,1) = 0;
        t_start_4(0,2) = -sin(inputs(3));
        t_start_4(0,3) = 0;
        t_start_4(1,0) = sin(inputs(3));
        t_start_4(1,1) = 0;
        t_start_4(1,2) = cos(inputs(3));
        t_start_4(1,3) = 0;
        t_start_4(2,0) = 0;
        t_start_4(2,1) = -1;
        t_start_4(2,2) = 0;
        t_start_4(2,3) = 0;
        t_start_4(3,0) = 0;
        t_start_4(3,1) = 0;
        t_start_4(3,2) = 0;
        t_start_4(3,3) = 1;

         //matrix from 5 to 4
        t_start_5(0,0) = cos(inputs(4));
        t_start_5(0,1) = -sin(inputs(4));
        t_start_5(0,2) = 0;
        t_start_5(0,3) = 0;
        t_start_5(1,0) = sin(inputs(4));
        t_start_5(1,1) = cos(inputs(4));
        t_start_5(1,2) = 0;
        t_start_5(1,3) = 0;
        t_start_5(2,0) = 0;
        t_start_5(2,1) = 0;
        t_start_5(2,2) = 1;
        t_start_5(2,3) = d5;
        t_start_5(3,0) = 0;
        t_start_5(3,1) = 0;
        t_start_5(3,2) = 0;
        t_start_5(3,3) = 1;

        //matrix t50
        t_start = t_start_1 * t_start_2 * t_start_3 * t_start_4 * t_start_5;

        Vector3d t_input_column_1(t_input(0,0),t_input(1,0),t_input(2,0)); //in MATLAB t_input(1:3,1)
        Vector3d t_input_column_2(t_input(0,1),t_input(1,1),t_input(2,1)); //in MATLAB t_input(1:3,2)
        Vector3d t_input_column_3(t_input(0,2),t_input(1,2),t_input(2,2)); //in MATLAB t_input(1:3,3)
        Vector3d t_input_column_4(t_input(0,3),t_input(1,3),t_input(2,3)); //in MATLAB t_input(1:3,4)

        Vector3d t_start_column_1(t_start(0,0),t_start(1,0),t_start(2,0)); //in MATLAB t_start(1:3,1)
        Vector3d t_start_column_2(t_start(0,1),t_start(1,1),t_start(2,1)); //in MATLAB t_start(1:3,2)
        Vector3d t_start_column_3(t_start(0,2),t_start(1,2),t_start(2,2)); //in MATLAB t_start(1:3,3)
        Vector3d t_start_column_4(t_start(0,3),t_start(1,3),t_start(2,3)); //in MATLAB t_start(1:3,4)

        VectorXd e_1(3);
        e_1 = t_input_column_4 - t_start_column_4;

        VectorXd e_2(3);
        e_2 = 0.5 * (t_start_column_1.cross(t_input_column_1) + t_start_column_2.cross(t_input_column_2) + t_start_column_3.cross(t_input_column_3));

        VectorXd e(6); //vector of deviation
        e << e_1, e_2;

//        cout << e << endl;
//        cout << endl;

        MatrixXd U(4,4); //eye matrix

        U(0,0) = 1;
        U(0,1) = 0;
        U(0,2) = 0;
        U(0,3) = 0;
        U(1,0) = 0;
        U(1,1) = 1;
        U(1,2) = 0;
        U(1,3) = 0;
        U(2,0) = 0;
        U(2,1) = 0;
        U(2,2) = 1;
        U(2,3) = 0;
        U(3,0) = 0;
        U(3,1) = 0;
        U(3,2) = 0;
        U(3,3) = 1;

        Vector3d d;
        Vector3d delta;

        VectorXd J5 (6);
        VectorXd J4 (6);
        VectorXd J3 (6);
        VectorXd J2 (6);
        VectorXd J1 (6);
        MatrixXd J(6,5);

        for (int i=5; i>0; i--)
        {
                MatrixXd T(4,4);
                switch (i)
                {
                        case 1:
                                T(0,0) = cos(inputs(0));
                                T(0,1) = 0;
                                T(0,2) = -sin(inputs(0));
                                T(0,3) = a1*cos(inputs(0));
                                T(1,0) = sin(inputs(0));
                                T(1,1) = 0;
                                T(1,2) = cos(inputs(0));
                                T(1,3) = a1*sin(inputs(0));
                                T(2,0) = 0;
                                T(2,1) = -1;
                                T(2,2) = 0;
                                T(2,3) = d1;
                                T(3,0) = 0;
                                T(3,1) = 0;
                                T(3,2) = 0;
                                T(3,3) = 1;
                                break;

                        case 2:
                                T(0,0) = cos(inputs(1));
                                T(0,1) = -sin(inputs(1));
                                T(0,2) = 0;
                                T(0,3) = a2*cos(inputs(1));
                                T(1,0) = sin(inputs(1));
                                T(1,1) = cos(inputs(1));
                                T(1,2) = 0;
                                T(1,3) = a2*sin(inputs(1));
                                T(2,0) = 0;
                                T(2,1) = 0;
                                T(2,2) = 1;
                                T(2,3) = 0;
                                T(3,0) = 0;
                                T(3,1) = 0;
                                T(3,2) = 0;
                                T(3,3) = 1;
                                break;

                        case 3:
                                T(0,0) = cos(inputs(2));
                                T(0,1) = -sin(inputs(2));
                                T(0,2) = 0;
                                T(0,3) = a3*cos(inputs(2));
                                T(1,0) = sin(inputs(2));
                                T(1,1) = cos(inputs(2));
                                T(1,2) = 0;
                                T(1,3) = a3*sin(inputs(2));
                                T(2,0) = 0;
                                T(2,1) = 0;
                                T(2,2) = 1;
                                T(2,3) = 0;
                                T(3,0) = 0;
                                T(3,1) = 0;
                                T(3,2) = 0;
                                T(3,3) = 1;
                                break;

                        case 4:
                                T(0,0) = cos(inputs(3));
                                T(0,1) = 0;
                                T(0,2) = -sin(inputs(3));
                                T(0,3) = 0;
                                T(1,0) = sin(inputs(3));
                                T(1,1) = 0;
                                T(1,2) = cos(inputs(3));
                                T(1,3) = 0;
                                T(2,0) = 0;
                                T(2,1) = -1;
                                T(2,2) = 0;
                                T(2,3) = 0;
                                T(3,0) = 0;
                                T(3,1) = 0;
                                T(3,2) = 0;
                                T(3,3) = 1;
                                break;

                        case 5:
                                T(0,0) = cos(inputs(4));
                                T(0,1) = -sin(inputs(4));
                                T(0,2) = 0;
                                T(0,3) = 0;
                                T(1,0) = sin(inputs(4));
                                T(1,1) = cos(inputs(4));
                                T(1,2) = 0;
                                T(1,3) = 0;
                                T(2,0) = 0;
                                T(2,1) = 0;
                                T(2,2) = 1;
                                T(2,3) = d5;
                                T(3,0) = 0;
                                T(3,1) = 0;
                                T(3,2) = 0;
                                T(3,3) = 1;
                                break;
                }

                U = T * U;

                d << (-U(0,0) * U(1,3) + U(1,0) * U(0,3)), (-U(0,1) * U(1,3) + U(1,1) * U(0,3)), (-U(0,2) * U(1,3) + U(1,2) * U(0,3));
                delta <<  U(2,0), U(2,1), U(2,2);

                if (i == 5)
                {
                   J5 << d, delta;
                }
                if (i == 4)
                {
                    J4 << d,delta;
                }
                if (i == 3)
                {
                    J3 << d,delta;
                }
                if (i == 2)
                {
                    J2 << d,delta;
                }
                if (i == 1)
                {
                    J1 << d,delta;
                }

        }

        J << J1,J2,J3,J4,J5;

        MatrixXd R(3,3);
        MatrixXd Z(3,3);
        MatrixXd Jakobian(6,6);

        R(0,0) = t_start(0,0);
        R(0,1) = t_start(0,1);
        R(0,2) = t_start(0,2);
        R(1,0) = t_start(1,0);
        R(1,1) = t_start(1,1);
        R(1,2) = t_start(1,2);
        R(2,0) = t_start(2,0);
        R(2,1) = t_start(2,1);
        R(2,2) = t_start(2,2);

        Z(0,0) = 0;
        Z(0,1) = 0;
        Z(0,2) = 0;
        Z(1,0) = 0;
        Z(1,1) = 0;
        Z(1,2) = 0;
        Z(2,0) = 0;
        Z(2,1) = 0;
        Z(2,2) = 0;

        Jakobian << R,Z,Z,R;

        Jakobian = Jakobian * J;

        MatrixXd pinv;

        MatrixXd inverseJaK;

        inverseJaK = Jakobian.adjoint() * Jakobian;
        pinv = inverseJaK.inverse() * Jakobian.adjoint();

        MatrixXd dq;

        dq = pinv * e;

        inputs = inputs + dq;


        blad = dq.norm();
        count+=1;
        //cout<<"count: "<<count<<endl;
        //cout<<"blad: "<<blad<<endl;
        //cout<<"tolerancja: "<<tolerancja<<endl;
        if (count > 1000)
        {
            break;
        }
    }

    if (count > jlimit)
    {
        q[0] = QNode::subscriber_joint1;
        q[1] = QNode::subscriber_joint2;
        q[2] = QNode::subscriber_joint3;
        q[3] = QNode::subscriber_joint4;
        q[4] = QNode::subscriber_joint5;

        if(logi==true)
        {
            log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
        }
        linear_solution_exist=false;

        return q;
    }
    else
    {
        for (int i=0; i<=4; i++)
        {
            while (inputs(i) > 2 * M_PI)
            {
                inputs(i) = inputs(i) - 2 * M_PI;
            }
            while (inputs(i) < -2 * M_PI)
            {
                inputs(i) = inputs(i) + 2 * M_PI;
            }
        }
        q[0]=inputs(0)+offset1;
        q[1]=inputs(1)+offset2;
        q[2]=inputs(2)+offset3;
        q[3]=inputs(3)+offset4+M_PI/2;
        q[4]=inputs(4)+offset5;

//        q[0]=inputs(0);
//        q[1]=inputs(1);
//        q[2]=inputs(2);
//        q[3]=inputs(3);//+M_PI/2;
//        q[4]=inputs(4);

        //cout<<q[0]<<endl;
        //cout<<q[1]<<endl;
        //cout<<q[2]<<endl;
        //cout<<q[3]<<endl;
        //cout<<q[4]<<endl;

        if ((q[0] < MainWindow::min_1) || (q[0] > MainWindow::max_1)||
                (q[1] < MainWindow::min_2) || (q[1] > MainWindow::max_2)||
                (q[2] < MainWindow::min_3) || (q[2] > MainWindow::max_3)||
                (q[3] < MainWindow::min_4) || (q[3] > MainWindow::max_4)||
                (q[4] < MainWindow::min_5) || (q[4] > MainWindow::max_5))
        {
                q[0] = QNode::subscriber_joint1;
                q[1] = QNode::subscriber_joint2;
                q[2] = QNode::subscriber_joint3;
                q[3] = QNode::subscriber_joint4;
                q[4] = QNode::subscriber_joint5;

                if(logi==true)
                {
                    log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
                }
                linear_solution_exist=false;

                return q;
        }
        else
        {
            return q;
        }
    }
}

//Signum function
int QNode::sgn(double v)
{
        if (v < 0) return -1;
        if (v >= 0) return 1;
}

//half working ik
double* QNode::inverseKinematic(double xk, double yk, double zk, double Rz, double Ry, double Rx, bool logi)
{
    cout<<"x zad: "<<xk<<endl;
    cout<<"y zad: "<<yk<<endl;
    cout<<"z zad: "<<zk<<endl;
    cout<<"roll zad: "<<Rz<<endl;
    cout<<"pitch zad: "<<Ry<<endl;
    cout<<"yaw zad: "<<Rx<<endl;

    //logi==false: do not show logs, logi==true: show logs;

    linear_solution_exist=true;
    static double q[5]; //final array with calculations of IK
    double *cords; //calculation of FK

    double RPY_1_3; //r13 from RPY matrix
    double RPY_2_3; //r23 from RPY matrix
    double RPY_3_3; //r33 from RPY matrix
    double RPY_3_1; //r31 from RPY matrix
    double RPY_3_2; //r32 from RPY matrix


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

    //Calculating elements from RPY matrix
    RPY_1_3 = cos(Rz)*sin(Ry)*cos(Rx) + sin(Rz)*sin(Rx);
    RPY_2_3 = sin(Rz)*sin(Ry)*cos(Rx) - cos(Rz)*sin(Rx);
    RPY_3_3 = cos(Ry)*cos(Rx);
    RPY_3_1 = -sin(Ry);
    RPY_3_2 = cos(Ry)*sin(Rx);

    xk = round(xk*100)/100;
    yk = round(yk*100)/100;
    zk = round(zk*100)/100;

    q[0] = atan2(yk,xk);

    beta = atan2(RPY_3_3,sqrt(RPY_1_3*RPY_1_3 + RPY_2_3*RPY_2_3));

    xp4 = round((xk - d5*cos(q[0])*cos(beta))*100)/100;
    yp4 = round((yk - d5*sin(q[0])*cos(beta))*100)/100;
    zp4 = zk - d5*sin(beta);

    if (sgn(xp4)!=sgn(xk) || sgn(yp4)!=sgn(yk))
    {
        q[0] = atan2(yp4,xp4);

        x4 = xp4 - a1*cos(q[0]);
        y4 = yp4 - a1*sin(q[0]);
        z4 = zp4 - d1;

        l = sqrt(x4*x4 + y4*y4 + z4*z4);

        cos_phi = (a2*a2 + a3*a3 - l*l)/(2*a2*a3);
        sin_phi = sqrt(1 - cos_phi*cos_phi);

        phi = atan2(sin_phi,cos_phi);

        if(elbow_state)
        {
        q[2] = (-1)*(M_PI - phi);
        }
        else
        {
        q[2] = (M_PI - phi);
        }

        A = atan2(z4,sqrt(x4*x4 + y4*y4));

        cos_B = (a2*a2 + l*l - a3*a3)/(2*l*a2);
        sin_B = sqrt(1 - cos_B*cos_B);

        B = atan2(sin_B,cos_B);

        if (q[2] < 0)
        {
            q[1] = (-1)*(A - B);
        }
        else if (q[2] > 0)
        {
            q[1] = (-1)*(A+B);
        }
        else if (q[2] == 0 || q[2] == M_PI)
        {
            q[1] = (-1)*atan2(z4*z4,sqrt(x4*x4 + y4*y4));
        }

        q[3] = (-1)*(beta + q[1] + q[2]) - M_PI;

        q[4] = atan2(RPY_3_2/(sin(q[1]+q[2])*cos(q[3])+cos(q[1]+q[2])*sin(q[3])),-RPY_3_1/(sin(q[1]+q[2])*cos(q[3])+cos(q[1]+q[2])*sin(q[3])));

        q[0] = q[0] + offset1;
        q[1] = q[1] + offset2;
        q[2] = q[2] + offset3;
        q[3] = q[3] + offset4;
        q[4] = q[4] + offset5;


        cout<<"odwr q1: "<<q[0]<<endl;
        cout<<"odwr q2: "<<q[1]<<endl;
        cout<<"odwr q3: "<<q[2]<<endl;
        cout<<"odwr q4: "<<q[3]<<endl;
        cout<<"odwr q5: "<<q[4]<<endl;


        cords = forwardKinematic(q[0],q[1],q[2],q[3],q[4]);

        if      (
                 (q[0] < MainWindow::min_1) || (q[0] > MainWindow::max_1)||
                 (q[1] < MainWindow::min_2) || (q[1] > MainWindow::max_2)||
                 (q[2] < MainWindow::min_3) || (q[2] > MainWindow::max_3)||
                 (q[3] < MainWindow::min_4) || (q[3] > MainWindow::max_4)||
                 (q[4] < MainWindow::min_5) || (q[4] > MainWindow::max_5)
                 )
        {
            q[0] = QNode::subscriber_joint1;
            q[1] = QNode::subscriber_joint2;
            q[2] = QNode::subscriber_joint3;
            q[3] = QNode::subscriber_joint4;
            q[4] = QNode::subscriber_joint5;



            if(logi==true)
            {
            log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
            }
            linear_solution_exist=false;

            return q;
        }

        else
        {

            cout<<"x zad: "<<xk<<endl;
            cout<<"y zad: "<<yk<<endl;
            cout<<"z zad: "<<zk<<endl;

            cout<<"x obl: "<<cords[0]<<endl;
            cout<<"y obl: "<<cords[1]<<endl;
            cout<<"z obl: "<<cords[2]<<endl;

//            if (xk == cords[0] && yk == cords[1] && zk == cords[2])
            if (abs(xk-cords[0])<1.0 && abs(yk-cords[1])<1.0 && abs(zk-cords[2])<1.0)
            {
                return q;
            }
            else
            {
                q[0] = QNode::subscriber_joint1;
                q[1] = QNode::subscriber_joint2;
                q[2] = QNode::subscriber_joint3;
                q[3] = QNode::subscriber_joint4;
                q[4] = QNode::subscriber_joint5;

                if(logi==true)
                {
                log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
                }
                linear_solution_exist=false;

                return q;
            }
        }


    }
    else if (sgn(xp4)==sgn(xk) || sgn(yp4)==sgn(yk))
    {
        q[0] = atan2(yk,xk);

        x4 = xp4 - a1*cos(q[0]);
        y4 = yp4 - a1*sin(q[0]);
        z4 = zp4 - d1;

        l = sqrt(x4*x4 + y4*y4 + z4*z4);

        cos_phi = (a2*a2 + a3*a3 - l*l)/(2*a2*a3);
        sin_phi = sqrt(1 - cos_phi*cos_phi);

        phi = atan2(sin_phi,cos_phi);

        if(elbow_state)
        {
        q[2] = (-1)*(M_PI - phi);
        }
        else
        {
        q[2] = (M_PI - phi);
        }

        A = atan2(z4,sqrt(x4*x4 + y4*y4));

        cos_B = (a2*a2 + l*l - a3*a3)/(2*l*a2);
        sin_B = sqrt(1 - cos_B*cos_B);

        B = atan2(sin_B,cos_B);

        if (q[2] < 0)
        {
            q[1] = (-1)*(A - B);
        }
        else if (q[2] > 0)
        {
            q[1] = (-1)*(A+B);
        }
        else if (q[2] == 0 || q[2] == M_PI)
        {
            q[1] = (-1)*atan2(z4*z4,sqrt(x4*x4 + y4*y4));
        }

        q[3] = (-1)*(beta + q[1] + q[2]);

        q[4] = atan2(RPY_3_2/(sin(q[1]+q[2])*cos(q[3])+cos(q[1]+q[2])*sin(q[3])),-RPY_3_1/(sin(q[1]+q[2])*cos(q[3])+cos(q[1]+q[2])*sin(q[3])));

        q[0] = q[0] + offset1;
        q[1] = q[1] + offset2;
        q[2] = q[2] + offset3;
        q[3] = q[3] + offset4;
        q[4] = q[4] + offset5;

        cords = forwardKinematic(q[0],q[1],q[2],q[3],q[4]);

        if      (
                 (q[0] < MainWindow::min_1) || (q[0] > MainWindow::max_1)||
                 (q[1] < MainWindow::min_2) || (q[1] > MainWindow::max_2)||
                 (q[2] < MainWindow::min_3) || (q[2] > MainWindow::max_3)||
                 (q[3] < MainWindow::min_4) || (q[3] > MainWindow::max_4)||
                 (q[4] < MainWindow::min_5) || (q[4] > MainWindow::max_5)
                 )
        {
            q[0] = QNode::subscriber_joint1;
            q[1] = QNode::subscriber_joint2;
            q[2] = QNode::subscriber_joint3;
            q[3] = QNode::subscriber_joint4;
            q[4] = QNode::subscriber_joint5;

            if(logi==true)
            {
            log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
            }
            linear_solution_exist=false;

            return q;
        }

        else
        {
            cout<<"x zad: "<<xk<<endl;
            cout<<"y zad: "<<yk<<endl;
            cout<<"z zad: "<<zk<<endl;

            cout<<"x obl: "<<cords[0]<<endl;
            cout<<"y obl: "<<cords[1]<<endl;
            cout<<"z obl: "<<cords[2]<<endl;

            if (abs(xk-cords[0])<1.0 && abs(yk-cords[1])<1.0 && abs(zk-cords[2])<1.0)
            {
                return q;
            }
            else
            {
                q[0] = QNode::subscriber_joint1;
                q[1] = QNode::subscriber_joint2;
                q[2] = QNode::subscriber_joint3;
                q[3] = QNode::subscriber_joint4;
                q[4] = QNode::subscriber_joint5;

                if(logi==true)
                {
                log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
                }
                linear_solution_exist=false;

                return q;
            }
        }


    }
}

//fk
double* QNode::forwardKinematic(double q1, double q2,double q3,double q4,double q5)
{
    static double cords[6];

    q1 = q1 - offset1;
    q2 = q2 - offset2;
    q3 = q3 - offset3;
    q4 = q4 - offset4 - M_PI/2;
    q5 = q5 - offset5;

    cords[0] = a1*cos(q1) - d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - sin(q4)*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3))) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3);
    cords[1] = a1*sin(q1) - d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3);
    cords[2] = d1 - a2*sin(q2) - d5*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - a3*cos(q2)*sin(q3) - a3*cos(q3)*sin(q2);

    cords[3] = atan2(- cos(q1)*sin(q5) - cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))), sin(q1)*sin(q5) - cos(q5)*(cos(q4)*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))));
    cords[4] = atan2(cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))), sqrt(sin(q5)*sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + (cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))));
    cords[5] = atan2(sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))), sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));

    cords[0] = round(cords[0]);
    cords[1] = round(cords[1]);
    cords[2] = round(cords[2]);

    cords[3] = round(cords[3]*100)/100;
    cords[4] = round(cords[4]*100)/100;
    cords[5] = round(cords[5]*100)/100;

    return cords;
}

//Symulator
trajectory_msgs::JointTrajectory createArmPositionCommand(std::vector<double>& newPositions)
{
        int numberOfJoints = 5;
        trajectory_msgs::JointTrajectory msg;

        trajectory_msgs::JointTrajectoryPoint point;

        for (int i = 0; i < 5; i++) {
                point.positions.push_back(newPositions[i]);
                point.velocities.push_back(5);//byly zera
                point.accelerations.push_back(3);
        }
        point.time_from_start = ros::Duration(0.1);//było 0.5
        msg.points.push_back(point);

        for (int i = 0; i < 5; i++) {
                std::stringstream jointName;
                jointName << "arm_joint_" << (i + 1);
                msg.joint_names.push_back(jointName.str());
        }

        msg.header.frame_id = "arm_link_0";
        msg.header.stamp = ros::Time::now();

        return msg;
}

//gazebo simulator msg
void QNode::moveArm(double q1, double q2,double q3,double q4,double q5)
{
        trajectory_msgs::JointTrajectory msg;
        std::vector<double> jointvalues(5);

        jointvalues[0] = q1;
        jointvalues[1] = q2;
        jointvalues[2] = q3;
        jointvalues[3] = q4;
        jointvalues[4] = q5;

        msg = createArmPositionCommand(jointvalues);
        simulatorArmPositionsPublisher.publish(msg);
}

bool QNode::isPositionAchived(int movement_iteration_temp)
{
           if(
             ((abs(P[point[movement_iteration_temp]][0]-subscriber_joint1)<0.01) &&
             (abs(P[point[movement_iteration_temp]][1]-subscriber_joint2)<0.01) &&
             (abs(P[point[movement_iteration_temp]][2]-subscriber_joint3)<0.01) &&
             (abs(P[point[movement_iteration_temp]][3]-subscriber_joint4)<0.01) &&
             (abs(P[point[movement_iteration_temp]][4]-subscriber_joint5)<0.01)) || (greatest_value==0 && start_lin_mov)
            )
                {
                    cout<<"Nr punktu gdzie dojechano: "<<point[movement_iteration_temp]<<endl;
                    cout<<"Wartość joint 1: "<<P[point[movement_iteration_temp]][0]<<endl;
                    cout<<"T_Abs: "<<abs(P[point[movement_iteration_temp]][0]-subscriber_joint1)<<endl;
                    QNode::movement_iteration++;
                    start_lin_mov=false;
                    return true;
                }
           else
           {
               //cout<<"P[point[movement_iteration_temp]][0]: "<<P[point[movement_iteration_temp]][0]<<endl;
               //cout<<"subscriber_joint1: "<<subscriber_joint1<<endl;
               //cout<<"roznica: "<<P[point[movement_iteration_temp]][0]-subscriber_joint1<<endl;
               //cout<<"F_Abs: "<<abs(P[point[movement_iteration_temp]][0]-subscriber_joint1)<<endl;
//               std_msgs::String msg;
//               std::stringstream ss;
//               ss << "F_Abs: "<<abs(P[point[movement_iteration_temp]][0]-subscriber_joint1);
//               msg.data = ss.str();

//               log(Info,std::string("")+msg.data);
                cout << "Jeszcze nie dojechałem, a greatest_value =" << greatest_value << endl;
               return false;
           }
}

//żeby przywrócić wersję przed joint_state należy zakomentować ciało jointsCallaback i odkomentować gripperCallback
void jointsCallback(const sensor_msgs::JointStateConstPtr& youbotArmState)
{
    double q1 = youbotArmState->position[0];
    double q2 = youbotArmState->position[1];
    double q3 = youbotArmState->position[2];
    double q4 = youbotArmState->position[3];
    double q5 = youbotArmState->position[4];

    q1 = q1 - offset1;
    q2 = q2 - offset2;
    q3 = q3 - offset3;
    q4 = q4 - offset4 - M_PI/2;
    q5 = q5 - offset5;

    QNode::x = a1*cos(q1) - d5*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - sin(q4)*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3))) + a2*cos(q1)*cos(q2) + a3*cos(q1)*cos(q2)*cos(q3) - a3*cos(q1)*sin(q2)*sin(q3);
    QNode::y = a1*sin(q1) - d5*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + a2*cos(q2)*sin(q1) + a3*cos(q2)*cos(q3)*sin(q1) - a3*sin(q1)*sin(q2)*sin(q3);
    QNode::z = d1 - a2*sin(q2) - d5*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - a3*cos(q2)*sin(q3) - a3*cos(q3)*sin(q2);

    QNode::x = round(QNode::x);
    QNode::y = round(QNode::y);
    QNode::z = round(QNode::z);

    QNode::roll = atan2(- cos(q1)*sin(q5) - cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))), sin(q1)*sin(q5) - cos(q5)*(cos(q4)*(cos(q1)*sin(q2)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))));
    QNode::pitch = atan2(cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))), sqrt(sin(q5)*sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + (cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))));
    QNode::yaw = atan2(sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))), sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));

    QNode::roll = round(QNode::roll*100)/100;
    QNode::pitch = round(QNode::pitch*100)/100;
    QNode::yaw = round(QNode::yaw*100)/100;

    QNode::subscriber_joint1 = youbotArmState->position[0];
    QNode::subscriber_joint2 = youbotArmState->position[1];
    QNode::subscriber_joint3 = youbotArmState->position[2];
    QNode::subscriber_joint4 = youbotArmState->position[3];
    QNode::subscriber_joint5 = youbotArmState->position[4];
    QNode::subscriber_gripper_1 = youbotArmState->position[5];
    QNode::subscriber_gripper_2 = youbotArmState->position[6];

    MainWindow::joint_1 = QNode::subscriber_joint1;
    MainWindow::joint_2 = QNode::subscriber_joint2;
    MainWindow::joint_3 = QNode::subscriber_joint3;
    MainWindow::joint_4 = QNode::subscriber_joint4;
    MainWindow::joint_5 = QNode::subscriber_joint5;
}

void diagnosticsCallback(const diagnostic_msgs::DiagnosticArrayConstPtr& youbotArmDiagnostic)
{
    if(youbotArmDiagnostic->status[2].message=="EtherCAT connnection is established")
    {
        if(QNode::ethercat_connection==false)
        {
        QNode::ethercat_connection_temp=true;
        }
        QNode::ethercat_connection=true;
    }

    if(youbotArmDiagnostic->status[2].message=="EtherCAT connnection lost")
    {
        if(QNode::ethercat_connection==true)
        {
        QNode::ethercat_connection_temp2=true;
        }
        QNode::ethercat_connection=false;
    }
}

double stringToDouble (string q)
{
    std::replace(q.begin(), q.end(), '.', ','); // Na kompilatorze terminalowym czyta kropki, zapisuje jako kropki
    double q_d = atof(q.c_str());               // Catkin czyta przecinki, zapisuje kropki - WTF??11!!11

    return q_d;
}

void QNode::readPointsFromFile()
{
    line_nmb=0;
    points.open( "punkty.txt", ios::in | ios::out | ios::app);
    if( points.good() == true )
    {
        cout << "Uzyskano dostep do pliku!" << endl;
        while (getline(points, line[line_nmb]))
        {
            pos1 = line[line_nmb].find(";",1);
            pos2 = line[line_nmb].find(";",pos1+1);
            pos3 = line[line_nmb].find(";",pos2+1);
            pos4 = line[line_nmb].find(";",pos3+1);
            pos5 = line[line_nmb].find(";",pos4+1);
            pos6 = line[line_nmb].find(";",pos5+1);
            q1[line_nmb] = stringToDouble(line[line_nmb].substr(pos1+1,pos2-(pos1+1)));
            q2[line_nmb] = stringToDouble(line[line_nmb].substr(pos2+1,pos3-(pos2+1)));
            q3[line_nmb] = stringToDouble(line[line_nmb].substr(pos3+1,pos4-(pos3+1)));
            q4[line_nmb] = stringToDouble(line[line_nmb].substr(pos4+1,pos5-(pos4+1)));
            q5[line_nmb] = stringToDouble(line[line_nmb].substr(pos5+1,pos6-(pos5+1)));
            ++line_nmb;
        } //Czyta jointy z pliku punkty.txt
        points.close();
        for (int j=0;j<5;j++)
        {
            for (int i=1;i<=line_nmb;i++)
            {
                if(j==0)
                {
                    P[i][j] = q1[i-1];
                }
                else if (j==1)
                {
                    P[i][j] = q2[i-1];
                }
                else if (j==2)
                {
                    P[i][j] = q3[i-1];
                }
                else if (j==3)
                {
                    P[i][j] = q4[i-1];
                }
                else
                {
                    P[i][j] = q5[i-1];
                }
            }
        }
    }

    else cout << "Brak dostępu do pliku" << endl;
}

void QNode::readProgramFromFile()
{
    //number_of_lin_mov=0;
    std_msgs::String msg1;
    std::stringstream error;
    msg1.data = error.str();
    //error << "Treść błędu";
    error << endl;
    string line[100];
    int row_number=0;
    fstream file;
    state = 0;

    file.open( "program.txt", ios::in | ios::out | ios::app);
    if( file.good() == true )
    {
        cout << "Uzyskano dostep do pliku!" << endl;
        while (getline(file, line[row_number]))
        {
            // cout << line[row_number] << endl;
            ++row_number;
        }
        cout << "Plik ma " << row_number << " wierszy" << endl;
        file.close();
        QNode::program_line_number=row_number;
    }

    else
    {
        cout << "Brak dostępu do pliku" << endl; // przerwać funkcje
    }
    string temp_point[row_number];
    string temp_velocity_command[row_number];
    string temp_velocity_value[row_number];
    //string command[row_number];

    for (int i = 0; i<row_number;  i++)
    {
        command[i] = line[i].substr(0,3);
        if (command[i] == "GRI")
        {
            state+=2;
            string gripper_command;
            command[i] = line[i].substr(0,8);
            if (command[i] == "GRIPPER ")
            {
                state++;
                gripper_command = line[i].substr(8,4);
                if (gripper_command=="OPEN")
                {
                    state++;
                    command[i]="GRO";
                    if(i>0)
                        point[i]=point[i-1];
                    else
                    {
                        state--;
                        error << " Program nie może zaczynać się komendą GRIPPER [linia " << i+1 << "]" << endl;
                    }
                    cout << "Otwieram gripper" << endl;
                }

                else if (gripper_command=="CLOS")
                {
                    state++;
                    command[i]="GRC";
                    if(i>0)
                        point[i]=point[i-1];
                    else
                    {
                        state--;
                        error << " Program nie może zaczynać się komendą GRIPPER [linia " << i+1 << "]" << endl;
                    }
                    cout << "Zamykam gripper" << endl;
                }
                else
                {
                    error << " Brak komendy OPEN/CLOSE [linia " << i+1 << "]" << endl;
                }
                state+=2;
            }
            else
            {
                error << " Nieznana komenda [linia " << i+1 << "]" << endl;

            }
        }
        else
        {
            //        cout << line[i].length() << endl;
            if (line[i].length()>=6)
            {
                temp_point[i] = line[i].substr (4,1);
                point[i] = atoi((line[i].substr (5,2)).c_str());
            }
            if (line[i].length()>=12)
            {
                if (point[i]<=9)
                {
                    temp_velocity_command[i] = line[i].substr (7,4);
                    temp_velocity_value[i] = line[i].substr (11,3);
                }
                else if (point[i]>9)
                {
                    temp_velocity_command[i] = line[i].substr (8,4);
                    temp_velocity_value[i] = line[i].substr (12,3);
                }

            }
            if ((command[i]=="PTP"))
                state+=2;
            else if ((command[i]=="LIN"))
            {
                cout<<"readProgram: znalazlem LIN"<<endl;
                //**************************************************************
                state++;
                //   lin_mov_array[number_of_lin_mov]=point[i];
                //   number_of_lin_mov++;
                if(checkLinearMovementPossibility(i, false))
                {
                    state++;
                }
                else
                {
                    error << " Nie można zrealizować ruchu liniowego [linia " << i+1 << "]" << endl;
                }
            }
            else
                error << "Nieznana komenda [linia " << i+1 << "]" << endl;


            if (temp_point[i]=="P")
                state++;
            else
                error << " Błędnie wprowadzony punkt [linia " << i+1 << "]" << endl;
            if (point[i]<=line_nmb)
            {
                state++;
                if (temp_velocity_command[i]=="VEL ")
                {
                    state++;
                    if ((atoi(temp_velocity_value[i].c_str())>0) && (atoi(temp_velocity_value[i].c_str())<=100))
                    {
                        velocity[i] = atoi(temp_velocity_value[i].c_str());
                        state++;
                    }
                    else
                        error << " Wprowadzono błędną wartość prędkości [linia " << i+1 << "]" << endl;
                }
                else
                    error << " Brak komendy VEL dla ruchu " << command[i] << " P" << point[i] <<"[linia " << i+1 << "]" << endl;

            }
            else
                error << " Punkt P" << point[i] << " nie został zdefiniowany [linia " << i+1 << "]" << endl;
        }
    }
    if (state/6!=row_number)
    {
        cout << state << endl;
        msg1.data=error.str();
        //cout << "Błąd składni kodu" << endl; // wyprowadzić log i przerwać funkcje
        log(Error,std::string("Błąd składni kodu:")+msg1.data);
    }
    else
    {
        if((movement_iteration==row_number)&&(play_program==true))
        {
            play_program=false;
            movement_iteration=0;
            log(Info,std::string("[Tryb automatyczny] Wykonano program"));
        }

        for (int i=0;i<row_number;i++)
        {
            cout << command[i] << " Punkt " << point[i] << " prędkość: " << velocity[i] << endl;
        }

        if((movement_iteration!=row_number)&&(play_program==true))
        {
            do
            {
               // cout << "Prędkość 1: " << velocity[5] << endl;

                if(command[movement_iteration]=="PTP")
                {
                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << point[QNode::movement_iteration];
                    msg.data = ss.str();
                    log(Info,std::string("[Tryb automatyczny] Wykonano ruch PTP P")+msg.data);

                    QNode::jointPublisher(P[point[QNode::movement_iteration]][0],P[point[QNode::movement_iteration]][1],
                            P[point[QNode::movement_iteration]][2],P[point[QNode::movement_iteration]][3],
                            P[point[QNode::movement_iteration]][4]);
                }

                if(command[movement_iteration]=="LIN")
                {

                    cout<<"wszedlem do do while - lin"<<endl;


                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << point[movement_iteration];
                    msg.data = ss.str();
                    log(Info,std::string("[Tryb automatyczny] Wykonano ruch LIN P")+msg.data);


                    QNode::executeLIN(movement_iteration);
                }

                if(command[movement_iteration]=="GRO")
                {
                    log(Info,std::string("[Tryb automatyczny] Otwarto chwytak"));
                    gripperPublisher(0.011, 0.011);
                    opening_gripper=true;
                }

                if(command[movement_iteration]=="GRC")
                {
                    log(Info,std::string("[Tryb automatyczny] Zamknięto chwytak"));
                    gripperPublisher(0, 0);
                    closing_gripper=true;
                }



                QNode::execute_movement_flag=false;
            }

            while (QNode::execute_movement_flag);
        }
    }
}


void QNode::loadPointsList()
{
    list_model.removeRows(0,list_model.rowCount());


    if(points_list_view_mode)
    {
    for (int i=-1; i<line_nmb; i++)
    {
        double *cords;
        cords = forwardKinematic(q1[i],q2[i],q3[i],q4[i],q5[i]);
                std::stringstream msg;
        msg<< "P" << i+1 << ": x: " << cords[0] << "\ty: " << cords[1] << "\tz: " << cords[2]
           <<  "\troll: " << cords[3] << "\tpitch: " << cords[4] << "\tyaw: " << cords[5];
        list_model.setData(list_model.index(0),"    [mm]\t[mm]\t[mm]\t[rad]\t[rad]\t[rad]");
        list_model.insertRows(list_model.rowCount(),1);
        QVariant new_row(QString(msg.str().c_str()));
        list_model.setData(list_model.index(list_model.rowCount()-1),new_row);
        Q_EMIT listUpdated(); // used to readjust the scrollbar
    }
    }
    else
    {
        for (int i=-1; i<line_nmb; i++)
            {
            double q_1 = round(q1[i]*1000)/1000;
            double q_2 = round(q2[i]*1000)/1000;;
            double q_3 = round(q3[i]*1000)/1000;;
            double q_4 = round(q4[i]*1000)/1000;;
            double q_5 = round(q5[i]*1000)/1000;;
                std::stringstream msg;
                msg<< "P" << i+1 << ": \tq1: " << q_1 << "\tq2: " << q_2 << "\tq3: " << q_3
                   <<  "\tq4: " << q_4 << "\tq5: " << q_5;

                list_model.setData(list_model.index(0),"\t[rad]\t[rad]\t[rad]\t[rad]\t[rad]");
                list_model.insertRows(list_model.rowCount(),1);
                QVariant new_row(QString(msg.str().c_str()));
                list_model.setData(list_model.index(list_model.rowCount()-1),new_row);
                Q_EMIT listUpdated(); // used to readjust the scrollbar
            }
    }

}

void QNode::jointPublisher(double q1, double q2,double q3,double q4,double q5)
{
    double *cords;
    cords = forwardKinematic(q1, q2, q3, q4, q5);
    if(cords[2]<5)
    {
        log(Info,std::string("Nie można wykonać - robot uderzy w stół"));
    }
    else
    {
    brics_actuator::ProgramExecuteVelocity msg;
    if (play_program||isManualPTPActiv)
    {
        msg.velocity=QNode::velocity[movement_iteration];
    }
    else
    {
        msg.velocity=100;
    }
    velocityPublisher.publish(msg);

    static const int numberOfArmJoints = 5;
    brics_actuator::JointPositions command;
    vector <brics_actuator::JointValue> jointPosition;
    jointPosition.resize(numberOfArmJoints); //TODO:change that
    std::stringstream jointName;

    jointPosition[0].value = q1;
    jointPosition[1].value = q2;
    jointPosition[2].value = q3;
    jointPosition[3].value = q4;
    jointPosition[4].value = q5;


    for (int i = 0; i < numberOfArmJoints; ++i)
    {
        //cout << "Please type in value for joint " << i + 1 << endl;
        //cin >> readValue;

        jointName.str("");
        jointName << "arm_joint_" << (i + 1);

        jointPosition[i].joint_uri = jointName.str();
        //jointPosition[i].value = readValue;

        jointPosition[i].unit = boost::units::to_string(boost::units::si::radians);
        //cout << "Joint " << jointPosition[i].joint_uri << " = " << jointPosition[i].value << " " << jointPosition[i].unit << endl;
    };

    command.positions = jointPosition;
    armPositionsPublisher.publish(command);
    QNode::moveArm(q1, q2, q3, q4, q5);
    }
}

void QNode::gripperPublisher(double gripper_1, double gripper_2)
{
    static const int numberOfGripperJoints = 2;
    brics_actuator::JointPositions command;
    vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(numberOfGripperJoints);

    gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
    gripperJointPositions[0].value = gripper_1;
    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

    gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
    gripperJointPositions[1].value = gripper_2;
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

    command.positions = gripperJointPositions;
    gripperPositionPublisher.publish(command);
}

bool QNode::isGripperPositionAchived(double value)
{
    if(
      (abs(value-subscriber_gripper_1)<0.0005) &&
      (abs(value-subscriber_gripper_2)<0.0005)
     )
         {
             movement_iteration++;
             return true;
         }
    else
    {
        return false;
    }
}

void QNode::moveHome()
{
    QNode::jointPublisher(QNode::home[0],QNode::home[1],QNode::home[2],QNode::home[3],QNode::home[4]);
    log(Info,std::string("Wykonano ruch do pozycji domowej"));
}

bool QNode::isHomePositionAchived()
{
    if(
      (abs(QNode::home[0]-subscriber_joint1)<0.0005) &&
      (abs(QNode::home[1]-subscriber_joint2)<0.0005) &&
      (abs(QNode::home[2]-subscriber_joint3)<0.0005) &&
      (abs(QNode::home[3]-subscriber_joint4)<0.0005) &&
      (abs(QNode::home[4]-subscriber_joint5)<0.0005)
     )
         {
             return true;
         }
    else
    {
        return false;
    }
}

bool QNode::isLittleStepExecuted()
{
    if(
      (abs(QNode::actual_little_step_position[0]-subscriber_joint1)<0.01) &&
      (abs(QNode::actual_little_step_position[1]-subscriber_joint2)<0.01) &&
      (abs(QNode::actual_little_step_position[2]-subscriber_joint3)<0.01) &&
      (abs(QNode::actual_little_step_position[3]-subscriber_joint4)<0.01) &&
      (abs(QNode::actual_little_step_position[4]-subscriber_joint5)<0.01)
     )
         {
             return true;
         }
    else
    {
        return false;
    }
}

void QNode::manualPTP(int i)
{
         QNode::isManualPTPActiv=true;
         jointPublisher(P[point[i]][0],P[point[i]][1],P[point[i]][2],P[point[i]][3],P[point[i]][4]);
         cout<<"Numer pkt: "<<i<<endl;

         std_msgs::String msg;
         std::stringstream ss;
         ss << point[i];
         msg.data = ss.str();

         log(Info,std::string("[Tryb ręczny] Wykonano ruch PTP P")+msg.data);
         QNode::isManualPTPActiv=false;
}

void QNode::convActNumbOfLinMov2Joint()
{
//    myYouBotManipulator = new YouBotManipulator("youbot-manipulator", "/home/mateusz/youbot/src/youbot_driver-hydro-devel/config/");
//    YouBotManipulator* myYouBotManipulator = 0;
//    YouBotManipulator myYouBotManipulator("youbot-manipulator", "/home/mateusz/youbot/src/youbot_driver-hydro-devel/config/");
//    MaximumPositioningVelocity maxPositioningVelocity;
//    myYouBotManipulator->getArmJoint(1).getConfigurationParameter(maxPositioningVelocity);
//    quantity<angular_velocity> velocity;
//    maxPositioningVelocity.getParameter(velocity);
//    std::cout << "Maximum positioning speed of joint 1: " << velocity << std::endl;
}

void QNode::lin(double q1, double q2,double q3,double q4,double q5)
{
//    if((distance_x!=0)&&(distance_y!=0)&&(distance_z!=0))
//    {
    if(greatest_value>0)
    {
        prev_x+=distance_x/greatest_value;
        prev_y+=distance_y/greatest_value;
        prev_z+=distance_z/greatest_value;
    }
//    }

    double* joints;
    joints=QNode::inverseKinematicJacobi(prev_x, prev_y, prev_z, prev_roll, prev_pitch, prev_yaw, false, false);

    actual_little_step_position[0]=joints[0];
    actual_little_step_position[1]=joints[1];
    actual_little_step_position[2]=joints[2];
    actual_little_step_position[3]=joints[3];
    actual_little_step_position[4]=joints[4];

    jointPublisher(actual_little_step_position[0], actual_little_step_position[1], actual_little_step_position[2], actual_little_step_position[3], actual_little_step_position[4]);
cout<<"to maly krok dla czlowieka, ale wielki dla Youbota"<<endl;
    executed_little_step=false;
    //lin_mov_little_steps_count++;

}

bool QNode::checkLinearMovementPossibility(int destination_point, bool mode)
{
        linear_solution_exist=true;

        double q1_destination=P[point[destination_point]][0];
        double q2_destination=P[point[destination_point]][1];
        double q3_destination=P[point[destination_point]][2];
        double q4_destination=P[point[destination_point]][3];
        double q5_destination=P[point[destination_point]][4];

//        double q1_destination=P[destination_point][0];
//        double q2_destination=P[destination_point][1];
//        double q3_destination=P[destination_point][2];
//        double q4_destination=P[destination_point][3];
//        double q5_destination=P[destination_point][4];


        //cout<<"q1_destination: "<<q1_destination<<endl;
        //cout<<"q2_destination: "<<q2_destination<<endl;
        //cout<<"q3_destination: "<<q3_destination<<endl;
        //cout<<"q4_destination: "<<q4_destination<<endl;
        //cout<<"q5_destination: "<<q5_destination<<endl;

        double q1_prev;
        double q2_prev;
        double q3_prev;
        double q4_prev;
        double q5_prev;

        //mode=false: read previous point from program, mode=true: read actual positions

        if(mode==true)
        {
        q1_prev=QNode::subscriber_joint1;
        q2_prev=QNode::subscriber_joint2;
        q3_prev=QNode::subscriber_joint3;
        q4_prev=QNode::subscriber_joint4;
        q5_prev=QNode::subscriber_joint5;
        }
        else
        {
        q1_prev=P[point[destination_point-1]][0];
        q2_prev=P[point[destination_point-1]][1];
        q3_prev=P[point[destination_point-1]][2];
        q4_prev=P[point[destination_point-1]][3];
        q5_prev=P[point[destination_point-1]][4];
        }

        //cout<<"q1_prev: "<<q1_prev<<endl;
        //cout<<"q2_prev: "<<q2_prev<<endl;
        //cout<<"q3_prev: "<<q3_prev<<endl;
        //cout<<"q4_prev: "<<q4_prev<<endl;
        //cout<<"q5_prev: "<<q5_prev<<endl;

        double *destination_cords;
        destination_cords = forwardKinematic(q1_destination, q2_destination, q3_destination, q4_destination, q5_destination);

        double destination_x = destination_cords[0];
        double destination_y = destination_cords[1];
        double destination_z = destination_cords[2];
        double destination_roll = destination_cords[3];
        double destination_pitch = destination_cords[4];
        double destination_yaw = destination_cords[5];

        //cout<<"destination_x: "<<destination_x<<endl;
        //cout<<"destination_y: "<<destination_y<<endl;
        //cout<<"destination_z: "<<destination_z<<endl;

        double *prev_cords;
        prev_cords = forwardKinematic(q1_prev, q2_prev, q3_prev, q4_prev, q5_prev);

         prev_x = prev_cords[0];
         prev_y = prev_cords[1];
         prev_z = prev_cords[2];
         prev_roll = prev_cords[3];
         prev_pitch = prev_cords[4];
         prev_yaw = prev_cords[5];

        //cout<<"prev_x: "<<prev_x<<endl;
        //cout<<"prev_y: "<<prev_y<<endl;
        //cout<<"prev_z: "<<prev_z<<endl;

         distance_x = destination_x-prev_x;
         distance_y = destination_y-prev_y;
         distance_z = destination_z-prev_z;

      //  cout<<"distance_x: "<<distance_x<<endl;
    //    cin>>distance_x;
       // cout<<"distance_y: "<<distance_y<<endl;
    //    cin>>distance_y;
        //cout<<"distance_z: "<<distance_z<<endl;
    //    cin>>distance_z;

    if (abs(distance_x) >= abs(distance_y))
    {
        if(abs(distance_x) >= abs(distance_z))
        {
            greatest_value = abs(distance_x);
        }
        else
        {
            greatest_value = abs(distance_z);
        }
    }
    else
    {
        if(abs(distance_y) >= abs(distance_z))
        {
            greatest_value = abs(distance_y);
        }
        else
        {
            greatest_value = abs(distance_z);
        }
    }
    greatest_value=greatest_value/2;

cout<<"greatest_value: "<<greatest_value<<endl;

    double execute_x=prev_x;
    double execute_y=prev_y;
    double execute_z=prev_z;
    int counter=0;
    q1_lin_check=q1_prev;
    q2_lin_check=q2_prev;
    q3_lin_check=q3_prev;
    q4_lin_check=q4_prev;
    q5_lin_check=q5_prev;
    double *q_lin_check;
    for (int i=1; i < greatest_value; i++)
    {
        execute_x += distance_x/greatest_value;
        execute_y += distance_y/greatest_value;
        execute_z += distance_z/greatest_value;
        //cout<<"execute_x: "<<execute_x<<endl;
        //cout<<"execute_y: "<<execute_y<<endl;
        //cout<<"execute_z: "<<execute_z<<endl;


        q_lin_check=QNode::inverseKinematicJacobi(execute_x, execute_y, execute_z, prev_roll, prev_pitch ,prev_yaw, false, true);
        q1_lin_check=q_lin_check[0];
        q2_lin_check=q_lin_check[1];
        q3_lin_check=q_lin_check[2];
        q4_lin_check=q_lin_check[3];
        q5_lin_check=q_lin_check[4];
/*
        std_msgs::String msg3;
        std::stringstream ss;
        ss << counter;
        msg3.data = ss.str();
        std_msgs::String msg4;
        std::stringstream ss2;
        ss2 << greatest_value;
        msg4.data = ss.str();
        log(Error,std::string("Counter="+msg3.data+"Greatest_value="+msg4.data));*/
        cout << "ITERACJA=" << i << endl;
        if(linear_solution_exist==false)
        {
            counter++;
        }
    }
   // cout<<"counter: "<<counter<<endl;
    if (counter==0)
    {
        cout<<"można wykonać ruch"<<endl;
        return true;
    }
    else
    {
        cout<<" nie można wykonać ruchu"<<endl;
        return false;
    }
}

void QNode::executeLIN(int point_number)
{
         if(QNode::checkLinearMovementPossibility(point_number, true))
         {
         start_lin_mov=true;
         lin(P[point_number][0],P[point_number][1],P[point_number][2],P[point_number][3],P[point_number][4]);

         }
         //cout<<"Numer pkt: "<<i<<endl;

}

void QNode::executeProgram()
{
        line_nmb=0;
        state=0;

        readPointsFromFile();
        readProgramFromFile();

}

void QNode::jointSimulator(int i)
{
    currentTime = ros::Time::now();
    sensor_msgs::JointState armJointStateMessages;

    armJointStateMessages.header.stamp = currentTime;
    armJointStateMessages.name.resize(7);
    armJointStateMessages.position.resize(7);
    armJointStateMessages.velocity.resize(7);
    armJointStateMessages.effort.resize(7);

    for(int i=0;i<7;i++)
    {
        armJointStateMessages.name[i] = "nazwa";
        armJointStateMessages.velocity[i] = 0;
        armJointStateMessages.effort[i] = 0;
    }
    double th_1;
    double th_2;
    double th_3;
    double th_4;
    double th_5;
    double th_6;
    double th_7;

    if(i==0)
    {
th_1=1.1;th_2=0.886693;th_3=-2.15466;th_4=1.60133;th_5=2.93141; th_6=0; th_7=0;
cout<<"Jestem w stanie 0, P1, close"<<endl;

    }

    else if(i==1)
    {
th_1=1.1;th_2=0.886693;th_3=-2.15466;th_4=1.60133;th_5=2.93141; th_6=0.007; th_7=0.007;
cout<<"Jestem w stanie 1, P1, opening"<<endl;

    }

    else if(i==2)
    {
th_1=1.1;th_2=0.886693;th_3=-2.15466;th_4=1.60133;th_5=2.93141; th_6=0.011; th_7=0.011;
cout<<"Jestem w stanie 2, P1, open"<<endl;

    }

    else if(i==3)
    {
        th_1=2.1;th_2=1.83454;th_3=-3.36926;th_4=3.28443;th_5=2.93141;th_6=0.011; th_7=0.011;
        cout<<"Jestem w stanie 3, P2 ,open"<<endl;
    }

    else if(i==4)
    {
        th_1=2.1;th_2=1.83454;th_3=-3.36926;th_4=3.28443;th_5=2.93141;th_6=0.007; th_7=0.007;
        cout<<"Jestem w stanie 4, P2, closing"<<endl;
    }

    else if(i==5)
    {
    th_1=2.1;th_2=1.83465;th_3=-3.30264;th_4=3.2177;th_5=2.93141;th_6=0; th_7=0;
    cout<<"Jestem w stanie 5, P2 close"<<endl;
    }

    else if(i==6)
    {
    th_1=3.1;th_2=1.83465;th_3=-3.30264;th_4=3.2177;th_5=2.93141;th_6=0; th_7=0;
    cout<<"Jestem w stanie 6, P3 close"<<endl;
    }
    else
    {
        cout<<"Jestem w stanie innym"<<endl;
    th_1=0.0;
    th_2=0.0;
    th_3=0.0;
    th_4=0.0;
    th_5=0.0;
    }


    armJointStateMessages.position[0] = th_1;
    armJointStateMessages.position[1] = th_2;
    armJointStateMessages.position[2] = th_3;
    armJointStateMessages.position[3] = th_4;
    armJointStateMessages.position[4] = th_5;
    armJointStateMessages.position[5] = th_6;
    armJointStateMessages.position[6] = th_7;


    jointsPublisher.publish(armJointStateMessages);
}

QNode::QNode(int argc, char** argv)://, YouBotOODLWrapper* youBot) :
        init_argc(argc),
        init_argv(argv)
        {
    //  this->youBot = youBot;
    //  YouBotManipulator youBotArmName1;
    //  public:
    //  YouBotManipulator myYouBotManipulator("youbot-manipulator", "/home/mateusz/youbot/src/youbot_driver-hydro-devel/config/");
    //, YouBotOODLWrapper* youBot
        }

QNode::~QNode()
{
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
        wait();
}

bool QNode::init()
{
        ros::init(init_argc,init_argv,"youbot_gui");
        if ( ! ros::master::check() )
        {
                return false;
        }
        ros::start();
        ros::NodeHandle n;
        armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
        gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);
        jointsPublisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
        jointsSubscriber = n.subscribe<sensor_msgs::JointState >("/joint_states", 10, jointsCallback);
        diagnosticsSubscriber = n.subscribe<diagnostic_msgs::DiagnosticArray >("/diagnostics", 10, diagnosticsCallback);
        simulatorArmPositionsPublisher = n.advertise<trajectory_msgs::JointTrajectory>("arm_1/arm_controller/command", 1);
        velocityPublisher = n.advertise<brics_actuator::ProgramExecuteVelocity>("arm_1/arm_controller/velocity", 1);


        P[0][0]=MainWindow::min_1;
        P[0][1]=MainWindow::min_2;
        P[0][2]=MainWindow::max_3;
        P[0][3]=MainWindow::min_4;
        P[0][4]=MainWindow::min_5;

        home[0]=P[0][0];
        home[1]=P[0][1];
        home[2]=P[0][2];
        home[3]=P[0][3];
        home[4]=P[0][4];

        for (int i=0;i<100;i++)
        {
        QNode::velocity[i]=5;
        }


        QNode::log(Info,std::string("Połączono ze sterownikiem robota"));

        start();
        return true;
}

void QNode::run()
{
        ros::Rate loop_rate(5); //zmienione z 1 na 20
        int count = 0;
        MainWindow::joint_1 = QNode::subscriber_joint1;
        MainWindow::joint_2 = QNode::subscriber_joint2;
        MainWindow::joint_3 = QNode::subscriber_joint3;
        MainWindow::joint_4 = QNode::subscriber_joint4;
        MainWindow::joint_5 = QNode::subscriber_joint5;




        while ( ros::ok() ) {

//            brics_actuator::ProgramExecuteVelocity msg;

//            msg.velocity=50;

//            velocityPublisher.publish(msg);

//            MainWindow::joint_1 = QNode::subscriber_joint1;
//            MainWindow::joint_2 = QNode::subscriber_joint2;
//            MainWindow::joint_3 = QNode::subscriber_joint3;
//            MainWindow::joint_4 = QNode::subscriber_joint4;
//            MainWindow::joint_5 = QNode::subscriber_joint5;


                this->ui.lcd_q1->display(QNode::subscriber_joint1);
                this->ui.lcd_q2->display(QNode::subscriber_joint2);
                this->ui.lcd_q3->display(QNode::subscriber_joint3);
                this->ui.lcd_q4->display(QNode::subscriber_joint4);
                this->ui.lcd_q5->display(QNode::subscriber_joint5);
                this->ui.lcd_x->display(QNode::x);
                this->ui.lcd_y->display(QNode::y);
                this->ui.lcd_z->display(QNode::z);
                this->ui.lcd_roll->display(QNode::roll);
                this->ui.lcd_pitch->display(QNode::pitch);
                this->ui.lcd_yaw->display(QNode::yaw);
                this->ui.lcd_xyz_step->display(MainWindow::xyz_step);
                this->ui.lcd_joints_step->display(MainWindow::joints_step);

                MainWindow::xyz_step=ui.krok_xyz->value();
                MainWindow::joints_step=ui.krok_zlacza->value();
                MainWindow::joints_step=MainWindow::joints_step/20;



            if((QNode::execute_movement_flag==false)&&(closing_gripper==false)&&(opening_gripper==false))
            {
                if(QNode::execute_movement_flag = QNode::isPositionAchived(QNode::movement_iteration))
                {QNode::readProgramFromFile();}
            }

            if((QNode::execute_movement_flag==false)&&(closing_gripper==false)&&(opening_gripper==false)&&(start_lin_mov==true))
            {

                if(executed_little_step==false)
                {
                    if(QNode::isLittleStepExecuted())
                    {
                        executed_little_step=true;
                        cout<<"els: false"<<endl;
                    }
                }

                if(executed_little_step==true)
                {
                    cout<<"els: true"<<endl;
                    executeLIN(movement_iteration);
                }

            }



            else if(closing_gripper==true)
            {
                if(QNode::execute_movement_flag = QNode::isGripperPositionAchived(0))
                {
                    closing_gripper=false;
                    QNode::readProgramFromFile();
                }
            }

            else if(opening_gripper==true)
            {
                if(QNode::execute_movement_flag = QNode::isGripperPositionAchived(0.011))
                {
                    opening_gripper=false;
                    QNode::readProgramFromFile();
                }
            }


            if (QNode::ethercat_connection_temp==true)
            {
            log(Info,std::string("Nawiązano łączność EtherCAT"));

                QNode::ethercat_connection_temp=false;
            }
            else if (QNode::ethercat_connection_temp2==true)
            {
               log(Error,std::string("Utracono łączność EtherCAT"));
                QNode::ethercat_connection_temp2=false;
            }
                ros::spinOnce();
                loop_rate.sleep();
                ++count;
        }
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg) {
        logging_model.insertRows(logging_model.rowCount(),1);
        std::stringstream logging_model_msg;
        switch ( level ) {
                case(Debug) : {
                                ROS_DEBUG_STREAM(msg);
                                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                                break;
                }
                case(Info) : {
                                ROS_INFO_STREAM(msg);
                                logging_model_msg << "[INFO]: " << msg;
                                break;
                }
                case(Warn) : {
                                ROS_WARN_STREAM(msg);
                                logging_model_msg << "[WARN]: " << msg;
                                break;
                }
                case(Error) : {
                                ROS_ERROR_STREAM(msg);
                                logging_model_msg << "[ERROR]: " << msg;
                                break;
                }
                case(Fatal) : {
                                ROS_FATAL_STREAM(msg);
                                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                                break;
                }
        }
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}
