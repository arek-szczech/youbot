#include "../include/youbot_gui/main_window.hpp" //Kolejność includów jest krytyczna xD Musi być na górze
#include <ros/ros.h>
#include <ros/network.h>
#include <string.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
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
#include "../include/youbot_gui/qnode.hpp"

using namespace std;
namespace youbot_gui {

//jointsCallback
double QNode::subscriber_joint1;
double QNode::subscriber_joint2;
double QNode::subscriber_joint3;
double QNode::subscriber_joint4;
double QNode::subscriber_joint5;

double QNode::subscriber_gripper_1;
double QNode::subscriber_gripper_2;

//specialIK
double QNode::ik_th1;
double QNode::ik_th2;
double QNode::ik_th3;
double QNode::ik_th4;
double QNode::ik_th5;

//zestaw z jointsCallbac - aktualna fk, wyświetlacze, zadawanie odwrotnej
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
double offset1 = 2.8668;
double offset2 = 2.59191;
double offset3 = -2.52113;
double offset4 = 1.75973;
double offset5 = 2.93141;

//Signum function do odwrotnej
int QNode::sgn(double v)
{
        if (v < 0) return -1;
        if (v >= 0) return 1;
}

double* QNode::inverseKinematic(double xk, double yk, double zk, double Rz, double Ry, double Rx)
{
    static double q[5]; //final array with calculations of IK
    double *cords; //calculation of FK

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

    //Calculating elements from RPY matrix
    RPY_3_1 = cos(Rz)*sin(Ry)*cos(Rx) + sin(Rz)*sin(Rx);
    RPY_3_2 = sin(Rz)*sin(Ry)*cos(Rx) - cos(Rz)*sin(Rx);
    RPY_3_3 = cos(Ry)*cos(Rx);

    xk = round(xk*100)/100;
    yk = round(yk*100)/100;
    zk = round(zk*100)/100;

    q[0] = atan2(yk,xk);

    beta = atan2(RPY_3_3,sqrt(RPY_3_1*RPY_3_1 + RPY_3_2*RPY_3_2));

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

        q[4] = 0;

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

            log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
            linear_solution_exist=false;

            return q;
        }

        else
        {



            if (xk == cords[0] && yk == cords[1] && zk == cords[2])
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

                log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
                linear_solution_exist=false;

                return q;
            }
        }


    }
    else if (sgn(xp4)==sgn(xk) || sgn(yp4)==sgn(yk))
    {
        q[1] = atan2(yk,xk);

        x4 = xp4 - a1*cos(q[1]);
        y4 = yp4 - a1*sin(q[1]);
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

        q[4] = 0;

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

            log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
            linear_solution_exist=false;

            return q;
        }

        else
        {


            if (xk == cords[0] && yk == cords[1] && zk == cords[2])
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

                log(Warn,std::string("Nie można osiągnąć zadanej pozycji."));
                linear_solution_exist=false;

                return q;
            }
        }


    }
}

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

void QNode::specialInverseKinematics(double xk, double yk, double zk, double Rz, double Ry, double Rx)
{
//    //Calculating elements from RPY matrix
//    RPY_3_1 = cos(Rz)*sin(Ry)*cos(Rx) + sin(Rz)*sin(Rx);
//    RPY_3_2 = sin(Rz)*sin(Ry)*cos(Rx) - cos(Rz)*sin(Rx);
//    RPY_3_3 = cos(Ry)*cos(Rx);

//    xk = round(xk*100)/100;
//    yk = round(yk*100)/100;
//    zk = round(zk*100)/100;

//    theta_1 = atan2(yk,xk);

//    beta = atan2(RPY_3_3,sqrt(RPY_3_1*RPY_3_1 + RPY_3_2*RPY_3_2));

//    xp4 = round((xk - d5*cos(theta_1)*cos(beta))*100)/100;
//    yp4 = round((yk - d5*sin(theta_1)*cos(beta))*100)/100;
//    zp4 = zk - d5*sin(beta);

//    if (sgn(xp4)!=sgn(xk) || sgn(yp4)!=sgn(yk))
//    {
//        theta_1 = atan2(yp4,xp4);

//        x4 = xp4 - a1*cos(theta_1);
//        y4 = yp4 - a1*sin(theta_1);
//        z4 = zp4 - d1;

//        l = sqrt(x4*x4 + y4*y4 + z4*z4);

//        cos_phi = (a2*a2 + a3*a3 - l*l)/(2*a2*a3);
//        sin_phi = sqrt(1 - cos_phi*cos_phi);

//        phi = atan2(sin_phi,cos_phi);

//        theta_3 = (-1)*(M_PI - phi);
//        //theta_3 = (M_PI - phi);

//        A = atan2(z4,sqrt(x4*x4 + y4*y4));

//        cos_B = (a2*a2 + l*l - a3*a3)/(2*l*a2);
//        sin_B = sqrt(1 - cos_B*cos_B);

//        B = atan2(sin_B,cos_B);

//        if (theta_3 < 0)
//        {
//            theta_2 = (-1)*(A - B);
//        }
//        else if (theta_3 > 0)
//        {
//            theta_2 = (-1)*(A+B);
//        }
//        else if (theta_3 == 0 || theta_3 == M_PI)
//        {
//            theta_2 = (-1)*atan2(z4*z4,sqrt(x4*x4 + y4*y4));
//        }

//        theta_4 = (-1)*(beta + theta_2 + theta_3) - M_PI;

//        theta_5 = 0;

//        double th1 = theta_1;
//        double th2 = theta_2;
//        double th3 = theta_3;
//        double th4 = theta_4 - M_PI/2;
//        double th5 = theta_5;

//        double x = a1*cos(th1) - d5*(cos(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3);
//        double y = a1*sin(th1) - d5*(cos(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) + a2*cos(th2)*sin(th1) + a3*cos(th2)*cos(th3)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3);
//        double z = d1 - a2*sin(th2) - d5*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))) - a3*cos(th2)*sin(th3) - a3*cos(th3)*sin(th2);

//        //double roll = atan2(- cos(th1)*sin(th5) - cos(th5)*(cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2))), sin(th1)*sin(th5) - cos(th5)*(cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2))));
//        //double pitch = atan2(cos(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sqrt(sin(th5)*sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)))*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))) + (cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))));
//        //double yaw = atan2(sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) - cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)));

//        x = round(x*1)/1;
//        y = round(y*1)/1;
//        z = round(z*1)/1;

//        QNode::ik_th1 = theta_1 + offset1;
//        QNode::ik_th2 = theta_2 + offset2;
//        QNode::ik_th3 = theta_3 + offset3;
//        QNode::ik_th4 = theta_4 + offset4;
//        QNode::ik_th5 = theta_5 + offset5;

//        if      (
//                 (QNode::ik_th1 < MainWindow::min_1) || (QNode::ik_th1 > MainWindow::max_1)||
//                 (QNode::ik_th2 < MainWindow::min_2) || (QNode::ik_th2 > MainWindow::max_2)||
//                 (QNode::ik_th3 < MainWindow::min_3) || (QNode::ik_th3 > MainWindow::max_3)||
//                 (QNode::ik_th4 < MainWindow::min_4) || (QNode::ik_th4 > MainWindow::max_4)||
//                 (QNode::ik_th5 < MainWindow::min_5) || (QNode::ik_th5 > MainWindow::max_5)
//                 )
//        {
//            log(Warn,std::string("Nie można osiągnąć zadanej pozycji"));
//        }

//        else
//        {
////            cout << "Wpisana wspolrzedna X: " << xk << endl;
////            cout << "Obliczona wspolrzedna X: " << x << endl;
////            cout << "Wpisana wspolrzedna Y: " << yk << endl;
////            cout << "Obliczona wspolrzedna Y: " << y << endl;
////            cout << "Wpisana wspolrzedna Z: " << zk << endl;
////            cout << "Obliczona wspolrzedna Z: " << z << endl;

//            if (xk == x && yk == y && zk == z)
//            {
//                QNode::jointPublisher(ik_th1, ik_th2, ik_th3, ik_th4, ik_th5);
////                cout << "Wyliczona z IK theta_1: " << theta_1 << endl;
////                cout << "Wyliczona z IK theta_2: " << theta_2 << endl;
////                cout << "Wyliczona z IK theta_3: " << theta_3 << endl;
////                cout << "Wyliczona z IK theta_4: " << theta_4 << endl;
////                cout << "Wyliczona z IK theta_5: " << theta_5 << endl;
////                cout << "" << endl;

////                cout << "Wysylana do robota theta_1: " << theta_1 + offset1 << endl;
////                cout << "Wysylana do robota theta_2: " << theta_2 + offset2 << endl;
////                cout << "Wysylana do robota theta_3: " << theta_3 + offset3 << endl;
////                cout << "Wysylana do robota theta_4: " << theta_4 + offset4 << endl;
////                cout << "Wysylana do robota theta_5: " << theta_5 + offset5 << endl;
////                cout << "" << endl;
//            }
//            else
//            {
//                QNode::ik_th1 = QNode::subscriber_joint1;
//                QNode::ik_th2 = QNode::subscriber_joint2;
//                QNode::ik_th3 = QNode::subscriber_joint3;
//                QNode::ik_th4 = QNode::subscriber_joint4;
//                QNode::ik_th5 = QNode::subscriber_joint5;
//                log(Warn,std::string("Nie można osiągnąć zadanej pozycji"));
////              cout << "BRAK ROZWIAZANIA!" << endl;
//                cout << "" << endl;
//            }
//        }


//    }
//    else if (sgn(xp4)==sgn(xk) || sgn(yp4)==sgn(yk))
//    {
//        theta_1 = atan2(yk,xk);

//        x4 = xp4 - a1*cos(theta_1);
//        y4 = yp4 - a1*sin(theta_1);
//        z4 = zp4 - d1;

//        l = sqrt(x4*x4 + y4*y4 + z4*z4);

//        cos_phi = (a2*a2 + a3*a3 - l*l)/(2*a2*a3);
//        sin_phi = sqrt(1 - cos_phi*cos_phi);

//        phi = atan2(sin_phi,cos_phi);

//        theta_3 = (-1)*(M_PI - phi);
//        //theta_3 = (M_PI - phi);

//        A = atan2(z4,sqrt(x4*x4 + y4*y4));

//        cos_B = (a2*a2 + l*l - a3*a3)/(2*l*a2);
//        sin_B = sqrt(1 - cos_B*cos_B);

//        B = atan2(sin_B,cos_B);

//        if (theta_3 < 0)
//        {
//            theta_2 = (-1)*(A - B);
//        }
//        else if (theta_3 > 0)
//        {
//            theta_2 = (-1)*(A+B);
//        }
//        else if (theta_3 == 0 || theta_3 == M_PI)
//        {
//            theta_2 = (-1)*atan2(z4*z4,sqrt(x4*x4 + y4*y4));
//        }

//        theta_4 = (-1)*(beta + theta_2 + theta_3);

//        theta_5 = 0;

//        double th1 = theta_1;
//        double th2 = theta_2;
//        double th3 = theta_3;
//        double th4 = theta_4-M_PI/2;
//        double th5 = theta_5;

//        double x = a1*cos(th1) - d5*(cos(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3);
//        double y = a1*sin(th1) - d5*(cos(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) + a2*cos(th2)*sin(th1) + a3*cos(th2)*cos(th3)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3);
//        double z = d1 - a2*sin(th2) - d5*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))) - a3*cos(th2)*sin(th3) - a3*cos(th3)*sin(th2);

//        //double roll = atan2(- cos(th1)*sin(th5) - cos(th5)*(cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2))), sin(th1)*sin(th5) - cos(th5)*(cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2))));
//        //double pitch = atan2(cos(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sqrt(sin(th5)*sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)))*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))) + (cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))));
//        //double yaw = atan2(sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) - cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)));

//        x = round(x*1)/1;
//        y = round(y*1)/1;
//        z = round(z*1)/1;

//        QNode::ik_th1 = theta_1 + offset1;
//        QNode::ik_th2 = theta_2 + offset2;
//        QNode::ik_th3 = theta_3 + offset3;
//        QNode::ik_th4 = theta_4 + offset4;
//        QNode::ik_th5 = theta_5 + offset5;



//        if      (
//                 (QNode::ik_th1 < MainWindow::min_1) || (QNode::ik_th1 > MainWindow::max_1)||
//                 (QNode::ik_th2 < MainWindow::min_2) || (QNode::ik_th2 > MainWindow::max_2)||
//                 (QNode::ik_th3 < MainWindow::min_3) || (QNode::ik_th3 > MainWindow::max_3)||
//                 (QNode::ik_th4 < MainWindow::min_4) || (QNode::ik_th4 > MainWindow::max_4)||
//                 (QNode::ik_th5 < MainWindow::min_5) || (QNode::ik_th5 > MainWindow::max_5)
//                 )
//        {
//            log(Warn,std::string("Nie można osiągnąć zadanej pozycji"));
//        }

//        else
//        {
////            cout << "Wpisana wspolrzedna X: " << xk << endl;
////            cout << "Obliczona wspolrzedna X: " << x << endl;
////            cout << "Wpisana wspolrzedna Y: " << yk << endl;
////            cout << "Obliczona wspolrzedna Y: " << y << endl;
////            cout << "Wpisana wspolrzedna Z: " << zk << endl;
////            cout << "Obliczona wspolrzedna Z: " << z << endl;

//            if (xk == x && yk == y && zk == z)
//            {
//                QNode::jointPublisher(ik_th1, ik_th2, ik_th3, ik_th4, ik_th5);
////                cout << "" << endl;
////                cout << "Wyliczona z IK theta_1: " << theta_1 << endl;
////                cout << "Wyliczona z IK theta_2: " << theta_2 << endl;
////                cout << "Wyliczona z IK theta_3: " << theta_3 << endl;
////                cout << "Wyliczona z IK theta_4: " << theta_4 << endl;
////                cout << "Wyliczona z IK theta_5: " << theta_5 << endl;
////                cout << "" << endl;

////                cout << "Wysylana do robota theta_1: " << theta_1 + offset1 << endl;
////                cout << "Wysylana do robota theta_2: " << theta_2 + offset2 << endl;
////                cout << "Wysylana do robota theta_3: " << theta_3 + offset3 << endl;
////                cout << "Wysylana do robota theta_4: " << theta_4 + offset4 << endl;
////                cout << "Wysylana do robota theta_5: " << theta_5 + offset5 << endl;
////                cout << "" << endl;
//            }
//            else
//            {
//                QNode::ik_th1 = QNode::subscriber_joint1;
//                QNode::ik_th2 = QNode::subscriber_joint2;
//                QNode::ik_th3 = QNode::subscriber_joint3;
//                QNode::ik_th4 = QNode::subscriber_joint4;
//                QNode::ik_th5 = QNode::subscriber_joint5;


//                log(Warn,std::string("Nie można osiągnąć zadanej pozycji"));

////                cout << "BRAK ROZWIAZANIA!" << endl;
//            }
//        }


//    }
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
             (abs(P[point[movement_iteration_temp]][0]-subscriber_joint1)<0.05) &&
             (abs(P[point[movement_iteration_temp]][1]-subscriber_joint2)<0.05) &&
             (abs(P[point[movement_iteration_temp]][2]-subscriber_joint3)<0.05) &&
             (abs(P[point[movement_iteration_temp]][3]-subscriber_joint4)<0.05) &&
             (abs(P[point[movement_iteration_temp]][4]-subscriber_joint5)<0.05)
            )
                {
                    cout<<"Nr punktu gdzie dojechano: "<<point[movement_iteration_temp]<<endl;
                    cout<<"Wartość joint 1: "<<P[point[movement_iteration_temp]][0]<<endl;
                    cout<<"T_Abs: "<<abs(P[point[movement_iteration_temp]][0]-subscriber_joint1)<<endl;
                    QNode::movement_iteration++;
                    return true;
                }
           else
           {
               //cout<<"P[point[movement_iteration_temp]][0]: "<<P[point[movement_iteration_temp]][0]<<endl;
               //cout<<"subscriber_joint1: "<<subscriber_joint1<<endl;
               //cout<<"roznica: "<<P[point[movement_iteration_temp]][0]-subscriber_joint1<<endl;
               //cout<<"F_Abs: "<<abs(P[point[movement_iteration_temp]][0]-subscriber_joint1)<<endl;
               std_msgs::String msg;
               std::stringstream ss;
               ss << "F_Abs: "<<abs(P[point[movement_iteration_temp]][0]-subscriber_joint1);
               msg.data = ss.str();

               log(Info,std::string("")+msg.data);

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
    //string command[row_number];

        for (int i = 0; i<row_number;  i++)
    {
        command[i] = line[i].substr(0,3);
        if (command[i] == "GRI")
        {
            state+=2;
            string gripper_command;
            command[i] = line[i].substr(0,7);
            if (command[i] == "GRIPPER")
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
                        cout << "Program nie może zaczynać się komendą GRIPPER";
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
                    cout << "Program nie może zaczynać się komendą GRIPPER";
                }
                cout << "Zamykam gripper" << endl;
            }
            else
            {
                cout << "Brak komendy OPEN/CLOSE";
            }
        }
        else
        {
            cout << "Nieznana komenda";

        }
        }
        else
        {

        temp_point[i] = line[i].substr (4,1);
        point[i] = atoi((line[i].substr (5,2)).c_str());
        if ((command[i]=="PTP"))
            state+=2;
        else if ((command[i]=="LIN"))
        {
            state++;
  //******************************************************************
            //if(checkLinearMovementPossibility(i))
  //******************************************************************
             if(true)
               {
                state++;
             }
            else
            {
                cout << "Nie można zrealizować ruchu liniowego";
            }
        }
        else
            cout << "Nieznana komenda" << endl;


        if (temp_point[i]=="P")
            state++;
        else
            cout << "Błędnie wprowadzony punkt" << endl;
        if (point[i]<=line_nmb)
            state++;
        else
            cout << "Punkt P" << point[i] << " nie został zdefiniowany" << endl;
    }
        }
        if (state/4!=row_number)
        {
         cout << state << endl;
         cout << "Błąd składni kodu" << endl; // wyprowadzić log i przerwać funkcje
        }
        else
        {
            if((movement_iteration==row_number)&&(play_program==true))
            {
                play_program=false;
                movement_iteration=0;
                log(Info,std::string("[Tryb automatyczny] Wykonano program"));
            }

            if((movement_iteration!=row_number)&&(play_program==true))
            {
            do
            {
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
                    std_msgs::String msg;
                    std::stringstream ss;
                    ss << point[QNode::movement_iteration];
                    msg.data = ss.str();
                    log(Info,std::string("[Tryb automatyczny] Wykonano ruch LIN P")+msg.data);

//                    QNode::ptp(P[point[QNode::movement_iteration]][0],P[point[QNode::movement_iteration]][1],
//                            P[point[QNode::movement_iteration]][2],P[point[QNode::movement_iteration]][3],
//                            P[point[QNode::movement_iteration]][4]);
//                    //cout<<"Ważny test: "<<point[QNode::movement_iteration]<<endl;
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

void QNode::jointPublisher(double q1, double q2,double q3,double q4,double q5)
{
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

void QNode::manualPTP(int i)
{
         jointPublisher(P[point[i]][0],P[point[i]][1],P[point[i]][2],P[point[i]][3],P[point[i]][4]);
         cout<<"Numer pkt: "<<i<<endl;

         std_msgs::String msg;
         std::stringstream ss;
         ss << point[i];
         msg.data = ss.str();

         log(Info,std::string("[Tryb ręczny] Wykonano ruch PTP P")+msg.data);
}

void QNode::lin(double q1, double q2,double q3,double q4,double q5)
{

}

bool QNode::checkLinearMovementPossibility(int destination_point)
{   
    linear_solution_exist=true;

    double q1_destination=P[destination_point][0]; //odczyt punktu do ktorego liniowo jedziemy
    double q2_destination=P[destination_point][1];
    double q3_destination=P[destination_point][2];
    double q4_destination=P[destination_point][3];
    double q5_destination=P[destination_point][4];

    double q1_prev=P[destination_point-1][0]; //odczyt punktu z ktorego liniowo jedziemy
    double q2_prev=P[destination_point-1][1];
    double q3_prev=P[destination_point-1][2];
    double q4_prev=P[destination_point-1][3];
    double q5_prev=P[destination_point-1][4];


    double q1 = q1_destination - 2.8668;
    double q2 = q2_destination - 2.5919;
    double q3 = q3_destination + 2.5211;
    double q4 = q4_destination - 3.3305;
    double q5 = q5_destination - 2.9314;

    double *destination_cords;
    destination_cords = forwardKinematic(q1, q2, q3, q4, q5);


    q1 = q1_prev - 2.8668;
    q2 = q2_prev - 2.5919;
    q3 = q3_prev + 2.5211;
    q4 = q4_prev - 3.3305;
    q5 = q5_prev - 2.9314;

    double *prev_cords;
    prev_cords = forwardKinematic(q1, q2, q3, q4, q5);


    double distance_x = destination_cords[0]-prev_cords[0];
    double distance_y = destination_cords[1]-prev_cords[1];
    double distance_z = destination_cords[2]-prev_cords[2];

    cout<<"distance_x"<<distance_x<<endl;
    cin>>distance_x;
    cout<<"distance_y"<<distance_y<<endl;
    cin>>distance_y;
    cout<<"distance_z"<<distance_z<<endl;
    cin>>distance_z;


    double greatestValue;

    if (abs(distance_x) >= abs(distance_y))
    {
        if(abs(distance_x) >= abs(distance_z))
        {
            greatestValue = abs(distance_x);
        }
        else
        {
            greatestValue = abs(distance_z);
        }
    }
    else
    {
        if(abs(distance_y) >= abs(distance_z))
        {
            greatestValue = abs(distance_y);
        }
        else
        {
            greatestValue = abs(distance_z);
        }
    }
cout<<"greatestValue"<<greatestValue<<endl;

    double execute_x=prev_cords[0];
    double execute_y=prev_cords[1];
    double execute_z=prev_cords[2];
    int counter=0;

    for (int i=1; i <= greatestValue; i++)
    {
        execute_x += distance_x/greatestValue;
        execute_y += distance_y/greatestValue;
        execute_z += distance_z/greatestValue;
        cout<<"execute_x: "<<execute_x<<endl;
        cout<<"execute_y: "<<execute_y<<endl;
        cout<<"execute_z: "<<execute_z<<endl;

        QNode::inverseKinematic(execute_x, execute_y, execute_x, prev_cords[3], prev_cords[4],prev_cords[5]);
        if(linear_solution_exist==false)
        {
            counter++;
        }
    }

    if (counter==0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void QNode::executeLIN(int i)
{
         lin(P[point[i]][0],P[point[i]][1],P[point[i]][2],P[point[i]][3],P[point[i]][4]);
         cout<<"Numer pkt: "<<i<<endl;

         std_msgs::String msg;
         std::stringstream ss;
         ss << point[i];
         msg.data = ss.str();

         log(Info,std::string("Wykonano ruch LIN P")+msg.data);
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

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

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

        QNode::log(Info,std::string("Połączono ze sterownikiem robota"));

        start();
	return true;
}

void QNode::run()
{
        ros::Rate loop_rate(2); //zmienione z 1 na 20
	int count = 0;
        MainWindow::joint_1 = QNode::subscriber_joint1;
        MainWindow::joint_2 = QNode::subscriber_joint2;
        MainWindow::joint_3 = QNode::subscriber_joint3;
        MainWindow::joint_4 = QNode::subscriber_joint4;
        MainWindow::joint_5 = QNode::subscriber_joint5;

	while ( ros::ok() ) {

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

            if((QNode::execute_movement_flag==false)&&(closing_gripper==false)&&(opening_gripper==false))
            {
                if(QNode::execute_movement_flag = QNode::isPositionAchived(QNode::movement_iteration))
                {QNode::readProgramFromFile();}
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
