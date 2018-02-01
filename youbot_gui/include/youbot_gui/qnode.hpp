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

#ifndef youbot_gui_QNODE_HPP_
#define youbot_gui_QNODE_HPP_

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <QtWidgets/QMainWindow>
#include <QLCDNumber>
#include "ui_main_window.h"


namespace youbot_gui {

class QNode : public QThread {
    Q_OBJECT
public:

    static double x;
    static double y;
    static double z;
    static double roll;
    static double pitch;
    static double yaw;

    static double subscriber_joint1;
    static double subscriber_joint2;
    static double subscriber_joint3;
    static double subscriber_joint4;
    static double subscriber_joint5;

    static double subscriber_gripper_1;
    static double subscriber_gripper_2;

    static int program_state;
    static int program_line_number;
    static int point[100];
    static std::string command[100];
    static bool back_to_home;
    static bool execute_movement_flag;
    static int movement_iteration;

    static double home[5];
    static bool play_program;
    static bool ethercat_connection;
    static bool ethercat_connection_temp;
    static bool ethercat_connection_temp2;
    static bool elbow_state;

    static bool opening_gripper;
    static bool closing_gripper;

    static bool linear_solution_exist;

    double q1_prev;
    double q2_prev;
    double q3_prev;
    double q4_prev;
    double q5_prev;
    static int greatest_value;
    static bool executed_little_step;
    static double actual_little_step_position[5];
    static double prev_x;
    static double prev_y;
    static double prev_z;
    static double prev_roll;
    static double prev_pitch;
    static double prev_yaw;
    static double distance_x;
    static double distance_y;
    static double distance_z;
    static bool start_lin_mov;

    static double q1_lin_check;
    static double q2_lin_check;
    static double q3_lin_check;
    static double q4_lin_check;
    static double q5_lin_check;

    static int velocity[100];
    static bool isManualPTPActiv;
    static bool points_list_view_mode;

    Ui::MainWindowDesign ui;

    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void run();
    void executeProgram();
    double* forwardKinematic(double q1, double q2,double q3,double q4,double q5);
    void readPointsFromFile();
    void loadPointsList();
    void readProgramFromFile();
    void jointPublisher(double q1, double q2,double q3,double q4,double q5);
    void gripperPublisher(double gripper_1, double gripper_2);
    void moveArm(double q1, double q2,double q3,double q4,double q5);
    void manualPTP(int i);
    double* inverseKinematic(double xk, double yk, double zk, double Rz, double Ry, double Rx, bool logi);
    double* inverseKinematicJacobi(double xk, double yk, double zk, double Rz, double Ry, double Rx, bool logi, bool check);
    int sgn(double v);
    void moveHome();
    void jointSimulator(int i);
    bool isPositionAchived(int movement_iteration_temp);
    void lin(double q1, double q2,double q3,double q4,double q5);
    void executeLIN(int point_number);
    bool checkLinearMovementPossibility(int destination_point, bool mode);
    bool isHomePositionAchived();
    bool isGripperPositionAchived(double state);
    bool isLittleStepExecuted();

    enum LogLevel {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg);

    QStringListModel* listModel() { return &list_model; }


Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void listUpdated();

private:
    int init_argc;
    char** init_argv;
    ros::Publisher armPositionsPublisher;
    ros::Publisher gripperPositionPublisher;
    ros::Publisher simulatorArmPositionsPublisher;
    ros::Publisher jointsPublisher;
    ros::Subscriber jointsSubscriber;
    ros::Subscriber diagnosticsSubscriber;
    ros::Publisher velocityPublisher;

    QStringListModel logging_model;
    QStringListModel list_model;

    ros::Time currentTime;

};

}  // namespace youbot_gui

#endif /* youbot_gui_QNODE_HPP_ */
