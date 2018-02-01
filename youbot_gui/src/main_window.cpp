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

#include "../include/youbot_gui/qnode.hpp"
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/youbot_gui/main_window.hpp"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <QProcess>
#include <QLCDNumber>
#include <QCloseEvent>
#include <QPixmap>
#include <std_msgs/String.h>

namespace youbot_gui {

using namespace Qt;
using namespace std;

double MainWindow::min_1 = 0.0100693;
double MainWindow::max_1 = 5.84014;
//double MainWindow::joint_1 = 0.0100692;
//static double joint_1 = 0.0100692;
double MainWindow::joint_1 = QNode::subscriber_joint1;

double MainWindow::min_2 = 0.0100693;
double MainWindow::max_2 = 2.61799;
//double MainWindow::joint_2 = 0.0100692;
double MainWindow::joint_2 = QNode::subscriber_joint2;

double MainWindow::min_3 = -5.02655;
double MainWindow::max_3 = -0.015709;
//double MainWindow::joint_3 = -0.015708;
double MainWindow::joint_3 = QNode::subscriber_joint3;

double MainWindow::min_4 = 0.0221240;
double MainWindow::max_4 = 3.4292;
//double MainWindow::joint_4 = 0.0221239;
double MainWindow::joint_4 = QNode::subscriber_joint4;

double MainWindow::min_5 = 0.110620;
double MainWindow::max_5 = 5.64159;
//double MainWindow::joint_5 = 0.110619;
double MainWindow::joint_5 = QNode::subscriber_joint5;

double min_6 = 0;
double max_6 = 0.011;
static double gripper_1 = 0;

double min_7 = 0;
double max_7 = 0.011;
static double gripper_2 = 0;

int MainWindow::zmienna=0;

int MainWindow::xyz_step = 1;
double MainWindow::joints_step = 0.05;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent )
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this);
    ui.youbot_picture->setPixmap(QPixmap("/home/arek/youbot/src/youbot/youbot_gui/resources/images/youbot2.png"));
//    ui.youbot_picture->setPixmap(QPixmap(":/images/youbot2.png"));
    qnode.ui = ui;
    //        qnode.initUi(ui);
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

    setWindowIcon(QIcon(":/images/icon.png"));

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    ui.points_list->setModel(qnode.listModel());
    QObject::connect(&qnode, SIGNAL(listUpdated()), this, SLOT(updateListView()));



    /*********************
        ** Logging
        **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

}



MainWindow::~MainWindow() {}

void MainWindow::on_edit_clicked(bool check)
{

    system("bash -c ''cd ~/youbot ; gedit program.txt''");
}

void MainWindow::on_execute_clicked(bool check)
{
    qnode.log(qnode.Info,std::string("[Tryb automatyczny] Wykonywanie programu w trybie automatycznym"));
    QNode::play_program=true;
    qnode.executeProgram();
}

void MainWindow::on_pause_clicked(bool check)
{
    qnode.log(qnode.Info,std::string("[Tryb automatyczny] Wykonywanie programu wstrzymane"));
    QNode::play_program=false;
    QNode::execute_movement_flag=true;
}

void MainWindow::on_stop_clicked(bool check)
{
    qnode.log(qnode.Info,std::string("[Tryb automatyczny] Wykonywanie programu zatrzymane"));
    QNode::play_program=false;
    QNode::movement_iteration=0;
    QNode::execute_movement_flag=true;
    QNode::executed_little_step=false;
    QNode::start_lin_mov=false;
}

void MainWindow::on_previous_clicked(bool check)
{
    if(QNode::play_program==false)
    {
        qnode.readPointsFromFile();
        qnode.readProgramFromFile();

        cout<<"Przed dek: "<<QNode::movement_iteration<<endl;

        if(QNode::movement_iteration>0)
        {
            cout<<"Prev_prog_line_numb: "<<QNode::program_line_number<<endl;
            if(QNode::movement_iteration==QNode::program_line_number)
            {
                QNode::movement_iteration=QNode::movement_iteration-2;
            }
            else
            {
                QNode::movement_iteration--;
            }

            if(QNode::command[QNode::movement_iteration]=="PTP")
            {
                qnode.manualPTP(QNode::movement_iteration);
            }

            if(QNode::command[QNode::movement_iteration]=="GRO")
            {
                qnode.gripperPublisher(0.011, 0.011);
                qnode.log(QNode::Info,std::string("[Tryb ręczny] Otwarto chwytak"));
            }

            if(QNode::command[QNode::movement_iteration]=="GRC")
            {
                qnode.gripperPublisher(0, 0);
                qnode.log(QNode::Info,std::string("[Tryb ręczny] Zamknięto chwytak"));
            }

            if(QNode::command[QNode::movement_iteration]=="LIN")
            {
                qnode.executeLIN(QNode::movement_iteration);
                std_msgs::String msg;
                std::stringstream ss;
                ss << QNode::point[QNode::movement_iteration];
                msg.data = ss.str();
                qnode.log(QNode::Info,std::string("[Tryb ręczny] Wykonano ruch LIN P")+msg.data);
                QNode::execute_movement_flag==false;
            }

        }
        cout<<"Po dek: "<<QNode::movement_iteration<<endl;
    }
}

void MainWindow::on_next_clicked(bool check)
{
    if(QNode::play_program==false)
    {
        qnode.readPointsFromFile();
        qnode.readProgramFromFile();

        cout<<"Przed ink: "<<QNode::movement_iteration<<endl;

        if(QNode::movement_iteration<QNode::program_line_number)
        {
            cout<<"Next_prog_line_numb: "<<QNode::program_line_number<<endl;

            if(QNode::command[QNode::movement_iteration]=="PTP")
            {
                qnode.manualPTP(QNode::movement_iteration);
            }


            if(QNode::command[QNode::movement_iteration]=="GRO")
            {
                qnode.gripperPublisher(0.011, 0.011);
                qnode.log(QNode::Info,std::string("[Tryb ręczny] Otwarto chwytak"));

            }

            if(QNode::command[QNode::movement_iteration]=="GRC")
            {
                qnode.gripperPublisher(0, 0);
                qnode.log(QNode::Info,std::string("[Tryb ręczny] Zamknięto chwytak"));
            }


            if(QNode::command[QNode::movement_iteration]=="LIN")
            {

                qnode.executeLIN(QNode::movement_iteration);
                std_msgs::String msg;
                std::stringstream ss;
                ss << QNode::point[QNode::movement_iteration];
                msg.data = ss.str();
                qnode.log(QNode::Info,std::string("[Tryb ręczny] Wykonano ruch LIN P")+msg.data);
                QNode::execute_movement_flag=false;
                QNode::movement_iteration--;
            }

            QNode::movement_iteration++;

        }
        cout<<"Po ink: "<<QNode::movement_iteration<<endl;
    }
}

void MainWindow::on_home_clicked(bool check)
{
    qnode.moveHome();
}

void MainWindow::on_run_driver_clicked(bool check)
{
    qnode.log(qnode.Info,std::string("Uruchamianie sterownika..."));
    system("gnome-terminal -x sh -c 'cd ~/youbot ; source devel/setup.bash ; roslaunch youbot_driver_ros_interface youbot_driver.launch'");
    ui.connect_master->setEnabled(true);
}

void MainWindow::on_connect_master_clicked(bool check)
{
    qnode.init();
    qnode.readPointsFromFile();

    //qnode.log(qnode.Info,std::string("Wczytano listę punktów"));
    qnode.readPointsFromFile();
    qnode.loadPointsList();
    qnode.moveHome();

    ui.execute->setEnabled(true);
    ui.pause->setEnabled(true);
    ui.stop->setEnabled(true);
    ui.edit->setEnabled(true);
    ui.previous->setEnabled(true);
    ui.next->setEnabled(true);
    ui.home->setEnabled(true);
    ui.save->setEnabled(true);
    ui.edit_list->setEnabled(true);
    ui.x_plus->setEnabled(true);
    ui.x_minus->setEnabled(true);
    ui.y_plus->setEnabled(true);
    ui.y_minus->setEnabled(true);
    ui.z_plus->setEnabled(true);
    ui.z_minus->setEnabled(true);
    ui.roll_plus->setEnabled(true);
    ui.roll_minus->setEnabled(true);
    ui.pitch_plus->setEnabled(true);
    ui.pitch_minus->setEnabled(true);
    ui.q1_plus->setEnabled(true);
    ui.q1_minus->setEnabled(true);
    ui.q2_plus->setEnabled(true);
    ui.q2_minus->setEnabled(true);
    ui.q3_plus->setEnabled(true);
    ui.q3_minus->setEnabled(true);
    ui.q4_plus->setEnabled(true);
    ui.q4_minus->setEnabled(true);
    ui.q5_plus->setEnabled(true);
    ui.q5_minus->setEnabled(true);
    ui.gripper_open->setEnabled(true);
    ui.gripper_close->setEnabled(true);

}

void MainWindow::on_save_clicked(bool check) //zapisz punkt
{
    string line[100];
    fstream file;
    fstream file_temp;
    int point_number = 0;

    file_temp.open( "punkty.txt", ios::in | ios::out | ios::app);
    if( file.good() == true )
    {
        while (getline(file_temp, line[point_number]))
        {
            point_number++;
        }
        file_temp.close();
    }
    else cout << "Dostep do pliku zostal zabroniony!" << endl;


    file.open( "punkty.txt", ios::in | ios::out | ios::app);
    if( file.good() == true )
    {
        cout << "Uzyskano dostep do pliku!" << endl;
        cout<<point_number<<endl;
        //file << "P0;1;1.1;1.1;1.1;1.1;" << endl;
        file<<"P"<<point_number+1<<";"<< QNode::subscriber_joint1 <<";"<< QNode::subscriber_joint2 <<";"<< QNode::subscriber_joint3 <<";"
           << QNode::subscriber_joint4 <<";"<< QNode::subscriber_joint5 <<";"<<endl;
        file.close();
    }

    else cout << "Dostep do pliku zostal zabroniony!" << endl;
    // cout<< QNode::x;
    std_msgs::String msg;
    std::stringstream ss;
    ss << point_number+1;
    msg.data = ss.str();
    qnode.log(qnode.Info,std::string("Zapisano punkt P")+msg.data);

    //qnode.log(qnode.Info,std::string("Wczytano listę punktów"));
    qnode.readPointsFromFile();
    qnode.loadPointsList();
}


void MainWindow::on_edit_list_clicked(bool check)
{
    //system("gnome-terminal -x sh -c 'cd ~/youbot ; gedit punkty.txt'"); //dodatkowo odpala terminal
    system("bash -c ''cd ~/youbot ; gedit punkty.txt''");   //nie odpala terminala

    //qnode.log(qnode.Info,std::string("Wczytano listę punktów"));
    qnode.readPointsFromFile();
    qnode.loadPointsList();
}

void MainWindow::on_x_plus_clicked(bool check)
{
    double *q;
    double x_temp=QNode::x;
    x_temp=x_temp+xyz_step;
    //q = qnode.inverseKinematic(x_temp, QNode::y, QNode::z, QNode::roll, QNode::pitch, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(x_temp, QNode::y, QNode::z, QNode::roll, QNode::pitch, QNode::yaw, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}
void MainWindow::on_x_minus_clicked(bool check)
{
    double *q;
    double x_temp=QNode::x;
    x_temp=x_temp-xyz_step;
    //q = qnode.inverseKinematic(x_temp, QNode::y, QNode::z, QNode::roll, QNode::pitch, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(x_temp, QNode::y, QNode::z, QNode::roll, QNode::pitch, QNode::yaw, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}
void MainWindow::on_y_plus_clicked(bool check)
{
    double *q;
    double y_temp=QNode::y;
    y_temp=y_temp+xyz_step;
    //q = qnode.inverseKinematic(QNode::x,y_temp, QNode::z, QNode::roll, QNode::pitch, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(QNode::x,y_temp, QNode::z, QNode::roll, QNode::pitch, QNode::yaw, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}
void MainWindow::on_y_minus_clicked(bool check)
{
    double *q;
    double y_temp=QNode::y;
    y_temp=y_temp-xyz_step;
    //q = qnode.inverseKinematic(QNode::x, y_temp, QNode::z, QNode::roll, QNode::pitch, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(QNode::x, y_temp, QNode::z, QNode::roll, QNode::pitch, QNode::yaw, true , false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}
void MainWindow::on_z_plus_clicked(bool check)
{
    double *q;
    double z_temp=QNode::z;
    z_temp=z_temp+xyz_step;
    //q = qnode.inverseKinematic(QNode::x, QNode::y, z_temp, QNode::roll, QNode::pitch, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(QNode::x, QNode::y, z_temp, QNode::roll, QNode::pitch, QNode::yaw, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}
void MainWindow::on_z_minus_clicked(bool check)
{
    double *q;
    double z_temp=QNode::z;
    z_temp=z_temp-xyz_step;
    //q = qnode.inverseKinematic(QNode::x, QNode::y, z_temp, QNode::roll, QNode::pitch, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(QNode::x, QNode::y, z_temp, QNode::roll, QNode::pitch, QNode::yaw, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}

void MainWindow::on_roll_plus_clicked(bool check)
{
    double *q;
    double roll_temp=QNode::roll;
    roll_temp=roll_temp+0.1;
    //q = qnode.inverseKinematic(QNode::x, QNode::y, QNode::z, roll_temp, QNode::pitch, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(QNode::x, QNode::y, QNode::z, roll_temp, QNode::pitch, QNode::yaw, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}
void MainWindow::on_roll_minus_clicked(bool check)
{
    double *q;
    double roll_temp=QNode::roll;
    roll_temp=roll_temp-0.1;
    //q = qnode.inverseKinematic(QNode::x, QNode::y, QNode::z, roll_temp, QNode::pitch, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(QNode::x, QNode::y, QNode::z, roll_temp, QNode::pitch, QNode::yaw, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}
void MainWindow::on_pitch_plus_clicked(bool check)
{
    double *q;
    double pitch_temp=QNode::pitch;
    pitch_temp=pitch_temp+0.1;
    //q = qnode.inverseKinematic(QNode::x, QNode::y, QNode::z, QNode::roll, pitch_temp, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(QNode::x, QNode::y, QNode::z, QNode::roll, pitch_temp, QNode::yaw, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}
void MainWindow::on_pitch_minus_clicked(bool check)
{
    double *q;
    double pitch_temp=QNode::pitch;
    pitch_temp=pitch_temp-0.1;
    //q = qnode.inverseKinematic(QNode::x, QNode::y, QNode::z, QNode::roll, pitch_temp, QNode::yaw, true);
    q = qnode.inverseKinematicJacobi(QNode::x, QNode::y, QNode::z, QNode::roll, pitch_temp, QNode::yaw, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);
}
void MainWindow::on_yaw_plus_clicked(bool check)
{
    double *q;
    double yaw_temp=QNode::yaw;
    yaw_temp=yaw_temp+0.1;
    //qnode.inverseKinematics(QNode::x, QNode::y, QNode::z, QNode::roll, QNode::pitch, yaw_temp, true);
    qnode.inverseKinematicJacobi(QNode::x, QNode::y, QNode::z, QNode::roll, QNode::pitch, yaw_temp, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);

}
void MainWindow::on_yaw_minus_clicked(bool check)
{
    double *q;
    double yaw_temp=QNode::yaw;
    yaw_temp=yaw_temp-0.1;
    //qnode.inverseKinematics(QNode::x, QNode::y, QNode::z, QNode::roll, QNode::pitch, yaw_temp, true);
    qnode.inverseKinematicJacobi(QNode::x, QNode::y, QNode::z, QNode::roll, QNode::pitch, yaw_temp, true, false);
    qnode.jointPublisher(q[0],q[1],q[2],q[3],q[4]);

}
void MainWindow::on_q1_plus_clicked(bool check)
{
   // if(joint_1 < max_1 - (((max_1 - min_1)/100)*joints_step))
    if (joint_1 < max_1 - joints_step)
    {
        //joint_1 = joint_1 + (((max_1 - min_1)/100)*joints_step);
        joint_1 = joint_1 + joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::max_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 1"));
    }
}

void MainWindow::on_q1_minus_clicked(bool check)
{
   // if(joint_1 > min_1 + (((max_1 - min_1)/100)*joints_step))
        if (joint_1 > min_1 + joints_step)
    {
   //     joint_1 = joint_1 - (((max_1 - min_1)/100)*joints_step);
        joint_1 = joint_1 - joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::min_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 1"));
    }
}

void MainWindow::on_q2_plus_clicked(bool check)
{
   // if(joint_2 < max_2 - (((max_2 - min_2)/100)*joints_step))
        if (joint_2 < max_2 - joints_step)
    {
      //  joint_2 = joint_2 + (((max_2 - min_2)/100)*joints_step);
        joint_2 = joint_2 + joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::max_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 2"));
    }
}

void MainWindow::on_q2_minus_clicked(bool check)
{
    //if(joint_2 > min_2 + (((max_2 - min_2)/100)*joints_step))
        if (joint_2 > min_2 + joints_step)
    {
       // joint_2 = joint_2 - (((max_2 - min_2)/100)*joints_step);
        joint_2 = joint_2 - joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::min_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 2"));
    }
}

void MainWindow::on_q3_plus_clicked(bool check)
{
  //  if(joint_3 < max_3 - (((max_3 - min_3)/100)*joints_step))
      if (joint_3 < max_3 - joints_step)
    {
        //joint_3 = joint_3 + (((max_3 - min_3)/100)*joints_step);
        joint_3 = joint_3 + joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::max_3,MainWindow::joint_4,MainWindow::joint_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 3"));
    }
}

void MainWindow::on_q3_minus_clicked(bool check)
{
    //if(joint_3 > min_3 + (((max_3 - min_3)/100)*joints_step))
    if (joint_3 > min_3 + joints_step)
    {
        //joint_3 = joint_3 - (((max_3 - min_3)/100)*joints_step);
        joint_3 = joint_3 - joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::min_3,MainWindow::joint_4,MainWindow::joint_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 3"));
    }
}

void MainWindow::on_q4_plus_clicked(bool check)
{
    //if(joint_4 < max_4 - (((max_4 - min_4)/100)*joints_step))
      if (joint_4 < max_4 - joints_step)
    {
       // joint_4 = joint_4 + (((max_4 - min_4)/100)*joints_step);
        joint_4 = joint_4 + joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::max_4,MainWindow::joint_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 4"));
    }
}

void MainWindow::on_q4_minus_clicked(bool check)
{
   // if(joint_4 > min_4 + (((max_4 - min_4)/100)*joints_step))
      if (joint_4 > min_4 + joints_step)
    {
       // joint_4 = joint_4 - (((max_4 - min_4)/100)*joints_step);
        joint_4 = joint_4 - joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::min_4,MainWindow::joint_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 4"));
    }
}

void MainWindow::on_q5_plus_clicked(bool check)
{
  //  if(joint_5 < max_5 - (((max_5 - min_5)/100)*joints_step))
      if (joint_5 < max_5 - joints_step)
    {
        //joint_5 = joint_5 + (((max_5 - min_5)/100)*joints_step);
        joint_5 = joint_5 + joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::max_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 5"));
    }
}

void MainWindow::on_q5_minus_clicked(bool check)
{
    //if(joint_5 > min_5 + (((max_5 - min_5)/100)*joints_step))
      if (joint_5 > min_5 + joints_step)
    {
        //joint_5 = joint_5 - (((max_5 - min_5)/100)*joints_step);
        joint_5 = joint_5 - joints_step;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
    }
    else
    {
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::min_5);
        qnode.log(qnode.Info,std::string("Osiągnięto skrajną pozycję złącza 5"));
    }
}

void MainWindow::on_gripper_open_clicked(bool check)
{
    qnode.gripperPublisher(0.011, 0.011);
}

void MainWindow::on_gripper_close_clicked(bool check)
{

    qnode.gripperPublisher(0, 0);
}

void MainWindow::on_elbow_up_clicked(bool check)
{
    QNode::elbow_state=false;
}

void MainWindow::on_elbow_down_clicked(bool check)
{
    QNode::elbow_state=true;
}

void MainWindow::on_radio_button_cords_clicked(bool check)
{
    QNode::points_list_view_mode=true;

    //qnode.log(qnode.Info,std::string("Wczytano listę punktów"));
    qnode.readPointsFromFile();
    qnode.loadPointsList();
}

void MainWindow::on_radio_button_joints_clicked(bool check)
{
    QNode::points_list_view_mode=false;

    //qnode.log(qnode.Info,std::string("Wczytano listę punktów"));
    qnode.readPointsFromFile();
    qnode.loadPointsList();
}

void MainWindow::updateLoggingView() {
    ui.view_logging->scrollToBottom();

}

void MainWindow::updateListView() {
    ui.points_list->scrollToBottom();

}

void MainWindow::closeEvent (QCloseEvent *event)
{
    if(qnode.isHomePositionAchived())
    {
        event->accept();
    }
    else
    {
        //        QMessageBox msgBox;
        //        msgBox.setText(tr("Confirm?"));
        //        QAbstractButton* pButtonYes = msgBox.addButton(tr("Yeah!"), QMessageBox::YesRole);


        //        const QMessageBox::StandardButton resBtn = QMessageBox::question( this, tr("youBot Arm GUI"),
        //                                                                    tr("Robot nie jest w pozycji domowej!\n"),
        //                                                                    QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes| QMessageBox::YesRole,
        //                                                                    QMessageBox::Yes);
        //        if (resBtn != QMessageBox::Yes)
        //        {
        //            event->ignore();
        //        }
        //        else
        //        {
        //            event->accept();
        //        }

        QMessageBox msgBox;
        msgBox.setText(tr("Robot nie jest w pozycji domowej!"));
        QAbstractButton* cancelButton = msgBox.addButton(tr("Anuluj"), QMessageBox::YesRole);
        QAbstractButton* homeButton = msgBox.addButton(tr("Pozycja domowa"), QMessageBox::NoRole);

        msgBox.exec();


        if (msgBox.clickedButton()==homeButton)
        {
            qnode.moveHome();
        }

        if (msgBox.clickedButton()==cancelButton)
        {
            event->ignore();
        }

    }

}

}  // namespace youbot_gui

