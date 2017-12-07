/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date November 2017
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

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
/*****************************************************************************
** Namespaces
*****************************************************************************/





namespace youbot_gui {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

double min_1 = 0.0100692;
double max_1 = 5.84014;
//double MainWindow::joint_1 = 0.0100692;
//static double joint_1 = 0.0100692;
double MainWindow::joint_1 = QNode::subscriber_joint1;

double min_2 = 0.0100692;
double max_2 = 2.61799;
//double MainWindow::joint_2 = 0.0100692;
double MainWindow::joint_2 = QNode::subscriber_joint2;

double min_3 = -5.02655;
double max_3 = -0.015708;
//double MainWindow::joint_3 = -0.015708;
double MainWindow::joint_3 = QNode::subscriber_joint3;

double min_4 = 0.0221239;
double max_4 = 3.4292;
//double MainWindow::joint_4 = 0.0221239;
double MainWindow::joint_4 = QNode::subscriber_joint4;

double min_5 = 0.110619;
double max_5 = 5.64159;
//double MainWindow::joint_5 = 0.110619;
double MainWindow::joint_5 = QNode::subscriber_joint5;

double min_6 = 0;
double max_6 = 0.011;
static double gripper_1 = 0;

double min_7 = 0;
double max_7 = 0.011;
static double gripper_2 = 0;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this);
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

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_edit_clicked(bool check)
{

        system("bash -c ''cd ~/youbot ; gedit program.txt''");
}

void MainWindow::on_execute_clicked(bool check)
{
        qnode.executeProgram();
}

void MainWindow::on_pause_clicked(bool check)
{

}

void MainWindow::on_stop_clicked(bool check)
{

}

void MainWindow::on_previous_clicked(bool check)
{
    qnode.readPoints();
    qnode.readProgram();

    cout<<"Przed dek: "<<QNode::program_state<<endl;

    if(QNode::program_state>0)
    {
        cout<<"Prev_prog_line_numb: "<<QNode::program_line_number<<endl;
        if(QNode::program_state==QNode::program_line_number)
        {
            QNode::program_state=QNode::program_state-2;
        }
        else
        {
        QNode::program_state--;
        }
        qnode.executePTP(QNode::program_state);

    }
    cout<<"Po dek: "<<QNode::program_state<<endl;
}

void MainWindow::on_next_clicked(bool check)
{
    qnode.readPoints();
    qnode.readProgram();

    cout<<"Przed ink: "<<QNode::program_state<<endl;

    if(QNode::program_state<QNode::program_line_number)
    {
        cout<<"Next_prog_line_numb: "<<QNode::program_line_number<<endl;
        qnode.executePTP(QNode::program_state);

            QNode::program_state++;

    }
    cout<<"Po ink: "<<QNode::program_state<<endl;
}

void MainWindow::on_home_clicked(bool check)
{

}

void MainWindow::on_run_driver_clicked(bool check)
{
        system("gnome-terminal -x sh -c 'cd ~/youbot ; source devel/setup.bash ; roslaunch youbot_driver_ros_interface youbot_driver.launch'");
}

void MainWindow::on_connect_master_clicked(bool check)
{
        qnode.init();
        qnode.readPoints();
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
}


void MainWindow::on_edit_list_clicked(bool check)
{
        //system("gnome-terminal -x sh -c 'cd ~/youbot ; gedit punkty.txt'"); //dodatkowo odpala terminal
        system("bash -c ''cd ~/youbot ; gedit punkty.txt''");   //nie odpala terminala
}
void  MainWindow::on_load_list_clicked(bool check)
{
        qnode.readPoints();
        qnode.loadPointsList();
}

void MainWindow::on_x_plus_clicked(bool check)
{
    QNode::x_temp=QNode::x;
    QNode::x_temp=QNode::x_temp+1;
    cout<<QNode::x_temp<<endl;

    qnode.inverseKinematics(QNode::x_temp, QNode::y, QNode::z, QNode::roll, QNode::pitch, QNode::yaw);

    //armJointPositions[0].value = theta_1 + offset1;
}
void MainWindow::on_x_minus_clicked(bool check)
{
    QNode::x_temp=QNode::x;
    QNode::x_temp=QNode::x_temp-1;
    cout<<QNode::x_temp<<endl;
    qnode.inverseKinematics(QNode::x_temp, QNode::y, QNode::z, QNode::roll, QNode::pitch, QNode::yaw);
}
void MainWindow::on_y_plus_clicked(bool check)
{
    QNode::y_temp=QNode::y;
    QNode::y_temp=QNode::y_temp+1;
    cout<<QNode::y_temp<<endl;
    qnode.inverseKinematics(QNode::x, QNode::y_temp, QNode::z, QNode::roll, QNode::pitch, QNode::yaw);
}
void MainWindow::on_y_minus_clicked(bool check)
{
    QNode::y_temp=QNode::y;
    QNode::y_temp=QNode::y_temp-1;
    cout<<QNode::y_temp<<endl;
    qnode.inverseKinematics(QNode::x, QNode::y_temp, QNode::z, QNode::roll, QNode::pitch, QNode::yaw);
}
void MainWindow::on_z_plus_clicked(bool check)
{
    QNode::z_temp=QNode::z;
    QNode::z_temp=QNode::z_temp+1;
    cout<<QNode::z_temp<<endl;
    qnode.inverseKinematics(QNode::x, QNode::y, QNode::z_temp, QNode::roll, QNode::pitch, QNode::yaw);
}
void MainWindow::on_z_minus_clicked(bool check)
{
    QNode::z_temp=QNode::z;
    QNode::z_temp=QNode::z_temp-1;
    cout<<QNode::z_temp<<endl;
    qnode.inverseKinematics(QNode::x, QNode::y, QNode::z_temp, QNode::roll, QNode::pitch, QNode::yaw);
}

void MainWindow::on_q1_plus_clicked(bool check)
{
        if(joint_1 < max_1 - ((max_1 - min_1)/100))
        joint_1 = joint_1 + (max_1 - min_1)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}

void MainWindow::on_q1_minus_clicked(bool check)
{
        if(joint_1 > min_1 + ((max_1 - min_1)/100))
        joint_1 = joint_1 - (max_1 - min_1)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}

void MainWindow::on_q2_plus_clicked(bool check)
{
        if(joint_2 < max_2 - ((max_2 - min_2)/100))
        joint_2 = joint_2 + (max_2 - min_2)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}

void MainWindow::on_q2_minus_clicked(bool check)
{
        if(joint_2 > min_2 + ((max_2 - min_2)/100))
        joint_2 = joint_2 - (max_2 - min_2)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}

void MainWindow::on_q3_plus_clicked(bool check)
{
        if(joint_3 < max_3 - ((max_3 - min_3)/100))
        joint_3 = joint_3 + (max_3 - min_3)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}

void MainWindow::on_q3_minus_clicked(bool check)
{
        if(joint_3 > min_3 + ((max_3 - min_3)/100))
        joint_3 = joint_3 - (max_3 - min_3)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}

void MainWindow::on_q4_plus_clicked(bool check)
{
        if(joint_4 < max_4 - ((max_4 - min_4)/100))
        joint_4 = joint_4 + (max_4 - min_4)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}

void MainWindow::on_q4_minus_clicked(bool check)
{
        if(joint_4 > min_4 + ((max_4 - min_4)/100))
        joint_4 = joint_4 - (max_4 - min_4)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}

void MainWindow::on_q5_plus_clicked(bool check)
{
        if(joint_5 < max_5 - ((max_5 - min_5)/100))
        joint_5 = joint_5 + (max_5 - min_5)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}

void MainWindow::on_q5_minus_clicked(bool check)
{
        if(joint_5 > min_5 + ((max_5 - min_5)/100))
        joint_5 = joint_5 - (max_5 - min_5)/100;
        qnode.jointPublisher(MainWindow::joint_1, MainWindow::joint_2,MainWindow::joint_3,MainWindow::joint_4,MainWindow::joint_5);
}



void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();

}

void MainWindow::updateListView() {
        ui.points_list->scrollToBottom();

}

//komentarz
/*****************************************************************************
** Implementation [Close]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace youbot_gui

