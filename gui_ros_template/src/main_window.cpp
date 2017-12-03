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
double MainWindow::joint_1 = 0.0100692;
//static double joint_1 = 0.0100692;

double min_2 = 0.0100692;
double max_2 = 2.61799;
double MainWindow::joint_2 = 0.0100692;

double min_3 = -5.02655;
double max_3 = -0.015708;
double MainWindow::joint_3 = -0.015708;

double min_4 = 0.0221239;
double max_4 = 3.4292;
double MainWindow::joint_4 = 0.0221239;

double min_5 = 0.110619;
double max_5 = 5.64159;
double MainWindow::joint_5 = 0.110619;

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
    	QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

	setWindowIcon(QIcon(":/images/icon.png"));

    	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));


        /*********************
        ** Logging
        **********************/
        ui.view_logging->setModel(qnode.loggingModel());
        QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));




}

/*MainWindow::MainWindow()//: QMainWindow(parent), qnode(argc,argv)
{}*/

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showQ1PlusMsg()
{
	QMessageBox msgBox;
	msgBox.setText("Zwiekszam q1");
	msgBox.exec();
}

void MainWindow::showQ1MinusMsg()
{
	QMessageBox msgBox;
	msgBox.setText("Zmniejszam q1");
	msgBox.exec();
}


void MainWindow::runYoubotDriver()
{
        system("gnome-terminal -x sh -c 'cd ~/youbot ; source devel/setup.bash ; roslaunch youbot_driver_ros_interface youbot_driver.launch'");
}

void MainWindow::on_run_driver_clicked(bool check)
{
        runYoubotDriver();
}

void MainWindow::on_connect_master_clicked(bool check)
{
        qnode.init();
}

void MainWindow::on_fold_clicked(bool check)
{
    ui.lcd_q1->display(QNode::subscriber_joint1);
    ui.lcd_q2->display(QNode::subscriber_joint2);
    ui.lcd_q3->display(QNode::subscriber_joint3);
    ui.lcd_q4->display(QNode::subscriber_joint4);
    ui.lcd_q5->display(QNode::subscriber_joint5);
}

void MainWindow::on_candle_clicked(bool check)
{

}

void MainWindow::on_save_clicked(bool check)
{
        fstream plik;
        plik.open( "punkty.txt", ios::in | ios::out | ios::app);
            if( plik.good() == true )
            {
                cout << "Uzyskano dostep do pliku!" << endl;
                plik << "P0;1;1.1;1.1;1.1;1.1;" << endl;
                plik.close();
            }

            else cout << "Dostep do pliku zostal zabroniony!" << endl;
      // cout<< QNode::x;
}


void MainWindow::on_edit_list_clicked(bool check)
{
        //system("gnome-terminal -x sh -c 'cd ~/youbot ; gedit punkty.txt'"); //dodatkowo odpala terminal
        system("bash -c ''cd ~/youbot ; gedit punkty.txt''");   //nie odpala terminala
}
void MainWindow::on_edit_clicked(bool check)
{

        system("bash -c ''cd ~/youbot ; gedit program.txt''");
}
void MainWindow::on_execute_clicked(bool check)
{

}
void MainWindow::on_x_plus_clicked(bool check)
{
//lcd_q1->display(1.0);
        //ui.lcd_q1->display("1.0"); //dziala
       // ui.lcd_q1->display(MainWindow::joint_1); //dziala ale pobiera wartosci ze zmiennej, a nie z subscribera

//ui.lcd_q1->display(QNode::subscriber_joint1); dziala najlepiej
}
void MainWindow::on_x_minus_clicked(bool check)
{

}
void MainWindow::on_y_plus_clicked(bool check)
{

}
void MainWindow::on_y_minus_clicked(bool check)
{

}
void MainWindow::on_z_plus_clicked(bool check)
{

}
void MainWindow::on_z_minus_clicked(bool check)
{

}

void MainWindow::on_q1_plus_clicked(bool check)
{
        if(joint_1 < max_1 - ((max_1 - min_1)/100))
        joint_1 = joint_1 + (max_1 - min_1)/100;
}

void MainWindow::on_q1_minus_clicked(bool check)
{
        if(joint_1 > min_1 + ((max_1 - min_1)/100))
        joint_1 = joint_1 - (max_1 - min_1)/100;
}

void MainWindow::on_q2_plus_clicked(bool check)
{
        if(joint_2 < max_2 - ((max_2 - min_2)/100))
        joint_2 = joint_2 + (max_2 - min_2)/100;
}

void MainWindow::on_q2_minus_clicked(bool check)
{
        if(joint_2 > min_2 + ((max_2 - min_2)/100))
        joint_2 = joint_2 - (max_2 - min_2)/100;
}

void MainWindow::on_q3_plus_clicked(bool check)
{
        if(joint_3 < max_3 - ((max_3 - min_3)/100))
        joint_3 = joint_3 + (max_3 - min_3)/100;
}

void MainWindow::on_q3_minus_clicked(bool check)
{
        if(joint_3 > min_3 + ((max_3 - min_3)/100))
        joint_3 = joint_3 - (max_3 - min_3)/100;
}

void MainWindow::on_q4_plus_clicked(bool check)
{
        if(joint_4 < max_4 - ((max_4 - min_4)/100))
        joint_4 = joint_4 + (max_4 - min_4)/100;
}

void MainWindow::on_q4_minus_clicked(bool check)
{
        if(joint_4 > min_4 + ((max_4 - min_4)/100))
        joint_4 = joint_4 - (max_4 - min_4)/100;
}

void MainWindow::on_q5_plus_clicked(bool check)
{
        if(joint_5 < max_5 - ((max_5 - min_5)/100))
        joint_5 = joint_5 + (max_5 - min_5)/100;
}

void MainWindow::on_q5_minus_clicked(bool check)
{
        if(joint_5 > min_5 + ((max_5 - min_5)/100))
        joint_5 = joint_5 - (max_5 - min_5)/100;
}

/*void MainWindow::refresh_value(bool check)
{
ui.lcd_q1->display(QNode::subscriber_joint1);
ui.lcd_q2->display(QNode::subscriber_joint2);
ui.lcd_q3->display(QNode::subscriber_joint3);
ui.lcd_q4->display(QNode::subscriber_joint4);
ui.lcd_q5->display(QNode::subscriber_joint5);
cout<<"gówno"<<endl;
} */

void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
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

