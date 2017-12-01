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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/youbot_gui/main_window.hpp"
#include <stdlib.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace youbot_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this);
    	QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));

	setWindowIcon(QIcon(":/images/icon.png"));

    	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

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

void MainWindow::on_q1_plus_clicked(bool check)
{
	showQ1PlusMsg();
}

void MainWindow::on_q1_minus_clicked(bool check)
{
	showQ1MinusMsg();
}

void MainWindow::runYoubotDriver()
{
        system("gnome-terminal -x sh -c 'cd ~/youbot ; source devel/setup.bash ; roslaunch youbot_driver_ros_interface youbot_driver.launch'");
}

void MainWindow::on_run_driver_clicked(bool check)
{
	runYoubotDriver();
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

