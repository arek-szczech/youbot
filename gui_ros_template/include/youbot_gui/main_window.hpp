/**
 * @file /include/youbot_gui/main_window.hpp
 *
 * @brief Qt based gui for youbot_gui.
 *
 * @date December 2017
 **/

#ifndef youbot_gui_MAIN_WINDOW_H
#define youbot_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace youbot_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event);

	void showQ1PlusMsg();
	void showQ1MinusMsg();
	void runYoubotDriver();

public Q_SLOTS:
	void on_q1_plus_clicked(bool check);
	void on_q1_minus_clicked(bool check);	
	void on_run_driver_clicked(bool check);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace youbot_gui

#endif // youbot_gui_MAIN_WINDOW_H
