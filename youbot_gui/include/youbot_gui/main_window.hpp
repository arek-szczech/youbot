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

#ifndef youbot_gui_MAIN_WINDOW_H
#define youbot_gui_MAIN_WINDOW_H

#include <QtWidgets/QMainWindow>
#include <QLCDNumber>
#include "ui_main_window.h"
#include "qnode.hpp"

namespace youbot_gui {

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void closeEvent(QCloseEvent *event);

    static double joint_1;
    static double joint_2;
    static double joint_3;
    static double joint_4;
    static double joint_5;

    static double min_1;
    static double min_2;
    static double min_3;
    static double min_4;
    static double min_5;

    static double max_1;
    static double max_2;
    static double max_3;
    static double max_4;
    static double max_5;

    static int xyz_step;
    static double joints_step;

public Q_SLOTS:
    void on_edit_clicked(bool check);
    void on_execute_clicked(bool check);
    void on_pause_clicked(bool check);
    void on_stop_clicked(bool check);
    void on_previous_clicked(bool check);
    void on_next_clicked(bool check);
    void on_home_clicked(bool check);

    void on_run_driver_clicked(bool check);
    void on_connect_master_clicked(bool check);

    void on_save_clicked(bool check);
    void on_edit_list_clicked(bool check);

    void on_x_plus_clicked(bool check);
    void on_x_minus_clicked(bool check);
    void on_y_plus_clicked(bool check);
    void on_y_minus_clicked(bool check);
    void on_z_plus_clicked(bool check);
    void on_z_minus_clicked(bool check);

    void on_roll_plus_clicked(bool check);
    void on_roll_minus_clicked(bool check);
    void on_pitch_plus_clicked(bool check);
    void on_pitch_minus_clicked(bool check);
    void on_yaw_plus_clicked(bool check);
    void on_yaw_minus_clicked(bool check);

    void on_q1_plus_clicked(bool check);
    void on_q1_minus_clicked(bool check);
    void on_q2_plus_clicked(bool check);
    void on_q2_minus_clicked(bool check);
    void on_q3_plus_clicked(bool check);
    void on_q3_minus_clicked(bool check);
    void on_q4_plus_clicked(bool check);
    void on_q4_minus_clicked(bool check);
    void on_q5_plus_clicked(bool check);
    void on_q5_minus_clicked(bool check);

    void on_gripper_open_clicked(bool check);
    void on_gripper_close_clicked(bool check);

    void on_elbow_up_clicked(bool check);
    void on_elbow_down_clicked(bool check);

    void on_radio_button_cords_clicked(bool check);
    void on_radio_button_joints_clicked(bool check);

    void updateLoggingView();
    void updateListView();



private:
    Ui::MainWindowDesign ui;
    QNode qnode;
};

}  // namespace youbot_gui

#endif // youbot_gui_MAIN_WINDOW_H
