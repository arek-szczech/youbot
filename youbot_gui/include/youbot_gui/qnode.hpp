/**
 * @file /include/youbot_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date December 2017
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef youbot_gui_QNODE_HPP_
#define youbot_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/


#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>


#include <QtWidgets/QMainWindow>
#include <QLCDNumber>
#include "ui_main_window.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace youbot_gui {

/*****************************************************************************
** Class
*****************************************************************************/

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

        static double ik_th1;
        static double ik_th2;
        static double ik_th3;
        static double ik_th4;
        static double ik_th5;

        static double list_x;
        static double list_y;
        static double list_z;
        static double list_roll;
        static double list_pitch;
        static double list_yaw;

        static double x_temp;
        static double y_temp;
        static double z_temp;
        static double roll_temp;
        static double pitch_temp;
        static double yaw_temp;

        static int program_state;
        static int program_line_number;
        static int point[100];
        static bool back_to_home;
        static bool execute_movement_flag;
        static int movement_iteration;

        Ui::MainWindowDesign ui;


	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
        void run();
        void executeProgram();
        void forwardKinematic(double q1, double q2,double q3,double q4,double q5);
        void readPoints();
        std::string showPoint(int i);
        void ptp(double q1, double q2,double q3,double q4,double q5);
        void loadPointsList();
        void readProgram();
        void jointPublisher(double q1, double q2,double q3,double q4,double q5);
        void moveArm(double q1, double q2,double q3,double q4,double q5);
        void executePTP(int i);
        void inverseKinematics(double xk, double yk, double zk, double Rz, double Ry, double Rx);
        int sgn(double v);
       // void chatterCallback(const brics_actuator::JointPositionsConstPtr& youbotArmCommand);

	/*********************
	** Logging
	**********************/
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
        void list(const std::string &msg);
        void addToList(std::string bufor);


Q_SIGNALS:
	void loggingUpdated();
    	void rosShutdown();
        void listUpdated();

private:
	int init_argc;
	char** init_argv;
        //ros::Publisher chatter_publisher;
        ros::Publisher armPositionsPublisher;
        ros::Publisher gripperPositionPublisher;

        ros::Publisher armPublisher;

        ros::Subscriber armPositionsSubscriber;

    	QStringListModel logging_model;
        QStringListModel list_model;

        //MainWindow obiekt;
};

}  // namespace youbot_gui

#endif /* youbot_gui_QNODE_HPP_ */
