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
//#include "diagnostic_msgs/DiagnosticArray.h"
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

        static double subscriber_gripper_1;
        static double subscriber_gripper_2;

        static double ik_th1;
        static double ik_th2;
        static double ik_th3;
        static double ik_th4;
        static double ik_th5;

        static double lin_x;
        static double lin_y;
        static double lin_z;
        static double lin_roll;
        static double lin_pitch;
        static double lin_yaw;

        static double lin_x_actual;
        static double lin_y_actual;
        static double lin_z_actual;
        static double lin_roll_actual;
        static double lin_pitch_actual;
        static double lin_yaw_actual;

        static int program_state;
        static int program_line_number;
        static int point[100];
        static std::string command[100];
        static bool back_to_home;
        static bool execute_movement_flag;
        static int movement_iteration;
        //static double P[100][5];
        static double home[5];
        static bool play_program;
        static bool ethercat_connection;
        static bool ethercat_connection_temp;
        static bool ethercat_connection_temp2;
        static bool elbow_state;

        static bool opening_gripper;
        static bool closing_gripper;



        Ui::MainWindowDesign ui;


	QNode(int argc, char** argv );
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
        double* inverseKinematic(double xk, double yk, double zk, double Rz, double Ry, double Rx);
        int sgn(double v);
        void moveHome();
        void jointSimulator(int i);
        bool isPositionAchived(int movement_iteration_temp);
        void lin(double q1, double q2,double q3,double q4,double q5);
        void executeLIN(int i);
        void specialInverseKinematics(double xk, double yk, double zk, double Rz, double Ry, double Rx);
        bool checkLinearMovementPossibility(double q1, double q2,double q3,double q4,double q5);
        bool isHomePositionAchived();
        bool isGripperPositionAchived(double state);

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

        ros::Publisher simulatorArmPositionsPublisher;
        ros::Publisher jointsPublisher;

        ros::Subscriber jointsSubscriber;
        ros::Subscriber diagnosticsSubscriber;


    	QStringListModel logging_model;
        QStringListModel list_model;

        ros::Time currentTime;

        //MainWindow obiekt;
};

}  // namespace youbot_gui

#endif /* youbot_gui_QNODE_HPP_ */
