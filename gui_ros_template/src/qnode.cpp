/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date December 2017
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/youbot_gui/main_window.hpp" //Kolejność includów jest krytyczna xD Musi być na górze
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
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
#include <iostream>
#include <assert.h>
#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>
#include <signal.h>
#include <curses.h>
#include <fstream>
#include <stdlib.h>
#include <cstdlib>

#include "../include/youbot_gui/qnode.hpp"
//#include "../include/youbot_gui/mainwindow.hpp"
//#include "../include/youbot_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
namespace youbot_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

double QNode::subscriber_joint1;
double QNode::subscriber_joint2;
double QNode::subscriber_joint3;
double QNode::subscriber_joint4;
double QNode::subscriber_joint5;

double QNode::x;
double QNode::y;
double QNode::z;
double QNode::roll;
double QNode::pitch;
double QNode::yaw;

double QNode::pkt_x;
double QNode::pkt_y;
double QNode::pkt_z;
double QNode::pkt_roll;
double QNode::pkt_pitch;
double QNode::pkt_yaw;


//*********EXECUTE_PROGRAM****************

string line[100];
int line_nmb=0;
int stan=0;
double q1[100];
double q2[100];
double q3[100];
double q4[100];
double q5[100];
double P[100][5];

size_t pos1,pos2,pos3,pos4,pos5,pos6;
fstream punkty;

//****************************************


//MainWindow obiekt = new MainWindow();
//youbot_gui::MainWindow obiekt(int argc, char** argv); //dziala prawie

void QNode::kin_prosta(double q1, double q2,double q3,double q4,double q5)
{
//cout << q2[1] << endl;

double th_1=q1;
double th_2=q2;
double th_3=q3;
double th_4=q4;
double th_5=q5;


double th1 = th_1 - 2.8668;
double th2 = th_2 - 2.5919;
double th3 = th_3 + 2.5211;
double th4 = th_4 - 3.3305;
double th5 = th_5 - 2.9314;


double a1 = 0.033;
double d1 = 0.147;
double a2 = 0.155;
double a3 = 0.135;
double d5 = 0.218;

QNode::pkt_x = a1*cos(th1) - d5*(cos(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3);
QNode::pkt_y = a1*sin(th1) - d5*(cos(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) + a2*cos(th2)*sin(th1) + a3*cos(th2)*cos(th3)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3);
QNode::pkt_z = d1 - a2*sin(th2) - d5*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))) - a3*cos(th2)*sin(th3) - a3*cos(th3)*sin(th2);
QNode::pkt_roll = atan2(- cos(th1)*sin(th5) - cos(th5)*(cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2))), sin(th1)*sin(th5) - cos(th5)*(cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2))));
QNode::pkt_pitch = atan2(cos(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sqrt(sin(th5)*sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)))*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))) + (cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))));
QNode::pkt_yaw = atan2(sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) - cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)));

}

void QNode::wyswietl_pkt()
{
        for (int i=0; i<line_nmb; i++)
        {
                kin_prosta(q1[i],q2[i],q3[i],q4[i],q5[i]);
                cout << "P" << i << ":  x: " << pkt_x << " y: " << pkt_y << " z: " << pkt_z
                     <<  " roll: " << pkt_roll << " pitch: " << pkt_pitch << " yaw: " << pkt_yaw <<endl;

        }
}

void QNode::ptp(double q1, double q2,double q3,double q4,double q5)
{
//      brics_actuator::JointPositions command;
//      vector <brics_actuator::JointValue> armJointPositions;
//      vector <brics_actuator::JointValue> gripperJointPositions;

//      armJointPositions.resize(numberOfArmJoints); //TODO:change that
//      gripperJointPositions.resize(numberOfGripperJoints);

//      std::stringstream jointName;


//      armJointPositions[0].value = q1;
//      armJointPositions[1].value = q2;
//      armJointPositions[2].value = q3;
//      armJointPositions[3].value = q4;
//      armJointPositions[4].value = q5;
//      //gripperJointPositions[0].value = gripper_1;
//      //gripperJointPositions[1].value = gripper_2;
//      for (int i = 0; i < numberOfArmJoints; ++i) {
//      //cout << "Please type in value for joint " << i + 1 << endl;
//      //cin >> readValue;

//      jointName.str("");
//      jointName << "arm_joint_" << (i + 1);

//      armJointPositions[i].joint_uri = jointName.str();
//      //armJointPositions[i].value = readValue;

//      armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
//      //cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;
//      };
}


void chatterCallback(const brics_actuator::JointPositionsConstPtr& youbotArmCommand)
{


        // Wartosci odczytane z robota
        double th_1 = youbotArmCommand->positions[0].value;
        double th_2 = youbotArmCommand->positions[1].value;
        double th_3 = youbotArmCommand->positions[2].value;
        double th_4 = youbotArmCommand->positions[3].value;
        double th_5 = youbotArmCommand->positions[4].value;


        double th1 = th_1 - 2.8668;
        double th2 = th_2 - 2.5919;
        double th3 = th_3 + 2.5211;
        double th4 = th_4 - 3.3305;
        double th5 = th_5 -	2.9314;

/*	cout << "Wartosci wysyłane do robota: " << endl;
        cout << "th_1: " << th_1 << endl;
        cout << "th_2: " << th_2 << endl;
        cout << "th_3: " << th_3 << endl;
        cout << "th_4: " << th_4 << endl;
        cout << "th_5: " << th_5 << endl;
        cout << "" << endl;
        cout << "Wartosci uzywane do liczenia fk: " << endl;
        cout << "th1: " << th1 << endl;
        cout << "th2: " << th2 << endl;
        cout << "th3: " << th3 << endl;
        cout << "th4: " << th4 << endl;
        cout << "th5: " << th5 << endl;
        cout << "" << endl;*/

        double a1 = 0.033;
        double d1 = 0.147;
        double a2 = 0.155;
        double a3 = 0.135;
        double d5 = 0.218;

        //double x = a1*cos(th1) - d5*(cos(th5)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) + sin(th5)*(cos(th1)*cos(th2)*cos(th3) - cos(th1)*sin(th2)*sin(th3))) + a2*cos(th1)*cos(th2) - a3*cos(th1)*sin(th2)*sin(th3) + a3*cos(th1)*cos(th2)*cos(th3);
        //double y = a1*sin(th1) - d5*(cos(th5)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) + sin(th5)*(cos(th2)*cos(th3)*sin(th1) - sin(th1)*sin(th2)*sin(th3))) + a2*cos(th2)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3) + a3*cos(th2)*cos(th3)*sin(th1);
        //double z = d1 - a2*sin(th2) - d5*(cos(th5)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th5)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))) - a3*cos(th2)*sin(th3) - a3*cos(th3)*sin(th2);

        QNode::x = a1*cos(th1) - d5*(cos(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2)) - sin(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3))) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2)*cos(th3) - a3*cos(th1)*sin(th2)*sin(th3);
        QNode::y = a1*sin(th1) - d5*(cos(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2)) - sin(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1))) + a2*cos(th2)*sin(th1) + a3*cos(th2)*cos(th3)*sin(th1) - a3*sin(th1)*sin(th2)*sin(th3);
        QNode::z = d1 - a2*sin(th2) - d5*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2))) - a3*cos(th2)*sin(th3) - a3*cos(th3)*sin(th2);

        QNode::roll = atan2(- cos(th1)*sin(th5) - cos(th5)*(cos(th4)*(sin(th1)*sin(th2)*sin(th3) - cos(th2)*cos(th3)*sin(th1)) + sin(th4)*(cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2))), sin(th1)*sin(th5) - cos(th5)*(cos(th4)*(cos(th1)*sin(th2)*sin(th3) - cos(th1)*cos(th2)*cos(th3)) + sin(th4)*(cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2))));
        //double pitch = atan2(cos(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sqrt(sin(th5)*sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)))*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))) + (cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))));
        //double yaw = atan2(sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) - cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)));

        QNode::pitch = atan2(cos(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sqrt(sin(th5)*sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)))*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))) + (cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))*(cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)) - sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)))));
        QNode::yaw = atan2(sin(th5)*(cos(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) + sin(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3))), sin(th4)*(cos(th2)*sin(th3) + cos(th3)*sin(th2)) - cos(th4)*(cos(th2)*cos(th3) - sin(th2)*sin(th3)));

//        cout << "Pozycja x: " << QNode::x << endl;
//        cout << "Pozycja y: " << QNode::y << endl;
//        cout << "Pozycja z: " << QNode::z << endl;
//        cout << "Kąt roll: " << QNode::roll << endl;
//        cout << "Kąt pitch: " << QNode::pitch << endl;
//        cout << "Kąt yaw: " << QNode::yaw << endl;
//        cout << "----------" << endl;
       // QNode::subscriber_joint1 = armJointPositions[0].value;

        static const int numberOfArmJoints = 5;
        static const int numberOfGripperJoints = 2;

        brics_actuator::JointPositions command;
            vector <brics_actuator::JointValue> armJointPositions;
         vector <brics_actuator::JointValue> gripperJointPositions;

          armJointPositions.resize(numberOfArmJoints); //TODO:change that
          gripperJointPositions.resize(numberOfGripperJoints);

          std::stringstream jointName;


          QNode::subscriber_joint1 = th_1;
          QNode::subscriber_joint2 = th_2;
          QNode::subscriber_joint3 = th_3;
          QNode::subscriber_joint4 = th_4;
          QNode::subscriber_joint5 = th_5;


          //gripperJointPositions[0].value = gripper_1;
          //gripperJointPositions[1].value = gripper_2;
          for (int i = 0; i < numberOfArmJoints; ++i) {
          //cout << "Please type in value for joint " << i + 1 << endl;
          //cin >> readValue;

          jointName.str("");
          jointName << "arm_joint_" << (i + 1);

          armJointPositions[i].joint_uri = jointName.str();
          //armJointPositions[i].value = readValue;

          armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
          //cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;
          };


}


void konwersjaDoTablicy2d()
{
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

double stringToDouble (string q)
{
std::replace(q.begin(), q.end(), '.', ','); // Na kompilatorze terminalowym czyta kropki, zapisuje jako kropki
double q_d = atof(q.c_str());               // Catkin czyta przecinki, zapisuje kropki - WTF??11!!11
cout << "q: " << q<< endl;
cout << "q_d: " << q_d << endl;

return q_d;
}

void czytaj_zmienne()
{

        while (getline(punkty, line[line_nmb]))
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
        cout << "PKT " << line_nmb+1 << ": q1 = " << q1[line_nmb] << " q2 = " << q2[line_nmb] << " q3 = " << q3[line_nmb] << " q4 = " << q4[line_nmb] << " q5 = " << q5[line_nmb] << endl;
        ++line_nmb;
}

        cout << "Plik ma " << line_nmb << " wierszy" << endl;
}

void QNode::czytajPunkty()
{
    punkty.open( "punkty.txt", ios::in | ios::out | ios::app);
    if( punkty.good() == true )
    {
        cout << "Uzyskano dostep do pliku!" << endl;
        czytaj_zmienne();
        punkty.close();
        konwersjaDoTablicy2d();
    }

    else cout << "Brak dostępu do pliku" << endl;
}

void pkt(string punkt)
{
        cout << " do punktu: "  << punkt << endl;
int pkt = atoi(punkt.c_str());
        cout << "q1 = " << P[pkt][0] << " q2 = " << P[pkt][1] << " q3 = " << P[pkt][2] << " q4 = " << P[pkt][3] << " q5 = " << P[pkt][4] << endl;

}

void czytajProgram()
{

    string temp_kom;
    string wiersz[100];
    int wiersz_nmb=0;
    fstream plik;


    plik.open( "program.txt", ios::in | ios::out | ios::app);
    if( plik.good() == true )
    {
        cout << "Uzyskano dostep do pliku!" << endl;
        while (getline(plik, wiersz[wiersz_nmb]))
        {
        cout << wiersz[wiersz_nmb] << endl;
        ++wiersz_nmb;
        }
        cout << "Plik ma " << wiersz_nmb << " wierszy" << endl;
        plik.close();
    }

    else
    {
        cout << "Brak dostępu do pliku" << endl; // przerwać funkcje
    }
    string temp_pkt[wiersz_nmb];
    string komenda[wiersz_nmb];
    string punkt[wiersz_nmb];

        for (int i = 0; i<wiersz_nmb;  i++)
    {
        komenda[i] = wiersz[i].substr (0,3);
        temp_pkt[i] = wiersz[i].substr (4,1);
        punkt[i] = wiersz[i].substr (5,2);
        if (komenda[i]=="PTP")
            stan++;
        else
            cout << "Nieznana komenda" << endl;
        if (temp_pkt[i]=="P")
            stan++;
        else
            cout << "Błędnie wprowadzony punkt" << endl;
        if (atoi(punkt[i].c_str())<=line_nmb)
            stan++;
        else
            cout << "Punkt P" << punkt[i] << " nie został zdefiniowany" << endl;
    }
        if (stan/3!=wiersz_nmb)
        {
         cout << "Błąd składni kodu" << endl; // wyprowadzić log i przerwać funkcje
        }
        else
        {
         for (int i=0;i<wiersz_nmb;i++)
         pkt(punkt[i]);
        }

   }



void QNode::execute_program()
{
        cout << "GÓWNO" << endl;
        line_nmb=0;
        stan=0;

        czytajPunkty();
        czytajProgram();

}

void QNode::load_points_list()
{

}


QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"youbot_gui");
	if ( ! ros::master::check() ) {
                return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
        //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
        //ros::Publisher armPositionsPublisher;
        //ros::Publisher gripperPositionPublisher;

        armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
        gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

        armPositionsSubscriber = n.subscribe<brics_actuator::JointPositions >("arm_1/arm_controller/position_command", 1, chatterCallback);



        start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"youbot_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
        //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;

}



void QNode::run() {
        ros::Rate loop_rate(4); //zmienione z 1 na 20
	int count = 0;

        static const int numberOfArmJoints = 5;
        static const int numberOfGripperJoints = 2;

	while ( ros::ok() ) {

          brics_actuator::JointPositions command;
              vector <brics_actuator::JointValue> armJointPositions;
           vector <brics_actuator::JointValue> gripperJointPositions;

            armJointPositions.resize(numberOfArmJoints); //TODO:change that
            gripperJointPositions.resize(numberOfGripperJoints);

            std::stringstream jointName;


            armJointPositions[0].value = MainWindow::joint_1;
            armJointPositions[1].value = MainWindow::joint_2;
            armJointPositions[2].value = MainWindow::joint_3;
            armJointPositions[3].value = MainWindow::joint_4;
            armJointPositions[4].value = MainWindow::joint_5;
            //gripperJointPositions[0].value = gripper_1;
            //gripperJointPositions[1].value = gripper_2;
            for (int i = 0; i < numberOfArmJoints; ++i) {
            //cout << "Please type in value for joint " << i + 1 << endl;
            //cin >> readValue;

            jointName.str("");
            jointName << "arm_joint_" << (i + 1);

            armJointPositions[i].joint_uri = jointName.str();
            //armJointPositions[i].value = readValue;

            armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
            //cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;
            };


		std_msgs::String msg;
		std::stringstream ss;
                double zmienna = MainWindow::joint_1;
                ss << "hello world " << count << zmienna;
		msg.data = ss.str();
                //chatter_publisher.publish(msg);
                command.positions = armJointPositions;
                armPositionsPublisher.publish(command);
                //log(Info,std::string("I sent: ")+msg.data);
                //MainWindow::refresh_value(true);
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
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
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

}  // namespace youbot_gui
