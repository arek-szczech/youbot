Installation
------------

### System requirements: 
* Linux 16.04 LTS
* ROS Kinetic
* Ethernet adapter 
* Root access to the Ethernet adapter 

These libraries are required by the youBot GUI: 

* [Simple Open EtherCAT master](http://soem.berlios.de) (included in the driver)
* [Boost C++ Libraries](http://www.boost.org)
* [ncurses library]


### Manual installation:
First step is to install ROS Kinetic Framework.
Setup your sources.list:

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

Set up your keys:

    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

Install ROS Kinetic:

    sudo apt-get update
    sudo apt-get install ros-kinetic-desktop-full

Initialize rosdep:

    sudo rosdep init
    rosdep update

Setup your environment:

    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Second step is to install additional libraries and programs.
Install GKSUDO:

    sudo apt-get install gksu

Install pr2-msgs package:

    sudo apt-get install ros-kinetic-pr2-msgs

Install ncurses library:

    sudo apt-get install libncurses-dev

Final step is to download youbot_gui repository and compile it.

Create and initialize catkin workspace (i. e. youbot_gui):

    mkdir -p ~/youbot_gui/src
    cd ~/youbot_gui/src
    catkin_init_workspace
    cd ..
    catkin_make

Clone the youBot GUI sources to /src folder:
    
    cd ~/youbot_gui/src
    git clone https://github.com/arek-szczech/youbot.git

Add the repository folder to the ROS_PACKAGE_PATH environment variable.

Compile the youbot GUI by typing:

    cd ~/youbot_gui
    catkin_make

Usage
------------

The youBot GUI give you  graphical user interface app to communicate with youBot arm via youbot_driver. You can move youBot arm with joints or in the global coordinate system, create list of points and create automatic program with PTP or LIN movements between these points. You can open and close youBot gripper in automatic or manual mode.

To run youbot_gui you need to follow this commands:

    cd ~/youbot_gui
    source devel/setup.bash
    rosrun youbot_gui youbot_gui

License
------------

This software is published under a dual-license: GNU Lesser General Public
License LGPL 2.1 and BSD license. The dual-license implies that users of this
code may choose which terms they prefer.

youBot's ROS packages were downloaded from official youBot repository.
Our GUI is based on qt_ros package's example.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License LGPL and the BSD license for more details.
